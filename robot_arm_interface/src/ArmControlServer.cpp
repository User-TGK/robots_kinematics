#include <cstdint>
#include <functional>
#include <robot_arm_interface/ArmControlServer.h>
#include <robot_arm_interface/RobotArm.h>

namespace ArmControl
{
ArmControlServer::ArmControlServer(const std::string& serverName,
	RobotArm& robotArm, std::uint8_t feedbackRate) :
	emergencyStopped(false),
	serverName(serverName), robotArm(robotArm),
	actionServer(nodeHandle, serverName + "_set_goal",
		std::bind(&ArmControlServer::GoalRecieved, this,
			std::placeholders::_1),
		false),
	feedbackRate(feedbackRate)
{
	this->robotArmStateService =
		this->nodeHandle.advertiseService(serverName + "_get_state",
			&ArmControlServer::GetRobotArmState, this);

	this->robotArmEmergencyStop = this->nodeHandle.advertiseService(
		serverName + "_emergency_stop",
		&ArmControlServer::EmergencyStop, this);

	this->robotArmStatePublisher =
		this->nodeHandle.advertise<robot_arm_interface::ArmState>(
			serverName + "_observe_state", 1000);

	actionServer.start();
}

ArmControlServer::~ArmControlServer()
{
}

void ArmControlServer::FillOutArmState(robot_arm_interface::ArmState& state)
{
	auto currentPositions = this->robotArm.GetCurrentJointPositions();
	state.now = ros::Time::now();
	state.isMoving = this->robotArm.IsMoving();
	state.pose.base.degree = currentPositions.base.degree;
	state.pose.shoulder.degree = currentPositions.shoulder.degree;
	state.pose.elbow.degree = currentPositions.elbow.degree;
	state.pose.wrist.degree = currentPositions.wrist.degree;
	state.pose.gripper.distance = currentPositions.gripper.distance;
	state.pose.wristRotate.degree = currentPositions.wristRotate.degree;
}

void ArmControlServer::GoalRecieved(
	const robot_arm_interface::ArmControlGoalConstPtr& goal)
{
	robot_arm_interface::ArmState currentState;
	robot_arm_interface::ArmControlFeedback feedback;
	robot_arm_interface::ArmControlResult result;

	this->FillOutArmState(currentState);
	feedback.state = currentState;
	result.state = currentState;

	if (this->emergencyStopped) {
		ROS_WARN("ARM is emergency stopped state can't move.");
		this->actionServer.setAborted(
			result, "Robot arm is in emergency stop state.");
		return;
	}

	ros::Rate r(1000 / this->feedbackRate);

	/// Check if predefined goal was used or a raw angle.
	RobotArmDestination destination;
	if (goal->goalType
		== robot_arm_interface::ArmControlGoal::TYPE_PREDEFINED) {
		switch (goal->goal) {
		case robot_arm_interface::ArmControlGoal::GOAL_PARK:
			destination.positions = RobotArm::GetParkPosition();
			break;
		case robot_arm_interface::ArmControlGoal::GOAL_READY:
			destination.positions = RobotArm::GetReadyPosition();
			break;
		case robot_arm_interface::ArmControlGoal::GOAL_STRAIGHT_UP:
			destination.positions =
				RobotArm::GetStraightUpPosition();
			break;
		default:
			ROS_WARN("redefined goal not found.");
			this->actionServer.setAborted(
				result, "Predefined goal not found.");
			return;
		}
	} else if (goal->goalType
		== robot_arm_interface::ArmControlGoal::TYPE_RAW) {
		destination.positions.base.degree = goal->pose.base.degree;
		destination.positions.shoulder.degree =
			goal->pose.shoulder.degree;
		destination.positions.elbow.degree = goal->pose.elbow.degree;
		destination.positions.gripper.distance =
			goal->pose.gripper.distance;
		destination.positions.wrist.degree = goal->pose.wrist.degree;
		destination.positions.wristRotate.degree =
			goal->pose.wristRotate.degree;
	} else {
		ROS_WARN("Invalid goalType provided.");
		this->actionServer.setAborted(
			result, "Invalid goalType provided.");
		return;
	}

	destination.positions.base.maxDegreesPerSecond =
		goal->pose.base.maxDegreesPerSecond;
	destination.positions.shoulder.maxDegreesPerSecond =
		goal->pose.shoulder.maxDegreesPerSecond;
	destination.positions.elbow.maxDegreesPerSecond =
		goal->pose.elbow.maxDegreesPerSecond;
	destination.positions.gripper.maxMmPerSecond =
		goal->pose.gripper.maxMmPerSecond;
	destination.positions.wrist.maxDegreesPerSecond =
		goal->pose.wrist.maxDegreesPerSecond;
	destination.positions.wristRotate.maxDegreesPerSecond =
		goal->pose.wristRotate.maxDegreesPerSecond;

	uint64_t timeToArrive = (goal->arrivalTime.sec * 1000)
		+ (goal->arrivalTime.nsec / 1000000);

	if (goal->arrivalTime.sec >= 66 || timeToArrive > UINT16_MAX) {
		ROS_WARN("timeToArrive is to long. Max 65536 milliseconds is "
			 "allowed.");
		this->actionServer.setAborted(result,
			"timeToArrive is to long. Max 65536 milliseconds is "
			"allowed.");
		return;
	}

	destination.timeToReachDestination =
		static_cast<uint16_t>(timeToArrive);

	if (this->robotArm.ExceedsMaxSpeed(destination)) {
		// ROS_WARN("Desired destination exceeds maximum speed of "
		//	 "robotArm, requested QOS possibly won't be reached.");
	}

	try {
		this->robotArm.Move(destination);
	} catch (const std::out_of_range& ex) {
		ROS_WARN("Robot arm out of reach.");
		std::cout << goal->pose << std::endl;
		this->actionServer.setAborted(
			result, "Desired robot arm position is out of reach.");
		return;
	}

	bool moving = this->robotArm.IsMoving();
	while (moving) {
		r.sleep();
		if (this->actionServer.isPreemptRequested() || !ros::ok()) {
			this->robotArm.Stop();
			this->actionServer.setPreempted();
			return;
		}

		if (this->emergencyStopped == true) {
			result.state = feedback.state;
			this->actionServer.setAborted(
				result, "Emergency stop was called.");
			return;
		}

		moving = this->robotArm.IsMoving();

		this->FillOutArmState(currentState);
		feedback.state = currentState;
		this->actionServer.publishFeedback(feedback);
		this->robotArmStatePublisher.publish(currentState);
	}

	result.state = currentState;

	this->actionServer.setSucceeded(result, "Goal reached");
}

bool ArmControlServer::GetRobotArmState(
	robot_arm_interface::GetArmState::Request&,
	robot_arm_interface::GetArmState::Response& res)
{
	res.emergencyStopped = this->emergencyStopped;
	this->FillOutArmState(res.state);
	return true;
}

bool ArmControlServer::EmergencyStop(
	robot_arm_interface::EmergencyStop::Request& req,
	robot_arm_interface::EmergencyStop::Response&)
{
	this->robotArm.Stop();
	this->emergencyStopped = req.stopState == 1;

	return true;
}
}