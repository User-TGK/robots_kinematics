

#include "RobotController.h"

#include "Kinematics.h"

namespace robot_controller
{
RobotController::RobotController(const std::string& serverName) :
	armControl("robot_arm_interface_set_goal", true)
{
	ROS_INFO("Waiting for robot_arm_interface action server...");
	this->armControl.waitForServer();
	this->moveObjectService =
		this->nodeHandle.advertiseService(serverName + "_move_object",
			&RobotController::moveObject, this);
	this->controlArmService =
		this->nodeHandle.advertiseService(serverName + "_control_arm",
			&RobotController::controlArm, this);

	ROS_INFO("robot_arm_interface found. Started robot_controller.");
}

RobotController::~RobotController()
{
}

void RobotController::moveArm(const Matrix<double, 6, 1>& angles)
{
	robot_arm_interface::ArmControlGoal goal;

	goal.goalType = robot_arm_interface::ArmControlGoal::TYPE_RAW;
	goal.pose.base.degree = angles[0][0];
	goal.pose.base.maxDegreesPerSecond = 30;
	goal.pose.shoulder.degree = angles[1][0];
	goal.pose.shoulder.maxDegreesPerSecond = 30;
	goal.pose.elbow.degree = angles[2][0];
	goal.pose.elbow.maxDegreesPerSecond = 30;
	goal.pose.wrist.degree = angles[3][0];
	goal.pose.wrist.maxDegreesPerSecond = 30;
	goal.pose.gripper.distance = angles[4][0];
	goal.pose.wristRotate.degree = angles[5][0];
	goal.pose.wristRotate.maxDegreesPerSecond = 30;

	this->armControl.sendGoalAndWait(goal);
}

double RobotController::calculateRequiredWristAngle(
	uint8_t shape, const std::vector<geometry_msgs::Point>& vertices) const
{
	if (shape == robot_controller::MoveObject::Request::CIRCLE) {
		return 0;
	}

	// find longest edge since that is the edge that has to stand parallel
	// to gripper
	double dx = 0, dz = 0;
	double magnitude = 0;

	for (size_t i = 0; i < vertices.size(); ++i) {
		const auto& v1 = vertices[i];
		const auto& v2 = vertices[(i + 1) % vertices.size()];
		const auto& va = v1.x > v2.x ? v1 : v2;
		const auto& vb = v1.x > v2.x ? v2 : v1;

		auto test_dx = va.x - vb.x;
		auto test_dz = va.z - vb.z;

		auto test_magnitude = std::hypot(test_dx, test_dz);
		if (test_magnitude > magnitude) {
			dx = test_dx;
			dz = test_dz;
			magnitude = test_magnitude;
		}
	}

	const auto angle = (180 * std::atan2(dz, dx)) / M_PI;

	std::cout << "dx: " << dx << ", dy: " << dz << ", angle: " << angle
		  << std::endl;

	return angle;
}

bool RobotController::moveObject(robot_controller::MoveObject::Request& req,
	robot_controller::MoveObject::Response&)
{
	ROS_INFO("Recieved moveObject command");

	// Convert ros message to matrices that are usable for
	// kinematics modules.
	const Matrix<double, 3, 1> shape_pos(
		{{req.shape_pos.x}, {req.shape_pos.y}, {req.shape_pos.z}});
	const Matrix<double, 3, 1> target_pos(
		{{req.target_pos.x}, {req.target_pos.y}, {req.target_pos.z}});

	std::cout << "Trying to goto goal: " << std::endl
		  << shape_pos << std::endl;

	// get angle the wrist has to stand in to pick up the object
	auto wrist_rotate = this->calculateRequiredWristAngle(
		req.shape, req.shape_vertices);

	// How much the arm moves down
	// to get the block
	const double moveDown = 0.035;

	// Create matrices of all targets
	// 1. above the shape
	// 2. Move down towards shape
	// 3. Grip shape
	// 4. Move above shape
	// 5. Move above target
	// 6. Move down into the target
	// 7. open gripper
	// 8. Move over target
	// 9. goto ready position
	auto above_shape_pos = shape_pos;
	above_shape_pos[1][0] += moveDown;
	auto above_target_pos = target_pos;
	above_target_pos[1][0] += moveDown;

	std::cout << "wrist rotate: " << wrist_rotate << std::endl;

	auto above_shape_angles_opt =
		Kinematics::inverse(above_shape_pos, 90.0, wrist_rotate);
	auto shape_angles_opt =
		Kinematics::inverse(shape_pos, 90.0, wrist_rotate);
	auto above_target_angles_opt =
		Kinematics::inverse(above_target_pos, 90.0, 0);
	auto target_angles_opt = Kinematics::inverse(target_pos, 90.0, 0);

	bool trajectory_invalid = false;
	if (above_shape_angles_opt.has_value() == false) {
		trajectory_invalid = true;
		ROS_WARN("No inverse to reach above the shape");
	}

	if (shape_angles_opt.has_value() == false) {
		trajectory_invalid = true;
		ROS_WARN("No inverse to reach shape");
	}

	if (above_target_angles_opt.has_value() == false) {
		trajectory_invalid = true;
		ROS_WARN("No inverse to reach above the target");
	}

	if (target_angles_opt.has_value() == false) {
		trajectory_invalid = true;
		ROS_WARN("No inverse to reach the target");
	}

	if (trajectory_invalid == true) {
		return false;
	}

	auto& above_shape_angles = above_shape_angles_opt.value();
	auto& shape_angles = shape_angles_opt.value();
	auto& above_target_angles = above_target_angles_opt.value();
	auto& target_angles = target_angles_opt.value();

	// joint 4 is the gripper. 31 is open. 0 is closed
	above_shape_angles[4][0] = 31;
	shape_angles[4][0] = 31;

	this->moveArm(above_shape_angles);
	ros::Duration(3).sleep();
	this->moveArm(shape_angles);
	ros::Duration(0.5).sleep();

	// Next movement has to be done with closed gripper since object should
	// be picked up
	shape_angles[4][0] = 0;
	above_shape_angles[4][0] = 0;
	above_target_angles[4][0] = 0;
	target_angles[4][0] = 0;

	this->moveArm(shape_angles);
	this->moveArm(above_shape_angles);
	this->moveArm(above_target_angles);

	ros::Duration(0.5).sleep();
	this->moveArm(target_angles);
	ros::Duration(1).sleep();

	// Open gripper
	target_angles[4][0] = 31;
	above_target_angles[4][0] = 31;
	this->moveArm(target_angles);
	this->moveArm(above_target_angles);

	robot_arm_interface::ArmControlGoal goal;
	goal.goalType = robot_arm_interface::ArmControlGoal::TYPE_PREDEFINED;
	goal.goal = robot_arm_interface::ArmControlGoal::GOAL_READY;
	goal.arrivalTime = ros::Duration(2);

	this->armControl.sendGoalAndWait(goal);

	return true;
}

bool RobotController::controlArm(robot_controller::ControlArm::Request& req,
	robot_controller::ControlArm::Response&)
{
	const Matrix<double, 3, 1> target_mat(
		{{req.target.x}, {req.target.y}, {req.target.z}});

	const auto target_angles_opt = Kinematics::inverse(
		target_mat, req.gripper_angle, req.wrist_rotate);

	if (target_angles_opt.has_value() == false) {
		ROS_WARN("Target cannot be reached.");
		return false;
	}

	std::cout << "Moving arm to state: " << target_angles_opt.value()
		  << std::endl;

	this->moveArm(target_angles_opt.value());

	return true;
}
}
