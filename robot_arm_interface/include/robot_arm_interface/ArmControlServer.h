#ifndef ROBOT_ARM_INTERFACE_ARMCONTROLSERVER_H
#define ROBOT_ARM_INTERFACE_ARMCONTROLSERVER_H

#include <atomic>
#include <string>

#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>

#include <robot_arm_interface/ArmControlAction.h>
#include <actionlib/server/simple_action_server.h>
#include <robot_arm_interface/EmergencyStop.h>
#include <robot_arm_interface/GetArmState.h>
#include <robot_arm_interface/RobotArm.h>

namespace ArmControl
{
/// \brief arm control action lib server.
class ArmControlServer
{
public:
	/// \brief Construct with robot arm.
	/// \param actionName The name of this server.
	/// \param feedbackRate The amount of time between feedbacks. in
	/// milliseconds.
	ArmControlServer(const std::string &serverName, RobotArm &robotArm,
		std::uint8_t feedbackRate);

	/// \param robotArm The robot arm this action server controls.

	/// \brief Destructor.
	~ArmControlServer();

	/// \brief Delete default constructor since server and arm interface are
	/// required.
	ArmControlServer() = delete;

	/// \brief Delete copy constructor. Because server shouldn't be copied.
	/// \param that
	ArmControlServer(const ArmControlServer &that) = delete;

	/// \brief Delete copy assignment operator. Because server shouldn't be
	/// copied \param that \return
	ArmControlServer &operator=(ArmControlServer &that) = delete;

private:
	/// \brief Fill out an armstate message with current data.
	/// \param state The state to fill out.
	void FillOutArmState(robot_arm_interface::ArmState &state);

	/// \brief Server was called with a goal.
	/// \param goal The goal to reach.
	void GoalRecieved(
		const robot_arm_interface::ArmControlGoalConstPtr &goal);

	/// \brief function to handle the service call get the current robot arm
	/// state. \param req The request data. In this case there is no
	/// request. \param res The response to write back. The current state.
	bool GetRobotArmState(robot_arm_interface::GetArmState::Request &req,
		robot_arm_interface::GetArmState::Response &res);

	/// \brief Stop the robot arm as soon as possible.
	/// \param req The request data. In this case there is no requested
	/// date. \param res The response to write back. No response in this
	/// case.
	bool EmergencyStop(robot_arm_interface::EmergencyStop::Request &req,
		robot_arm_interface::EmergencyStop::Response &res);

	/// \brief Boolean whether the arm is an emergency stop state.
	std::atomic<bool> emergencyStopped;

	/// \brief Tge action name.
	const std::string serverName;

	/// \brief the robotArm interface.
	RobotArm &robotArm;

	/// \brief The ROS node handle
	ros::NodeHandle nodeHandle;

	/// \brief The action server.
	actionlib::SimpleActionServer<robot_arm_interface::ArmControlAction>
		actionServer;

	/// \brief The service server to get the current robot arm state.
	ros::ServiceServer robotArmStateService;

	/// \brief The service server to get the current robot arm state.
	ros::ServiceServer robotArmEmergencyStop;

	/// \brief The publisher server that publishes the current robot arm
	/// state feedbackRate times per second.
	ros::Publisher robotArmStatePublisher;

	/// \brief The time between feedbacks in milliseconds.
	const std::uint8_t feedbackRate;
};
}

#endif // PROJECT_ARMCONTROLSERVER_H
