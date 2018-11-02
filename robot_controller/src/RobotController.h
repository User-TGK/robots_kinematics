#ifndef ROBOT_CONTROLLER_ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_ROBOT_CONTROLLER_H

#include <string>

#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <robot_controller/ControlArm.h>
#include <robot_controller/MoveObject.h>

#include <robot_arm_interface/ArmControlAction.h>

#include <geometry_msgs/Twist.h>

#include "Matrix.h"

namespace robot_controller
{
/**
 * \brief arm control action lib server.
 */
class RobotController
{
public:
	/**
	 * \brief Construct a new Robot Controller object
	 * \param serverName
	 */
	RobotController(const std::string& serverName);

	/**
	 * \brief Destroy the Robot Controller object
	 */
	~RobotController();

	/**
	 * \brief Deleted default constructor
	 */
	RobotController() = delete;

	/**
	 * \brief Deleted copy constructor. Because server shouldn't be copied.
	 * \param that
	 */
	RobotController(const RobotController& that) = delete;

	/**
	 * \brief Delete copy assignment operator.
	 * \param that
	 * \return
	 */
	RobotController& operator=(RobotController& that) = delete;

private:
	/**
	 * \brief Get the angle the wrist has to stand in to be able to pick up the object.
	 * \param vertices The vertices of the object
	 * \return The angle in degrees the wrist has to stand
	 */
	double calculateRequiredWristAngle(uint8_t shape,
		const std::vector<geometry_msgs::Point>& vertices) const;

	/**
	 * \brief Move robot arm joints to the provided angles.
	 * \param angles The angles to move towards
	 * \return Blocks untill finished
	 */
	void moveArm(const Matrix<double, 6, 1>& angles);

	/**
	 * \brief Control the robot arm directly to command it to go to positon
	 * \param req Contains the point to command the arm to.
	 * \param res Empty response
	 * \return True if succesfull, false if failed due to that the robot arm cannot reach.
	 */
	bool controlArm(robot_controller::ControlArm::Request& req,
		robot_controller::ControlArm::Response& res);

	/**
	 * \brief function to handle the service call to move an object to a position.
	 * \param req The request data. Contains data where the object to move is. What object it is and the goal
	 * \param res The response object. This is empty in this case
	 */
	bool moveObject(robot_controller::MoveObject::Request& req,
		robot_controller::MoveObject::Response& res);

	/**
	 * \brief The ROS node handle
	 */
	ros::NodeHandle nodeHandle;

	/// \brief The service server to move the  obhect
	ros::ServiceServer moveObjectService;

	/// \brief The service server to move the  obhect
	ros::ServiceServer controlArmService;

	/// \brief The action client to call into the robot arm interface
	actionlib::SimpleActionClient<robot_arm_interface::ArmControlAction>
		armControl;
};
}

#endif // ROBOT_CONTROLLER_ROBOT_CONTROLLER_H
