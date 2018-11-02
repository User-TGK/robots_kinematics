#ifndef ROBOT_ARM_INTERFACE_ROBOTARM_H
#define ROBOT_ARM_INTERFACE_ROBOTARM_H

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

#include <robot_arm_interface/ConfigParser.h>
#include <ssc32u/Driver.h>

namespace ArmControl
{
/// \brief The state of a single joint position. Default value of the speed is
/// the maximum it wil allow.
struct JointPosition
{
	/// \brief The degree to set the joint to.
	double degree = 0;
	/// \brief The max degrees per second it will allow during the move.
	double maxDegreesPerSecond = 0;
};

/// \brief The state of a single joint position. Default value of the speed is
/// the maximum it wil allow.
struct GripperPosition
{
	/// \brief The distance between the gripper grips
	double distance = 0;
	/// \brief The max milllimeters per second it will allow during the
	/// move.
	double maxMmPerSecond = 0;
};

/// \brief Struct containing all joint positions as degrees.
struct JointPositions
{
	JointPosition base;
	JointPosition shoulder;
	JointPosition elbow;
	JointPosition wrist;
	GripperPosition gripper;
	JointPosition wristRotate;
};

/// \brief Struct for the information to send a destination specification to the
/// robot arm.
struct RobotArmDestination
{
	/// \brief All positions
	JointPositions positions;

	/// \brief Time in microseconds to reach the target.
	uint16_t timeToReachDestination = 0;
};

/// \brief High level class for interfacing with the robot arm.
class RobotArm
{
public:
	/// \brief Get the predefined park position.
	/// \return The position.
	static const JointPositions &GetParkPosition();

	/// \brief Get the predefined ready position.
	/// \return The position.
	static const JointPositions &GetReadyPosition();

	/// \brief Get the predefined straight up position.
	/// \return The position.
	static const JointPositions &GetStraightUpPosition();

	/// \brief Get the maximum degrees per second of each joint the robotarm
	/// supports. \return JointPositions filled out with the max degrees per
	/// second.
	static const JointPositions &GetMaxDegreesPerSecond();

	/// \brief Construct robotArm instance with configuration data.
	/// \param robotArmConfigJson The json configuration for the robot arm.
	/// \param driverConfigJson The json configuration for the low level
	/// driver.
	RobotArm(const std::string &robotArmConfigJson,
		const std::string &driverConfigJson);

	/// \brief Deleted default constructor.
	RobotArm() = delete;

	/// \brief Destructor
	virtual ~RobotArm() = default;

	/// \brief Delete copy constructor. Because low level driver shouldn't
	/// be copied. \param that The robot arm to instantiate the current.
	RobotArm(const RobotArm &that) = delete;

	/// \brief Delete copy assignment operator. Because low level driver
	/// shouldn't be copied \return
	RobotArm &operator=(const RobotArm &) = delete;

	/// \brief Get feedback for current position of the arm. ma
	/// \return Struct containing the joint position of each joint
	JointPositions GetCurrentJointPositions() const;

	/// \brief Returns whether the robot arm is currently moving.
	/// \return True if moving false if not/
	bool IsMoving() const;

	/// \brief Move the robot arm to the desired destination.
	/// \param destination The destination to move the robot arm to.
	void Move(const RobotArmDestination &destination);

	/// \brief Stop the robot arm as fast as possible.
	void Stop();

	// \brief Check if the desired position exceeds the max speed.
	// \param destination The desired destination.
	// \return True if max speed is exceeded.
	bool ExceedsMaxSpeed(const RobotArmDestination &destination);

private:
	/// \brief Contains the configuration for each joint.
	const JointPositionConfig config;

	/// \brief Low level ssc32 driver.
	ssc32drv::SSC32 driver;
};
}

#endif // PROJECT_ROBOT_ARM_HIGHLEVEL_INTERFACE_H
