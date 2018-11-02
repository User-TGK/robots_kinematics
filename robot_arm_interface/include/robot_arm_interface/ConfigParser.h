#ifndef ROBOT_ARM_INTERFACE_CONFIGPARSER_H
#define ROBOT_ARM_INTERFACE_CONFIGPARSER_H

#include <cstdint>
#include <string>

namespace ArmControl
{
/// \brief Contains setting of a single joint of the robot arm.
struct JointPositionSetting
{
	/// \brief The channel this joint is associated with.
	std::uint8_t channel;

	/// \brief THe minimum pwm.
	std::uint16_t minPWM;

	/// \brief The maximum pwm.
	std::uint16_t maxPWM;

	/// \brief The minimum degrees this maps to.
	std::int16_t minDegree;

	/// \brief The maximum degrees this maps to.
	std::int16_t maxDegree;
};

/// \brief Contains setting of a single joint of the robot arm.
struct GripperPositionSetting
{
	/// \brief The channel this joint is associated with.
	std::uint8_t channel;

	/// \brief THe minimum pwm.
	std::uint16_t minPWM;

	/// \brief The maximum pwm.
	std::uint16_t maxPWM;

	/// \brief The minimum width between gripper points this maps to.
	std::int16_t minMm;

	/// \brief The maximum width between gripper points this maps to.
	std::int16_t maxMm;
};

struct JointPositionConfig
{
	/// \brief Construct from json string
	/// \param jsonData The json string.
	JointPositionConfig(const std::string &jsonData);

	/// \brief Setting for each joint.
	JointPositionSetting base;
	JointPositionSetting shoulder;
	JointPositionSetting elbow;
	JointPositionSetting wrist;
	GripperPositionSetting gripper;
	JointPositionSetting wristRotate;
};
}

#endif // PROJECT_CONFIGPARSER_H
