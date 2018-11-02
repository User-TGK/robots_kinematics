#include <robot_arm_interface/ConfigParser.h>

#include <boost/iostreams/device/array.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace ArmControl
{
namespace pt = boost::property_tree;

/// \brief Parse a single Joint position setting from the full ptree.
/// \param key The base key to get the settings for.
/// \param tree The tree where the settings are located in.
/// \return The constructed JointPosition settings.
JointPositionSetting GetJointSettingsFromTree(
	const std::string &key, const pt::ptree &tree)
{
	return JointPositionSetting{
		tree.get<std::uint8_t>(key + ".channel"),
		tree.get<std::uint16_t>(key + ".minPWM"),
		tree.get<std::uint16_t>(key + ".maxPWM"),
		tree.get<std::int16_t>(key + ".minDegree"),
		tree.get<std::int16_t>(key + ".maxDegree"),
	};
}

/// \brief Parse a single Gripper position setting from the full ptree.
/// \param key The base key to get the settings for.
/// \param tree The tree where the settings are located in.
/// \return The constructed GripperPosition settings.
GripperPositionSetting GetGripperSettingsFromTree(
	const std::string &key, const pt::ptree &tree)
{
	return GripperPositionSetting{
		tree.get<std::uint8_t>(key + ".channel"),
		tree.get<std::uint16_t>(key + ".minPWM"),
		tree.get<std::uint16_t>(key + ".maxPWM"),
		tree.get<std::int16_t>(key + ".minMm"),
		tree.get<std::int16_t>(key + ".maxMm"),
	};
}

JointPositionConfig::JointPositionConfig(const std::string &jsonData)
{
	pt::ptree tree;

	boost::iostreams::array_source as(&jsonData[0], jsonData.size());
	boost::iostreams::stream<boost::iostreams::array_source> is(as);

	pt::read_json(is, tree);

	this->base = GetJointSettingsFromTree("base", tree);
	this->shoulder = GetJointSettingsFromTree("shoulder", tree);
	this->elbow = GetJointSettingsFromTree("elbow", tree);
	this->wrist = GetJointSettingsFromTree("wrist", tree);
	this->gripper = GetGripperSettingsFromTree("gripper", tree);
	this->wristRotate = GetJointSettingsFromTree("wristRotate", tree);
}
}