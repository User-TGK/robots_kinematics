#include <iostream>
#include <string>

#include <fstream>
#include <streambuf>

#include <chrono>
#include <robot_arm_interface/ArmControlServer.h>
#include <robot_arm_interface/RobotArm.h>
#include <ros/package.h>
#include <thread>

int main(int argc, char** argv)
{
	const std::string progName = "robot_arm_interface";
	const std::uint8_t feedbackRate = 100;

	ros::init(argc, argv, progName);

	std::cout << ros::package::getPath(progName) << std::endl;

	std::ifstream inRobotArmConfig(
		ros::package::getPath(progName) + "/resources/robotArm.json");
	std::string robotArmConfigJson(
		(std::istreambuf_iterator<char>(inRobotArmConfig)),
		std::istreambuf_iterator<char>());
	std::ifstream inSsc32uConfig(
		ros::package::getPath(progName) + "/resources/ssc32u.json");
	std::string ssc32uConfigJson(
		(std::istreambuf_iterator<char>(inSsc32uConfig)),
		std::istreambuf_iterator<char>());

	ArmControl::RobotArm robotArm(robotArmConfigJson, ssc32uConfigJson);

	ArmControl::RobotArmDestination destination;

	destination.positions = robotArm.GetParkPosition();
	destination.timeToReachDestination = 2000;
	robotArm.Move(destination);
	do {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	} while (robotArm.IsMoving() == true);

	destination.positions = robotArm.GetReadyPosition();
	destination.timeToReachDestination = 2000;
	robotArm.Move(destination);
	do {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	} while (robotArm.IsMoving() == true);

	ArmControl::ArmControlServer server(progName, robotArm, feedbackRate);

	ros::spin();
	return 0;
}