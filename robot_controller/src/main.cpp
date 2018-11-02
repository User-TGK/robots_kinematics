#include <iostream>

#include "ros/ros.h"

#include "Matrix.h"
#include "RobotController.h"

int main(int argc, char **argv)
{
	const std::string prog_name = "robot_controller";
	ros::init(argc, argv, prog_name);

	robot_controller::RobotController server(prog_name);

	ros::spin();
	return 0;

	return 0;
}