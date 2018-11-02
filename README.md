# World of Robots - Robots

## Authors: Rowan Goemans & Ties Klappe
## Date: 01-11-2018

## To build the application the following prerequisites have to be met

1. Operating system: Linux Debian Stable or Ubuntu 17.04/18.04.1 LTS
2. Installed GCC compiler, version 7.3 or higher
3. OpenCV 3.0 or higher
4. ROS Melodic Morenia or ROS Lunar installed

## Build instructions

1. Create a ROS workspace (see this [tutorial](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) for more information)
2. Clone the contents of this repository into the source (src) folder of your workspace
3. Build typing catkin_make in the top directory of your workspace

## Run instructions

Assuming the AL5D and camera are attached to your device:

Normal procedure

1. Open a new (sourced) terminal and run `rosrun robot_arm_interface robot_arm_interface`
2. Open a new (sourced) terminal and run `rosrun robot_controller robot_controller`
3. Open a new (sourced) terminal and run `rosrun robot_vision robot_vision`

By typing commands in the robot_vision terminal, such as 'vierkant geel', the application will try to locate an object that satisfies the properties by the given command and move it to the target location

Calibration

1. Open a new (sourced) terminal and run `rosrun robot_vision robot_vision calibrate mode`

Note: the `-Wconversion` option was removed from the build options because ROS gives errors when building with this option. The code written by the students however, does not give any errors on the `-Wconversion` option.