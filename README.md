## Robotics Lab Homework2

This repository contains the packages required for the homework 2.
Follow the instructions to run the packages properly.

## Build and Source the packages

Clone the repository in your ROS2 workspace and build the packages.
build
bash
colcon build

then source

bash
source install/setup.bash


## use effort commands
By default the node publishes joint position commands. To use the effort commands

robot must be launched with the effort interface
bash
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="effort" robot_controller:="effort_controller"


in an other terminal start the node
bash
ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=effort

you can choose between the linear or the circular trajectory with the trapezoidal velocity or the cubic polinomial.
