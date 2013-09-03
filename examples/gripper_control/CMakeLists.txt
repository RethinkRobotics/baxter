cmake_minimum_required(VERSION 2.8.3)
project(gripper_control)

find_package(catkin
  REQUIRED
  rospy
  sensor_msgs
  baxter_core_msgs
  baxter_interface
  utilities
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
  sensor_msgs
  baxter_core_msgs
  baxter_interface
  utilities
)
