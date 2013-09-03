cmake_minimum_required(VERSION 2.8.3)
project(input_output)

find_package(catkin
  REQUIRED
  rospy
  baxter_core_msgs
  baxter_interface
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
  baxter_core_msgs
  baxter_interface
)
