cmake_minimum_required(VERSION 2.8.3)
project(tools)

find_package(catkin
  REQUIRED
  rospy
  sensor_msgs
  geometry_msgs
  cv_bridge
  baxter_interface
  baxter_msgs
  utilities
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
  sensor_msgs
  geometry_msgs
  cv_bridge
  baxter_interface
  baxter_msgs
  utilities
)
