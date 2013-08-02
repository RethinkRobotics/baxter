cmake_minimum_required(VERSION 2.8.3)
project(head_control)

find_package(catkin
  REQUIRED
  rospy
  cv_bridge
  baxter_interface
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
  cv_bridge
  baxter_interface
)
