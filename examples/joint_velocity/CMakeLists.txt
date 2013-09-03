cmake_minimum_required(VERSION 2.8.3)
project(joint_velocity)

find_package(catkin
  REQUIRED
  rospy
  baxter_interface
  utilities
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
  baxter_interface
  utilities
)
