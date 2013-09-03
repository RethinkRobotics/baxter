cmake_minimum_required(VERSION 2.8.3)
project(joint_position)

find_package(catkin
  REQUIRED
  rospy
  baxter_interface
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
  baxter_interface
)
