cmake_minimum_required(VERSION 2.8.3)
project(joint_trajectory)

find_package(catkin
  REQUIRED
  rospy
  actionlib
  control_msgs
  trajectory_msgs
  baxter_interface
  utilities
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
  actionlib
  control_msgs
  trajectory_msgs
  baxter_interface
  utilities
)
