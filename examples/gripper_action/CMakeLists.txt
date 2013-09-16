cmake_minimum_required(VERSION 2.8.3)
project(gripper_action)

find_package(catkin
  REQUIRED
  rospy
  actionlib
  control_msgs
  baxter_interface
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
  actionlib
  control_msgs
  baxter_interface
)
