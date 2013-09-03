cmake_minimum_required(VERSION 2.8.3)
project(baxter_interface)

find_package(catkin
  REQUIRED
  rospy
  actionlib
  sensor_msgs
  std_msgs
  control_msgs
  trajectory_msgs
  dynamic_reconfigure
  baxter_core_msgs
  utilities
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/JointTrajectoryActionServer.cfg
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  actionlib
  sensor_msgs
  std_msgs
  control_msgs
  trajectory_msgs
  dynamic_reconfigure
  baxter_core_msgs
  utilities
)
