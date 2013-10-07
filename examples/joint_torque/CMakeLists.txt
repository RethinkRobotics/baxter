cmake_minimum_required(VERSION 2.8.3)
project(joint_torque)

find_package(catkin
  REQUIRED
  rospy
  baxter_interface
  utilities
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/JointSpringsExample.cfg
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  baxter_interface
  utilities
)
