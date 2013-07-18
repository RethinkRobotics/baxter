cmake_minimum_required(VERSION 2.8.3)
project(utilities)

find_package(catkin REQUIRED rospy)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
)
