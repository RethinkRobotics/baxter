cmake_minimum_required(VERSION 2.8.3)
project(baxter_tools)

find_package(catkin
  REQUIRED
  rospy
  sensor_msgs
  geometry_msgs
  cv_bridge
  baxter_interface
  baxter_core_msgs
  baxter_maintenance_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
  sensor_msgs
  geometry_msgs
  cv_bridge
  baxter_interface
  baxter_core_msgs
  baxter_maintenance_msgs
)

install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY share/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/share
  USE_SOURCE_PERMISSIONS
)
