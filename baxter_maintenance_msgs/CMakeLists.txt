cmake_minimum_required(VERSION 2.8.3)
project(baxter_maintenance_msgs)

find_package(catkin REQUIRED
  message_generation
  std_msgs
)

add_message_files(DIRECTORY msg FILES
  CalibrateArmData.msg
  CalibrateArmEnable.msg
  TareData.msg
  TareEnable.msg
  UpdateSource.msg
  UpdateSources.msg
  UpdateStatus.msg
)

add_service_files(DIRECTORY srv FILES
  LSCores.srv
  RMCores.srv
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
)
