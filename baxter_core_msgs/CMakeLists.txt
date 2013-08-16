cmake_minimum_required(VERSION 2.8.3)
project(baxter_core_msgs)

find_package(catkin REQUIRED
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(DIRECTORY msg FILES
  AnalogIOState.msg
  AnalogIOStates.msg
  AnalogOutputCommand.msg
  AssemblyState.msg
  AssemblyStates.msg
  CameraControl.msg
  CameraSettings.msg
  CollisionAvoidanceState.msg
  CollisionDetectionState.msg
  DigitalIOState.msg
  DigitalIOStates.msg
  DigitalOutputCommand.msg
  EndEffectorCommand.msg
  EndEffectorProperties.msg
  EndEffectorState.msg
  EndpointState.msg
  EndpointStates.msg
  HeadPanCommand.msg
  HeadState.msg
  ITBState.msg
  ITBStates.msg
  JointCommand.msg
  JointPositions.msg
  NavigatorState.msg
  NavigatorStates.msg
  RobustControllerStatus.msg
  SEAJointState.msg
)

add_service_files(DIRECTORY srv FILES
  CloseCamera.srv
  ListCameras.srv
  OpenCamera.srv
  SolvePositionIK.srv
)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
)
