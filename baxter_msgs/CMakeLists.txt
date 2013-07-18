cmake_minimum_required(VERSION 2.8.3)
project(baxter_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs)

add_message_files(
  FILES
  AnalogIOState.msg
  AnalogIOStates.msg
  AnalogOutputCommand.msg
  AssemblyState.msg
  CalibrateArmEnable.msg 
  CameraControl.msg  
  CameraSettings.msg 
  DigitalIOState.msg
  DigitalIOStates.msg
  DigitalOutputCommand.msg 
  EndpointState.msg
  GripperCommand.msg 
  GripperIdentity.msg 
  GripperProperties.msg 
  GripperState.msg
  HeadPanCommand.msg
  HeadState.msg
  ITB.msg
  ITBStates.msg
  JointCommandMode.msg
  JointPositions.msg
  JointTorques.msg
  JointVelocities.msg
  RobustControllerStatus.msg
  TareData.msg
  TareEnable.msg
  UpdateSource.msg
  UpdateSources.msg
  UpdateStatus.msg
)

add_service_files(
  FILES
  CloseCamera.srv
  ListCameras.srv
  LSCores.srv
  OpenCamera.srv
  RMCores.srv
  SolvePositionIK.srv
)

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(CATKIN_DEPENDS std_msgs geometry_msgs)
