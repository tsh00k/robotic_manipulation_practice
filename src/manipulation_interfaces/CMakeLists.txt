cmake_minimum_required(VERSION 3.16)
project(manipulation_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EpisodeOutcome.msg"
  "msg/BridgeObservation.msg"
  "msg/VisionObjectPose.msg"
  "msg/RobotMaskDiagnostics.msg"
  "msg/BlockCandidate.msg"
  "msg/InitialBoxPose.msg"
  "msg/InitialBinPose.msg"
  "msg/LatchedPose.msg"
  "msg/InitialPoseLatch.msg"
  "srv/ResetScene.srv"
  DEPENDENCIES sensor_msgs geometry_msgs std_msgs builtin_interfaces
)

ament_package()
