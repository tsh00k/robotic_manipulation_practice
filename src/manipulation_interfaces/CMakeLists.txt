cmake_minimum_required(VERSION 3.16)
project(manipulation_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EpisodeOutcome.msg"
  "msg/BridgeObservation.msg"
  "msg/VisionObjectPose.msg"
  "srv/ResetScene.srv"
  DEPENDENCIES sensor_msgs geometry_msgs std_msgs
)

ament_package()
