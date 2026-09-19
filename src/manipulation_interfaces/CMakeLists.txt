cmake_minimum_required(VERSION 3.16)
project(manipulation_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EpisodeOutcome.msg"
)

ament_package()
