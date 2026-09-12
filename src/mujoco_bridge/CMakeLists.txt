cmake_minimum_required(VERSION 3.16)
project(mujoco_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(mujoco REQUIRED)

add_executable(mujoco_bridge_node
  src/mujoco_bridge_node.cpp
  src/mujoco_dl.cpp
)
target_include_directories(mujoco_bridge_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(mujoco_bridge_node
  rclcpp
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  control_msgs
  tf2_ros
  std_srvs
  ament_index_cpp
)

# We deliberately do NOT target_link_libraries(mujoco::mujoco): mujoco bundles
# its own tinyxml2 with default symbol visibility, which clashes with
# libfastrtps's own bundled tinyxml2 if mujoco is a normal link-time
# dependency (see mujoco_bridge/mujoco_dl.hpp for the full explanation).
# We only need its headers here; the library itself is dlopen'd at runtime
# in isolated mode (mujoco_dl.cpp).
get_target_property(MUJOCO_INCLUDE_DIRS mujoco::mujoco INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(mujoco_bridge_node PRIVATE ${MUJOCO_INCLUDE_DIRS})
target_link_libraries(mujoco_bridge_node dl)

install(TARGETS mujoco_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
