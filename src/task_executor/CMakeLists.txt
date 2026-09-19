cmake_minimum_required(VERSION 3.16)
project(task_executor)

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
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mujoco_bridge REQUIRED)

add_executable(task_executor_node
  src/task_executor_node.cpp
  src/phase.cpp
  src/fsm.cpp
)
target_include_directories(task_executor_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(task_executor_node mujoco_bridge::grasp_criteria)
ament_target_dependencies(task_executor_node
  rclcpp
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  control_msgs
  std_msgs
  std_srvs
  tf2_ros
)

install(TARGETS task_executor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_fsm
    test/test_fsm.cpp
    src/phase.cpp
    src/fsm.cpp
  )
  target_include_directories(test_fsm PRIVATE include)
  target_link_libraries(test_fsm mujoco_bridge::grasp_criteria)

  ament_add_gtest(test_keyframe_waypoint_source
    test/test_keyframe_waypoint_source.cpp
    src/phase.cpp
  )
  target_include_directories(test_keyframe_waypoint_source PRIVATE include)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
