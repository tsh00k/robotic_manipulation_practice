cmake_minimum_required(VERSION 3.16)
project(task_executor)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
find_package(mujoco_bridge REQUIRED)
find_package(manipulation_interfaces REQUIRED)
find_package(arm_kinematics REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(task_executor_node
  src/task_executor_node.cpp
  src/phase.cpp
  src/fsm.cpp
  src/cartesian_waypoint_source.cpp
  src/diff_ik_waypoint_source.cpp
  src/task_executor_config.cpp
  src/episode_telemetry.cpp
  src/episode_controller.cpp
  src/observation_snapshot.cpp
  src/pose_latch.cpp
  src/bin_containment.cpp
  src/carry_width_monitor.cpp
)
target_include_directories(task_executor_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(task_executor_node
  mujoco_bridge::grasp_criteria arm_kinematics::arm_kinematics)
ament_target_dependencies(task_executor_node
  rclcpp
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  control_msgs
  std_msgs
  manipulation_interfaces
  ament_index_cpp
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
    src/bin_containment.cpp
  )
  target_include_directories(test_fsm PRIVATE include)
  target_link_libraries(test_fsm mujoco_bridge::grasp_criteria)

  ament_add_gtest(test_keyframe_waypoint_source
    test/test_keyframe_waypoint_source.cpp
    src/phase.cpp
  )
  target_include_directories(test_keyframe_waypoint_source PRIVATE include)

  ament_add_gtest(test_diff_ik_waypoint_source
    test/test_diff_ik_waypoint_source.cpp
    src/cartesian_waypoint_source.cpp
    src/diff_ik_waypoint_source.cpp
    src/phase.cpp
  )
  target_include_directories(test_diff_ik_waypoint_source PRIVATE include)
  target_link_libraries(test_diff_ik_waypoint_source arm_kinematics::arm_kinematics)
  find_package(franka_description REQUIRED)
  get_filename_component(FRANKA_DESCRIPTION_SHARE_DIR
    "${franka_description_DIR}/.." ABSOLUTE)
  target_compile_definitions(test_diff_ik_waypoint_source PRIVATE
    KINEMATICS_YAML_PATH="${FRANKA_DESCRIPTION_SHARE_DIR}/robots/fer/kinematics.yaml"
    JOINT_LIMITS_YAML_PATH="${FRANKA_DESCRIPTION_SHARE_DIR}/robots/fer/joint_limits.yaml"
  )

  ament_add_gtest(test_cartesian_waypoint_source
    test/test_cartesian_waypoint_source.cpp
    src/cartesian_waypoint_source.cpp
    src/phase.cpp
  )
  target_include_directories(test_cartesian_waypoint_source PRIVATE include)
  target_link_libraries(test_cartesian_waypoint_source Eigen3::Eigen)

  ament_add_gtest(test_task_executor_config
    test/test_task_executor_config.cpp
    src/task_executor_config.cpp
  )
  target_include_directories(test_task_executor_config PRIVATE include)
  target_link_libraries(test_task_executor_config
    Eigen3::Eigen mujoco_bridge::grasp_criteria)
  ament_target_dependencies(test_task_executor_config rclcpp)

  ament_add_gtest(test_episode_telemetry
    test/test_episode_telemetry.cpp
    src/episode_telemetry.cpp
  )
  target_include_directories(test_episode_telemetry PRIVATE include)
  ament_target_dependencies(test_episode_telemetry manipulation_interfaces)

  ament_add_gtest(test_observation_snapshot
    test/test_observation_snapshot.cpp
    src/observation_snapshot.cpp
  )
  target_include_directories(test_observation_snapshot PRIVATE include)
  target_link_libraries(test_observation_snapshot
    Eigen3::Eigen mujoco_bridge::grasp_criteria)
  ament_target_dependencies(test_observation_snapshot sensor_msgs geometry_msgs)

  ament_add_gtest(test_carry_width_monitor
    test/test_carry_width_monitor.cpp
    src/carry_width_monitor.cpp
    src/phase.cpp
  )
  target_include_directories(test_carry_width_monitor PRIVATE include)

  ament_add_gtest(test_bin_containment
    test/test_bin_containment.cpp
    src/bin_containment.cpp
  )
  target_include_directories(test_bin_containment PRIVATE include)

  ament_add_gtest(test_pose_latch
    test/test_pose_latch.cpp
    src/pose_latch.cpp
  )
  target_include_directories(test_pose_latch PRIVATE include)

  ament_add_gtest(test_episode_controller
    test/test_episode_controller.cpp
    src/episode_controller.cpp
    src/fsm.cpp
    src/bin_containment.cpp
    src/phase.cpp
    src/diff_ik_waypoint_source.cpp
  )
  target_include_directories(test_episode_controller PRIVATE include)
  target_link_libraries(test_episode_controller
    mujoco_bridge::grasp_criteria Eigen3::Eigen arm_kinematics::arm_kinematics)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
