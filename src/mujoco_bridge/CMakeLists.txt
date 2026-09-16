cmake_minimum_required(VERSION 3.16)
project(mujoco_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(mujoco REQUIRED)

add_executable(mujoco_bridge_node
  src/mujoco_bridge_node.cpp
  src/mujoco_dl.cpp
  src/frame_math.cpp
  src/state_ops.cpp
  src/grasp_criteria.cpp
  src/grasp_state.cpp
)
target_include_directories(mujoco_bridge_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(mujoco_bridge_node
  rclcpp
  sensor_msgs
  rosgraph_msgs
  geometry_msgs
  trajectory_msgs
  control_msgs
  std_msgs
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

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_frame_math
    test/test_frame_math.cpp
    src/frame_math.cpp
    src/mujoco_dl.cpp
  )
  target_include_directories(test_frame_math PRIVATE include ${MUJOCO_INCLUDE_DIRS})
  target_link_libraries(test_frame_math dl)

  ament_add_gtest(test_state_ops
    test/test_state_ops.cpp
    src/state_ops.cpp
    src/mujoco_dl.cpp
  )
  target_include_directories(test_state_ops PRIVATE include ${MUJOCO_INCLUDE_DIRS})
  target_link_libraries(test_state_ops dl)
  # Compile-time path rather than ament_index or a cwd-relative path: colcon test
  # may invoke the test binary from an arbitrary working directory.
  target_compile_definitions(test_state_ops PRIVATE
    TEST_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")

  ament_add_gtest(test_grasp_criteria
    test/test_grasp_criteria.cpp
    src/grasp_criteria.cpp
  )
  target_include_directories(test_grasp_criteria PRIVATE include)

  ament_add_gtest(test_grasp_state
    test/test_grasp_state.cpp
    src/grasp_state.cpp
    src/mujoco_dl.cpp
  )
  target_include_directories(test_grasp_state PRIVATE include ${MUJOCO_INCLUDE_DIRS})
  target_link_libraries(test_grasp_state dl)
  target_compile_definitions(test_grasp_state PRIVATE
    TEST_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
