cmake_minimum_required(VERSION 3.16)
project(mujoco_bridge)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
find_package(manipulation_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(mujoco REQUIRED)
# scene_config.cpp / scene_ops.cpp (box/bin start layout) use Eigen transforms.
find_package(Eigen3 REQUIRED)
# For the optional debug viewer (debug_viewer.cpp) only. Unlike mujoco::mujoco,
# glfw3 does not bundle a conflicting tinyxml2, so there is no dlopen-isolation
# reason to avoid linking it normally. Provided by apt (libglfw3-dev); not listed
# in package.xml for the same reason mujoco isn't -- it's a manually-installed
# system library, not a rosdep-resolvable ROS package.
find_package(glfw3 REQUIRED)

# Exported as its own target (not just compiled into mujoco_bridge_node) because
# grasp_criteria.hpp/cpp has zero rclcpp/mjModel dependencies by design (Stage F's
# Layer 1) and grasp_criteria.hpp's GraspOutcome enum was written expecting a second
# consumer: task_executor (Stage I) needs kSlip/kGraspEmpty/etc. with the exact same
# semantics, plus the phase/timing context only it has (for kTimeout/kPlaceMissed,
# which classifyGrasp() itself never returns). Reusing this target beats
# reimplementing classifyGrasp() a second time in task_executor and having the two
# drift apart silently.
add_library(grasp_criteria src/grasp_criteria.cpp)
target_include_directories(grasp_criteria PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Exported so task_executor's tests can check its planned trajectories through the same
# interpolation the bridge executes (Week 5 Stage 3).
add_library(trajectory_interpolator src/trajectory_interpolator.cpp)
target_include_directories(trajectory_interpolator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(mujoco_bridge_node
  src/mujoco_bridge_node.cpp
  src/mujoco_dl.cpp
  src/frame_math.cpp
  src/state_ops.cpp
  src/grasp_state.cpp
  src/debug_viewer.cpp
  src/rgbd_camera.cpp
  # Scene layout lives in src/ (headers included), not include/, so it is neither
  # installed nor reachable through the exported grasp_criteria include directory.
  src/scene_config.cpp
  src/scene_ops.cpp
)
target_include_directories(mujoco_bridge_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(mujoco_bridge_node grasp_criteria trajectory_interpolator glfw Eigen3::Eigen)
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
  manipulation_interfaces
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

install(TARGETS grasp_criteria trajectory_interpolator
  EXPORT export_grasp_criteria
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_grasp_criteria HAS_LIBRARY_TARGET)

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

  ament_add_gtest(test_camera_geometry test/test_camera_geometry.cpp)
  target_include_directories(test_camera_geometry PRIVATE include)

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

  # Pure geometry/validation, no ROS and no MuJoCo.
  ament_add_gtest(test_scene_config
    test/test_scene_config.cpp
    src/scene_config.cpp
  )
  target_include_directories(test_scene_config PRIVATE src)
  target_link_libraries(test_scene_config Eigen3::Eigen)

  # Against the real pick_place_scene.xml (the default scene, no bin) and
  # pick_place_bin_scene.xml (what scene.enabled loads): the extents read from the model,
  # what applying a layout does, and that the second is the first plus only the bin. The
  # source-tree paths are compile-time definitions, like PANDA_MJCF_PATH below.
  get_filename_component(PICK_PLACE_SCENE_MJCF_PATH
    "${CMAKE_CURRENT_SOURCE_DIR}/../../robot_description/mujoco/franka_emika_panda/pick_place_scene.xml"
    ABSOLUTE)
  get_filename_component(PICK_PLACE_BIN_SCENE_MJCF_PATH
    "${CMAKE_CURRENT_SOURCE_DIR}/../../robot_description/mujoco/franka_emika_panda/pick_place_bin_scene.xml"
    ABSOLUTE)
  ament_add_gtest(test_scene_ops
    test/test_scene_ops.cpp
    src/scene_ops.cpp
    src/scene_config.cpp
    src/state_ops.cpp
    src/mujoco_dl.cpp
  )
  target_include_directories(test_scene_ops PRIVATE
    include src ${MUJOCO_INCLUDE_DIRS})
  target_link_libraries(test_scene_ops Eigen3::Eigen dl)
  target_compile_definitions(test_scene_ops PRIVATE
    PICK_PLACE_SCENE_MJCF_PATH="${PICK_PLACE_SCENE_MJCF_PATH}"
    PICK_PLACE_BIN_SCENE_MJCF_PATH="${PICK_PLACE_BIN_SCENE_MJCF_PATH}")

  # Launch glue for scene.* (only given values, only to the bridge) and the rule that no
  # other package uses the scene configuration. Pure Python, no simulator.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_scene_launch test/test_scene_launch.py)

  ament_add_gtest(test_grasp_criteria
    test/test_grasp_criteria.cpp
  )
  target_include_directories(test_grasp_criteria PRIVATE include)
  target_link_libraries(test_grasp_criteria grasp_criteria)

  ament_add_gtest(test_trajectory_interpolator test/test_trajectory_interpolator.cpp)
  target_link_libraries(test_trajectory_interpolator trajectory_interpolator)

  ament_add_gtest(test_grasp_state
    test/test_grasp_state.cpp
    src/grasp_state.cpp
    src/mujoco_dl.cpp
  )
  target_include_directories(test_grasp_state PRIVATE include ${MUJOCO_INCLUDE_DIRS})
  target_link_libraries(test_grasp_state dl)
  target_compile_definitions(test_grasp_state PRIVATE
    TEST_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")

  # Stage L integration test: keep MoveIt and arm_kinematics as test-only
  # dependencies so neither the bridge node nor the pure math library acquires
  # an unnecessary production dependency.
  find_package(arm_kinematics REQUIRED)
  find_package(franka_description REQUIRED)
  find_package(moveit_core REQUIRED)
  find_package(xacro REQUIRED)

  get_filename_component(FRANKA_DESCRIPTION_SHARE_DIR
    "${franka_description_DIR}/.." ABSOLUTE)
  get_filename_component(PANDA_MJCF_PATH
    "${CMAKE_CURRENT_SOURCE_DIR}/../../robot_description/mujoco/franka_emika_panda/panda.xml"
    ABSOLUTE)

  xacro_add_xacro_file(
    "${FRANKA_DESCRIPTION_SHARE_DIR}/robots/fer/fer.urdf.xacro"
    "${CMAKE_CURRENT_BINARY_DIR}/fer.urdf"
    OUTPUT FER_URDF_PATH)
  xacro_add_xacro_file(
    "${FRANKA_DESCRIPTION_SHARE_DIR}/robots/fer/fer.srdf.xacro"
    "${CMAKE_CURRENT_BINARY_DIR}/fer.srdf"
    OUTPUT FER_SRDF_PATH)
  add_custom_target(stage_l_robot_descriptions
    DEPENDS "${FER_URDF_PATH}" "${FER_SRDF_PATH}")

  ament_add_gtest(test_model_consistency
    test/test_model_consistency.cpp
    src/mujoco_dl.cpp
  )
  add_dependencies(test_model_consistency stage_l_robot_descriptions)
  target_include_directories(test_model_consistency PRIVATE include ${MUJOCO_INCLUDE_DIRS})
  target_link_libraries(test_model_consistency arm_kinematics::arm_kinematics dl)
  ament_target_dependencies(test_model_consistency moveit_core)
  target_compile_definitions(test_model_consistency PRIVATE
    FER_URDF_PATH="${FER_URDF_PATH}"
    FER_SRDF_PATH="${FER_SRDF_PATH}"
    KINEMATICS_YAML_PATH="${FRANKA_DESCRIPTION_SHARE_DIR}/robots/fer/kinematics.yaml"
    JOINT_LIMITS_YAML_PATH="${FRANKA_DESCRIPTION_SHARE_DIR}/robots/fer/joint_limits.yaml"
    PANDA_MJCF_PATH="${PANDA_MJCF_PATH}"
  )

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
