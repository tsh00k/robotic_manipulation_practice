cmake_minimum_required(VERSION 3.16)
project(mujoco_perception)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GLUT REQUIRED)
find_package(image_geometry REQUIRED)
# cv::connectedComponentsWithStats and cv::minAreaRect (initial_box_detector.cpp) are in imgproc.
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(geometric_shapes REQUIRED)
find_package(moveit_ros_perception REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(manipulation_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)

add_library(mujoco_perception_geometry
  src/robot_mask.cpp
  src/depth_window.cpp src/rectangle_blocks.cpp src/initial_box_detector.cpp
  src/initial_bin_detector.cpp src/initial_pose_estimator.cpp)
target_include_directories(mujoco_perception_geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_compile_options(mujoco_perception_geometry PRIVATE -O2)
target_link_libraries(mujoco_perception_geometry Eigen3::Eigen ${OpenCV_LIBS}
  geometric_shapes::geometric_shapes moveit_ros_perception::moveit_mesh_filter)
ament_target_dependencies(mujoco_perception_geometry image_geometry sensor_msgs)

add_executable(object_pose_estimator_node src/object_pose_estimator_node.cpp)
target_include_directories(object_pose_estimator_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(object_pose_estimator_node mujoco_perception_geometry)
ament_target_dependencies(object_pose_estimator_node
  rclcpp
  sensor_msgs
  geometry_msgs
  image_geometry
  manipulation_interfaces
  ament_index_cpp
  tf2
  tf2_ros
  tf2_msgs
)

install(TARGETS mujoco_perception_geometry object_pose_estimator_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Pure data structure and pure detector; the camera and the exactly ray-cast scene they
  # use are in test/workcell_camera.hpp.
  ament_add_gtest(test_depth_window test/test_depth_window.cpp)
  target_link_libraries(test_depth_window mujoco_perception_geometry)
  ament_add_gtest(test_initial_box_detector test/test_initial_box_detector.cpp)
  target_link_libraries(test_initial_box_detector mujoco_perception_geometry Eigen3::Eigen)
  ament_add_gtest(test_initial_bin_detector test/test_initial_bin_detector.cpp)
  target_link_libraries(test_initial_bin_detector mujoco_perception_geometry Eigen3::Eigen)
  ament_add_gtest(test_initial_pose_estimator test/test_initial_pose_estimator.cpp)
  target_link_libraries(test_initial_pose_estimator mujoco_perception_geometry Eigen3::Eigen)

  ament_add_gtest(test_robot_mask test/test_robot_mask.cpp)
  set_tests_properties(test_robot_mask PROPERTIES ENVIRONMENT "LIBGL_ALWAYS_SOFTWARE=1")
  target_include_directories(test_robot_mask PRIVATE include)
  target_link_libraries(test_robot_mask mujoco_perception_geometry Eigen3::Eigen
    geometric_shapes::geometric_shapes)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(Eigen3 image_geometry sensor_msgs geometric_shapes moveit_ros_perception)
ament_package()
