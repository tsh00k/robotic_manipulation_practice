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
find_package(PCL REQUIRED COMPONENTS common features filters io registration search segmentation)
find_package(image_geometry REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(moveit_ros_perception REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(manipulation_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)

add_library(mujoco_perception_geometry
  src/geometry_pipeline.cpp src/robot_mask.cpp src/object_tracker.cpp)
target_include_directories(mujoco_perception_geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_compile_definitions(mujoco_perception_geometry PUBLIC ${PCL_DEFINITIONS})
target_compile_options(mujoco_perception_geometry PRIVATE -O2)
target_link_libraries(mujoco_perception_geometry Eigen3::Eigen ${PCL_LIBRARIES}
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
  pcl_conversions
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
  add_executable(compare_registration test/compare_registration.cpp)
  target_link_libraries(compare_registration mujoco_perception_geometry)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_object_tracker test/test_object_tracker.cpp)
  target_link_libraries(test_object_tracker mujoco_perception_geometry)

  ament_add_gtest(test_geometry_pipeline
    test/test_geometry_pipeline.cpp
  )
  target_include_directories(test_geometry_pipeline PRIVATE include)
  target_include_directories(test_geometry_pipeline PRIVATE ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_geometry_pipeline mujoco_perception_geometry Eigen3::Eigen ${PCL_LIBRARIES})

  ament_add_gtest(test_robot_mask test/test_robot_mask.cpp)
  set_tests_properties(test_robot_mask PROPERTIES ENVIRONMENT "LIBGL_ALWAYS_SOFTWARE=1")
  target_include_directories(test_robot_mask PRIVATE include)
  target_link_libraries(test_robot_mask mujoco_perception_geometry Eigen3::Eigen ${PCL_LIBRARIES}
    geometric_shapes::geometric_shapes)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(Eigen3 image_geometry sensor_msgs geometric_shapes moveit_ros_perception)
ament_package()
