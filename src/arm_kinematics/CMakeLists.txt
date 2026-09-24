cmake_minimum_required(VERSION 3.16)
project(arm_kinematics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(arm_kinematics
  src/differential_ik.cpp
  src/forward_kinematics.cpp
  src/model_loader.cpp
)
target_include_directories(arm_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(arm_kinematics PUBLIC Eigen3::Eigen yaml-cpp)

add_executable(singularity_sweep tools/singularity_sweep.cpp)
target_link_libraries(singularity_sweep arm_kinematics)

install(TARGETS arm_kinematics
  EXPORT export_arm_kinematics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS singularity_sweep
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_arm_kinematics HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(franka_description REQUIRED)

  get_filename_component(FRANKA_DESCRIPTION_SHARE_DIR
    "${franka_description_DIR}/.." ABSOLUTE)

  ament_add_gtest(test_arm_kinematics
    test/test_arm_kinematics.cpp
  )
  target_link_libraries(test_arm_kinematics arm_kinematics)
  target_compile_definitions(test_arm_kinematics PRIVATE
    KINEMATICS_YAML_PATH="${FRANKA_DESCRIPTION_SHARE_DIR}/robots/fer/kinematics.yaml"
    JOINT_LIMITS_YAML_PATH="${FRANKA_DESCRIPTION_SHARE_DIR}/robots/fer/joint_limits.yaml"
    TEST_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures"
  )

  ament_add_gtest(test_differential_ik
    test/test_differential_ik.cpp
  )
  target_link_libraries(test_differential_ik arm_kinematics)
  target_compile_definitions(test_differential_ik PRIVATE
    KINEMATICS_YAML_PATH="${FRANKA_DESCRIPTION_SHARE_DIR}/robots/fer/kinematics.yaml"
    JOINT_LIMITS_YAML_PATH="${FRANKA_DESCRIPTION_SHARE_DIR}/robots/fer/joint_limits.yaml"
  )

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
