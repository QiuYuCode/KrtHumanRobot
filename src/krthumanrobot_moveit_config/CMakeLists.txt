cmake_minimum_required(VERSION 3.10)
project(krthumanrobot_moveit_config)

find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(moveit_core REQUIRED)
  ament_add_gtest(test_collision_model test/test_collision_model.cpp)
  ament_target_dependencies(test_collision_model moveit_core)
  target_compile_features(test_collision_model PRIVATE cxx_std_14)
  target_compile_options(test_collision_model PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_definitions(test_collision_model PRIVATE
    MODEL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../krthumanrobot_urdf/config/robot_moveit.urdf"
    SRDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/robot.srdf")
endif()
ament_package()
