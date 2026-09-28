cmake_minimum_required(VERSION 3.10)
project(krthumanrobot_urdf)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config meshes DESTINATION share/${PROJECT_NAME})
install(FILES robot.urdf DESTINATION share/${PROJECT_NAME})

ament_package()
