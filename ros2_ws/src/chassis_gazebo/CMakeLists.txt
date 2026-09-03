cmake_minimum_required(VERSION 3.8)
project(chassis_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()
