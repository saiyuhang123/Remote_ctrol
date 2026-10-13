cmake_minimum_required(VERSION 3.8)
project(chassis_nav2)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/lio_odometry.py scripts/jd03_bridge.py scripts/jd03_odometry.py scripts/jd03_keyboard.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
