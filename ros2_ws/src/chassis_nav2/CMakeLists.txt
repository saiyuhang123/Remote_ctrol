cmake_minimum_required(VERSION 3.8)
project(chassis_nav2)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 17)
foreach(dep nav2_core nav2_regulated_pure_pursuit_controller nav2_util nav2_costmap_2d
    rclcpp rclcpp_lifecycle tf2_geometry_msgs tf2_ros pluginlib nav_msgs geometry_msgs)
  find_package(${dep} REQUIRED)
endforeach()
add_library(chassis_ackermann_pursuit SHARED src/ackermann_pursuit.cpp)
target_include_directories(chassis_ackermann_pursuit PRIVATE include)
ament_target_dependencies(chassis_ackermann_pursuit nav2_core
  nav2_regulated_pure_pursuit_controller nav2_util nav2_costmap_2d rclcpp
  rclcpp_lifecycle tf2_geometry_msgs tf2_ros pluginlib nav_msgs geometry_msgs)
pluginlib_export_plugin_description_file(nav2_core controller_plugins.xml)
install(TARGETS chassis_ackermann_pursuit LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(pursuit_path_test tests/test_pursuit_path.cpp)
  target_include_directories(pursuit_path_test PRIVATE include)
  ament_target_dependencies(pursuit_path_test nav_msgs tf2_geometry_msgs)
endif()

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/lio_odometry.py scripts/jd03_bridge.py scripts/jd03_odometry.py scripts/jd03_keyboard.py scripts/cloud_crop_box.py scripts/final_approach.py scripts/ptz_patrol.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
