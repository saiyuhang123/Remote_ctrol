cmake_minimum_required(VERSION 3.5)

project(hi_ros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AreaCom.msg"
  "msg/LaserInfo.msg"
  "srv/CmdSrv.srv"
  DEPENDENCIES std_msgs
)

file(GLOB_RECURSE HEADER_FILES include/*.h)
file(GLOB_RECURSE SOURCE_FILES src/*.cpp)

include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/include
  #HEADER_FILES
  #SOURCE_FILES
)

add_executable(${PROJECT_NAME}_node src/main.cpp src/XingSongDriver/xingsong_driver.cpp src/XingSongDriver/laserprotocol.cpp src/XingSongDriver/shadowsfilter.cpp
              src/ConnectDriver/ConnectDriver.cpp src/ConnectDriver/NetworkDriver.h)
#add_executable(${PROJECT_NAME}_node src/main.cpp src/XingSongDriver/xingsong_driver.cpp)
target_link_libraries(${PROJECT_NAME}_node Boost::thread)
ament_target_dependencies(${PROJECT_NAME}_node
  "rclcpp"
  "sensor_msgs"
  "visualization_msgs"
  "geometry_msgs"

)
 
rosidl_target_interfaces(${PROJECT_NAME}_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")


install(TARGETS
  ${PROJECT_NAME}_node 
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY
DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

