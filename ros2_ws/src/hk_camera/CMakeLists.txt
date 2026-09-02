cmake_minimum_required(VERSION 3.14)
project(hk_camera)

# ============================================================
# 默认构建选项
# ============================================================
option(BUILD_HK_GUI "Build Qt5 GUI application" ON)

# ============================================================
# C++ 标准
# ============================================================
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ============================================================
# ROS2 + 自定义消息
# ============================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ocr_interfaces REQUIRED)


# 自定义消息/服务
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CameraAlarm.msg"
  "msg/MissionWaypoint.msg"
  "msg/MissionStatus.msg"
  "srv/CapturePicture.srv"
  "srv/SetPTZPose.srv"
  "srv/RunMission.srv"
  DEPENDENCIES builtin_interfaces sensor_msgs geometry_msgs
  ADD_LINTER_TESTS
)

# ============================================================
# Qt5
# ============================================================
# HKCamera 使用 Q_OBJECT 宏, AUTOMOC 无条件启用
set(CMAKE_AUTOMOC ON)
find_package(Qt5 COMPONENTS Core Gui REQUIRED)
if(BUILD_HK_GUI)
  set(CMAKE_AUTORCC ON)
  set(CMAKE_AUTOUIC ON)
  find_package(Qt5 COMPONENTS Widgets REQUIRED)
endif()

# ============================================================
# 海康 SDK
# ============================================================
if(DEFINED ENV{HK_SDK_LIB_DIR})
  set(HK_SDK_LIB_DIR "$ENV{HK_SDK_LIB_DIR}")
else()
  set(HK_SDK_LIB_DIR "${CMAKE_SOURCE_DIR}/../HK/C++demo/consoleDemo/linux64/lib")
endif()
get_filename_component(HK_SDK_LIB_DIR_ABS "${HK_SDK_LIB_DIR}" ABSOLUTE)

if(DEFINED ENV{HK_SDK_INCLUDE_DIR})
  set(HK_SDK_INCLUDE_DIR "$ENV{HK_SDK_INCLUDE_DIR}")
else()
  set(HK_SDK_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/../HK/C++demo/consoleDemo/include")
endif()
get_filename_component(HK_SDK_INCLUDE_DIR_ABS "${HK_SDK_INCLUDE_DIR}" ABSOLUTE)

message(STATUS "HK SDK libs:    ${HK_SDK_LIB_DIR_ABS}")
message(STATUS "HK SDK headers:  ${HK_SDK_INCLUDE_DIR_ABS}")

add_library(hcnetsdk SHARED IMPORTED)
set_target_properties(hcnetsdk PROPERTIES
  IMPORTED_LOCATION "${HK_SDK_LIB_DIR_ABS}/libhcnetsdk.so"
  INTERFACE_INCLUDE_DIRECTORIES "${HK_SDK_INCLUDE_DIR_ABS}"
)

# 辅助 SDK 库 (直接链接 .so 文件路径)
set(HK_EXTRA_LIBS
  "${HK_SDK_LIB_DIR_ABS}/libHCAlarm.so"
  "${HK_SDK_LIB_DIR_ABS}/libHCGeneralCfgMgr.so"
  "${HK_SDK_LIB_DIR_ABS}/libHCIndustry.so"
  "${HK_SDK_LIB_DIR_ABS}/libHCPreview.so"
  "${HK_SDK_LIB_DIR_ABS}/libPlayCtrl.so"
  "${HK_SDK_LIB_DIR_ABS}/libAudioProcess.so"
  "${HK_SDK_LIB_DIR_ABS}/libanalyzedata.so"
  "${HK_SDK_LIB_DIR_ABS}/libStreamTransClient.so"
)

# 公共头文件路径
set(COMMON_INCLUDE_DIRS
  include
  ${HK_SDK_INCLUDE_DIR_ABS}
)

# ============================================================
# 源文件列表 (直接编译到可执行文件, 避免 STATIC 库的传递依赖问题)
# ============================================================
set(HK_CAMERA_SRCS
  src/hk_camera.cpp
)

set(HK_DECODER_SRCS
  src/hk_decoder.cpp
)

set(HK_NODE_SRCS
  src/hk_camera_node.cpp
)

set(HK_GUI_SRCS
  src/main.cpp
  src/main_window.cpp
)

# ============================================================
# 1. 纯 ROS2 节点 (headless, 无 GUI)
# ============================================================
add_executable(hk_camera_node
  src/hk_camera_node_main.cpp
  ${HK_CAMERA_SRCS}
  ${HK_DECODER_SRCS}
  ${HK_NODE_SRCS}
  include/hk_camera/hk_camera.h          # 含 Q_OBJECT, 需要 MOC
  include/hk_camera/hk_camera_node.h
  include/hk_camera/hk_decoder.h         # 含 Q_OBJECT, 需要 MOC
)
target_include_directories(hk_camera_node PRIVATE ${COMMON_INCLUDE_DIRS})
target_link_libraries(hk_camera_node
  hcnetsdk
  Qt5::Core
  Qt5::Gui
  ${HK_EXTRA_LIBS}
)
ament_target_dependencies(hk_camera_node
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
  cv_bridge
  OpenCV
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(hk_camera_node ${cpp_typesupport_target})

# 补上生成的自定义消息头文件路径
target_include_directories(hk_camera_node PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)

# ============================================================
# 2. Qt GUI 应用 (集成 ROS2)
# ============================================================
if(BUILD_HK_GUI)
  add_executable(hk_camera_gui
    ${HK_GUI_SRCS}
    ${HK_CAMERA_SRCS}
    ${HK_DECODER_SRCS}
    ${HK_NODE_SRCS}
    include/hk_camera/hk_camera.h          # 含 Q_OBJECT, 需要 MOC
    include/hk_camera/hk_camera_node.h
    include/hk_camera/hk_decoder.h         # 含 Q_OBJECT, 需要 MOC
    src/main_window.h                      # 含 Q_OBJECT, 需要 MOC
  )
  target_include_directories(hk_camera_gui PRIVATE
    ${COMMON_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src   # for main_window.h
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  )
  target_link_libraries(hk_camera_gui
    hcnetsdk
    Qt5::Widgets
    ${HK_EXTRA_LIBS}
    ${cpp_typesupport_target}
  )
  ament_target_dependencies(hk_camera_gui
    rclcpp
    sensor_msgs
    std_msgs
    std_srvs
    geometry_msgs
    cv_bridge
    OpenCV
    ocr_interfaces
  )
endif()

# ============================================================
# 安装
# ============================================================
install(TARGETS
  hk_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_HK_GUI)
  install(TARGETS
    hk_camera_gui
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY
  include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ============================================================
# Ament 导出
# ============================================================
ament_export_dependencies(rosidl_default_runtime)
ament_package()
