cmake_minimum_required(VERSION 3.8)
project(ocr_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OcrDetection.msg"
  "msg/OcrResult.msg"
  "srv/RecognizeText.srv"
  DEPENDENCIES std_msgs
)

ament_package()
