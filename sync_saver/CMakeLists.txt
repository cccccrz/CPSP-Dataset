cmake_minimum_required(VERSION 3.5)
project(sync_saver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(sync_saver 
  src/sync_saver.cpp
  include/config_parser.hpp
)
target_include_directories(sync_saver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(sync_saver rclcpp sensor_msgs cv_bridge message_filters OpenCV)

install(TARGETS sync_saver
  DESTINATION lib/${PROJECT_NAME})

ament_package()
