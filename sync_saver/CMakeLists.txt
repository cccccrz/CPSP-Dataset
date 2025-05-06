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
find_package(yaml-cpp REQUIRED)

add_executable(sync_saver 
  src/sync_saver.cpp
)

# 头文件路径（如果有）
target_include_directories(sync_saver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ROS2 ament 依赖
ament_target_dependencies(sync_saver 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  message_filters 
  OpenCV
)

# 显式把 yaml-cpp 加入链接
target_link_libraries(sync_saver yaml-cpp)

install(TARGETS sync_saver
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
