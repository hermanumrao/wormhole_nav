cmake_minimum_required(VERSION 3.8)
project(simple_navigation)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)   # <-- add this

add_executable(auto_localize_and_nav src/auto_localize_and_nav.cpp)

ament_target_dependencies(auto_localize_and_nav
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  tf2
  sensor_msgs
)

install(TARGETS
  auto_localize_and_nav
  DESTINATION lib/${PROJECT_NAME})

ament_package()
