cmake_minimum_required(VERSION 3.5)
project(broadcaster)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rcutils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(leap_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(sub BroadcastNode.cpp)
ament_target_dependencies(sub rcutils rclcpp std_msgs leap_msgs tf2_ros)

install(TARGETS sub DESTINATION lib/broadcaster)

ament_package()
