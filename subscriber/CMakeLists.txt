cmake_minimum_required(VERSION 3.5)
project(subscriber)

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
find_package(messages REQUIRED)

add_executable(sub SubscriberNode.cpp)
ament_target_dependencies(sub rcutils rclcpp std_msgs messages)

install(TARGETS sub DESTINATION lib/subscriber)

ament_package()
