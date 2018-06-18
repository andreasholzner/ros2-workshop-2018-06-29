cmake_minimum_required(VERSION 3.5)
project(leap_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(leap_msgs
        "../leap_msgs/msg/Hand.msg"
        "../leap_msgs/msg/HandArray.msg"
        DEPENDENCIES std_msgs geometry_msgs)

ament_package()
