cmake_minimum_required(VERSION 3.5)
project(leap_srvs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(leap_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(leap_srvs
        "../leap_srvs/srv/HandToPose.srv"
        DEPENDENCIES geometry_msgs leap_msgs)

ament_package()
