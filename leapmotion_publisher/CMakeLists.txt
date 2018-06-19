cmake_minimum_required(VERSION 3.5)
project(leapmotion_publisher)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(LEAP_SDK $ENV{LEAP_SDK})

if(APPLE)
  set(LEAP_LIBRARY_NAME libLeap.dylib)
elseif(UNIX)
  set(LEAP_LIBRARY_NAME x64/libLeap.so)
endif()

find_library(LEAP_LIBRARY
  NAMES ${LEAP_LIBRARY_NAME}
  HINTS "${LEAP_SDK}/lib")
message(STATUS ${LEAP_LIBRARY})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(leap_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(${PROJECT_NAME} main.cpp LeapPublisher.cpp LeapPublisher.h)

target_include_directories(${PROJECT_NAME} PRIVATE ${LEAP_SDK}/include)
target_link_libraries(${PROJECT_NAME} ${LEAP_LIBRARY})
ament_target_dependencies(${PROJECT_NAME} rclcpp geometry_msgs leap_msgs tf2_ros visualization_msgs)

install(TARGETS ${PROJECT_NAME}
  DESTINATION bin/${PROJECT_NAME})
