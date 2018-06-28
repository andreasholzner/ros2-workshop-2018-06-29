cmake_minimum_required(VERSION 3.5)
project(ros2_workshop_demo_nodes)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(leap_msgs REQUIRED)
find_package(leap_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(hand_publisher hand_publisher.cpp)
ament_target_dependencies(hand_publisher rclcpp rcutils leap_msgs visualization_msgs)

add_executable(conversion_service conversion_service.cpp)
ament_target_dependencies(conversion_service rclcpp rcutils geometry_msgs leap_msgs leap_srvs)

add_executable(conversion_service_client conversion_service_client.cpp)
ament_target_dependencies(conversion_service_client
  rclcpp rcutils geometry_msgs leap_msgs leap_srvs)

add_executable(tf_broadcaster tf_broadcaster.cpp)
ament_target_dependencies(tf_broadcaster rclcpp tf2_ros leap_msgs geometry_msgs)

add_executable(tf_lookup tf_lookup.cpp)
ament_target_dependencies(tf_lookup rclcpp tf2 tf2_ros leap_msgs geometry_msgs)

add_executable(tf_lookup_hand2hand tf_lookup_hand2hand.cpp)
ament_target_dependencies(tf_lookup_hand2hand rclcpp tf2 tf2_ros leap_msgs geometry_msgs)

add_executable(tf_transform tf_transform.cpp)
ament_target_dependencies(tf_transform rclcpp tf2 tf2_geometry_msgs tf2_ros leap_msgs geometry_msgs)

install(TARGETS
  hand_publisher
  conversion_service
  conversion_service_client
  tf_broadcaster
  tf_lookup
  tf_lookup_hand2hand
  tf_transform
  DESTINATION lib/${PROJECT_NAME})

ament_package()
