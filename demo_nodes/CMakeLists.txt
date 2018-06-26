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
find_package(rclcpp REQUIRED)
find_package(rcutils)
find_package(geometry_msgs REQUIRED)
find_package(leap_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(leap_srvs REQUIRED)

include_directories(include)

add_executable(hand_publisher
  src/hand_publisher.cpp
  src/hand_publisher_node.cpp)
ament_target_dependencies(hand_publisher rclcpp rcutils leap_msgs visualization_msgs)

add_executable(conversion_service
  src/conversion_service.cpp
  src/conversion_service_node.cpp)
ament_target_dependencies(conversion_service rclcpp rcutils geometry_msgs leap_srvs)

add_executable(conversion_service_client
  src/conversion_service_client.cpp
  src/conversion_service_client_node.cpp)
ament_target_dependencies(conversion_service_client
  rclcpp rcutils geometry_msgs leap_msgs leap_srvs)

ament_export_include_directories(include)

install(TARGETS
  hand_publisher
  conversion_service
  conversion_service_client
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
