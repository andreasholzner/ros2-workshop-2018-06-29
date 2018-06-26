cmake_minimum_required(VERSION 3.5)
project(cpp_demo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rcutils REQUIRED)

add_executable(demo cpp_demo.cpp)
ament_target_dependencies(demo rcutils)

install(TARGETS demo DESTINATION bin)

ament_package()
