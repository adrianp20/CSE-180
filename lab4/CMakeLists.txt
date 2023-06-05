cmake_minimum_required(VERSION 3.5)
project(lab4)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(drawsquarefb src/drawsquarefb.cpp)

ament_target_dependencies(drawsquarefb rclcpp geometry_msgs turtlesim nav_msgs tf2)

install(TARGETS drawsquarefb DESTINATION lib/${PROJECT_NAME})



ament_package()
