cmake_minimum_required(VERSION 3.5)
project(replicateoop)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(replicateoop src/replicateoop.cpp)
ament_target_dependencies(replicateoop rclcpp turtlesim geometry_msgs)

install(TARGETS
  replicateoop
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

