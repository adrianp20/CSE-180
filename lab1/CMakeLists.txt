cmake_minimum_required(VERSION 3.5)
project(lab1)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(intpublisher src/intpublisher.cpp)
add_executable(intsubscriber src/intsubscriber.cpp)
ament_target_dependencies(intpublisher rclcpp std_msgs)
ament_target_dependencies(intsubscriber rclcpp std_msgs)

install(TARGETS
  intpublisher
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  intsubscriber
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
