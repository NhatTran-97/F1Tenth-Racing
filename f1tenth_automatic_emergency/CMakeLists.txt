cmake_minimum_required(VERSION 3.5)
project(f1tenth_automatic_emergency)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)




include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

set(f1tenth_lab2_dir "${CMAKE_CURRENT_SOURCE_DIR}/lab2_safety")

add_executable(safety_node
              src/lab2_safety/safety.cpp)
ament_target_dependencies(safety_node rclcpp sensor_msgs std_msgs nav_msgs ackermann_msgs)




install(TARGETS
  safety_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  ${f1tenth_lab2_dir}/safety_node.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
