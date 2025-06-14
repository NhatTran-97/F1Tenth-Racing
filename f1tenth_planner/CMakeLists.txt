cmake_minimum_required(VERSION 3.5)
project(f1tenth_planner)

find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(ackermann_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)


set(local_dynamic_occ_grid_path "src/dynamic_occ_gridmap")

add_library(local_dynamic_occupancy
  ${local_dynamic_occ_grid_path}/local_dynamic_occ_grid.cpp)
ament_target_dependencies(local_dynamic_occupancy
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)
target_include_directories(local_dynamic_occupancy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>$<INSTALL_INTERFACE:include>)
add_executable(local_dynamic_occ_node ${local_dynamic_occ_grid_path}/main.cpp)
target_link_libraries(local_dynamic_occ_node local_dynamic_occupancy)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
