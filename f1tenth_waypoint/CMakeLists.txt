cmake_minimum_required(VERSION 3.5)
project(f1tenth_waypoint)

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
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(OGRE REQUIRED)
find_package(rviz_default_plugins REQUIRED)



set(waypoint_generator_cpp_dir "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(waypoint_generator_python_dir "${CMAKE_CURRENT_SOURCE_DIR}/f1tenth_waypoint_generator")
set(f1tenth_waypoint_follower "${CMAKE_CURRENT_SOURCE_DIR}/f1tenth_waypoint_follower")


# ament_python_install_package(${PROJECT_NAME})
ament_python_install_package(f1tenth_waypoint_follower)
ament_python_install_package(f1tenth_waypoint_generator)

# #################################################################====BEGIN WAYPOINT_PUBLISHER_CPP===###################################################

add_executable(waypoint_publisher_node src/waypoint_publisher/waypoint_publisher.cpp)
# link dependencies
ament_target_dependencies(waypoint_publisher_node
  rclcpp
  nav_msgs
  geometry_msgs
  ament_index_cpp)

# include header files
target_include_directories(waypoint_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>$<INSTALL_INTERFACE:include>)

# Require C99 and C++17
target_compile_features(waypoint_publisher_node PUBLIC c_std_99 cxx_std_17)

# #################################################################====END WAYPOINT_PUBLISHER_CPP===###################################################

# #################################################################====BEGIN WAYPOINT_GENERATOR_CPP===###################################################

# Add library for the CSVFile class
add_library(utils_waypoint
  ${waypoint_generator_cpp_dir}/utils/waypoint_utils.cpp)

ament_target_dependencies(utils_waypoint
  rclcpp
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  tf2
  visualization_msgs
)

# Optionally link stdc++fs only if needed for older GCC/Clang versions
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_link_libraries(utils_waypoint stdc++fs)
endif()

target_link_libraries(utils_waypoint stdc++fs)

target_include_directories(utils_waypoint
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>$<INSTALL_INTERFACE:include>
)

add_library(odom_waypoint
  ${waypoint_generator_cpp_dir}/waypoint_generator/record_odom_waypoints.cpp)
ament_target_dependencies(odom_waypoint
  rclcpp)
target_include_directories(odom_waypoint
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>$<INSTALL_INTERFACE:include>)

target_link_libraries(odom_waypoint utils_waypoint)

add_library(clicked_waypoint
  ${waypoint_generator_cpp_dir}/waypoint_generator/record_clicked_waypoints.cpp)
ament_target_dependencies(clicked_waypoint
  rclcpp
  geometry_msgs)

target_include_directories(clicked_waypoint
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>$<INSTALL_INTERFACE:include>)

target_link_libraries(clicked_waypoint utils_waypoint)


add_executable(waypoint_generator_node ${waypoint_generator_cpp_dir}/waypoint_generator_node.cpp)

# link dependencies
ament_target_dependencies(waypoint_generator_node
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs)

# include header files
target_include_directories(waypoint_generator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>$<INSTALL_INTERFACE:include>)

# Require C99 and C++17
target_compile_features(waypoint_generator_node PUBLIC c_std_99 cxx_std_17)

target_link_libraries(waypoint_generator_node odom_waypoint clicked_waypoint)

# #################################################################====END_WAYPOINT_GENERATOR_CPP===#######################################################

# #################################################################====BEGIN INSTALL TARGETS===#######################################################
install(TARGETS
  waypoint_generator_node
  waypoint_publisher_node

  DESTINATION lib/${PROJECT_NAME})

# #################################################################====END INSTALL TARGETS===#######################################################

# #################################################################====BEGIN WAYPOINT_GENERATOR_PYTHON===###################################################
install(PROGRAMS
  ${f1tenth_waypoint_follower}/waypoint_follower.py
  ${f1tenth_waypoint_follower}/read_waypoint.py
  ${f1tenth_waypoint_follower}/waypoint_writer.py

  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS

  ${waypoint_generator_python_dir}/waypoint_generator_node.py
  ${waypoint_generator_python_dir}/__init__.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  ${waypoint_generator_python_dir}/waypoint_generation_options
  ${waypoint_generator_python_dir}/utils

  DESTINATION lib/${PROJECT_NAME})

# #################################################################====END WAYPOINT_GENERATOR_PYTHON===###################################################

# #################################################################====BEGIN LAUNCH CONFIGURATION===###################################################
install(DIRECTORY
  launch
  racelines
  rviz
  config
  DESTINATION share/${PROJECT_NAME})

# #################################################################====END LAUNCH CONFIGURATION===###################################################

# waypoint pannel below is currently under development

# set(CMAKE_AUTOMOC ON)

# add_library(waypoint_panel SHARED src/waypoint_plugin/waypoint_panel.cpp ${MOC_FILES})

# target_include_directories(waypoint_panel PUBLIC
# $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>$<INSTALL_INTERFACE:include>)

# ament_target_dependencies(waypoint_panel
# rclcpp
# rviz_common
# geometry_msgs
# pluginlib
# rviz_rendering)

# install(TARGETS waypoint_panel
# EXPORT export_f1tenth_waypoint_follower
# ARCHIVE DESTINATION lib
# LIBRARY DESTINATION lib
# RUNTIME DESTINATION bin
# )

# install(FILES
# plugin_description.xml
# DESTINATION share/${PROJECT_NAME}
# )
# ament_export_include_directories(include)
# ament_export_targets(export_f1tenth_waypoint_follower)

# pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
