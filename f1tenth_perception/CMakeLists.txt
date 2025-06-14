cmake_minimum_required(VERSION 3.5)
project(f1tenth_perception)

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

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
# find_package(tf_transformations REQUIRED)
find_package(rclpy REQUIRED)


set(f1tenth_lidar_perception_dir "${CMAKE_CURRENT_SOURCE_DIR}/f1tenth_lidar_perception")

install(
  DIRECTORY
  f1tenth_perception/build
  #f1tenth_perception/yolov7
  f1tenth_perception/Yolov7_ByteTracking
  
 # f1tenth_perception/
  DESTINATION lib/f1tenth_perception
)



install(DIRECTORY
      launch meshes
      DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/__init__.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/yolov7_objectDetection.py
  ${PROJECT_NAME}/yoloDet.py
  ${PROJECT_NAME}/object_coordinate.py    
  ${PROJECT_NAME}/object_marker_array.py 
 
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  ${f1tenth_lidar_perception_dir}/lidar_object_coordinates.py   
  # ${f1tenth_lidar_perception_dir}/lidar_perception.py

  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
