cmake_minimum_required(VERSION 3.5)
project(f1tenth_visualize)

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
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(visualization_msgs REQUIRED)



include_directories(include)


install(
  DIRECTORY include/
  DESTINATION include)

install(TARGETS

  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
   ${PROJECT_NAME}/visualize_marker_text.py    
  # ${PROJECT_NAME}/one_d_kalmanFilter.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
      launch
      DESTINATION share/${PROJECT_NAME})



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
