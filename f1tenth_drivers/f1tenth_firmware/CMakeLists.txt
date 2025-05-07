cmake_minimum_required(VERSION 3.5)
project(f1tenth_firmware)

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
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(JetsonGPIO REQUIRED)  # Ensure it's marked as REQUIRED if it must be found

include_directories(include)

set(DEPENDCIES_ROS  "rclcpp" "sensor_msgs" "JetsonGPIO")

# link_directories(/usr/local/lib) # not use


#add_executable(blink_led src/blink_led.cpp)
#ament_target_dependencies(blink_led rclcpp sensor_msgs)
#target_link_libraries(blink_led JetsonGPIO::JetsonGPIO)  # Changed to simple library name
#set_target_properties(blink_led PROPERTIES INSTALL_RPATH "$ORIGIN:/usr/local/lib") #(not use)

add_library(led  SHARED src/LED.cpp)
ament_target_dependencies(led ${DEPENDCIES_ROS} )
add_executable(blink_led src/main.cpp)
#ament_target_dependencies(blink_led ${DEPENDCIES_ROS} )
target_link_libraries(blink_led led)   # JetsonGPIO::JetsonGPIO
set_target_properties(blink_led PROPERTIES 
                              INSTALL_RPATH "$ORIGIN/../lib" 
                              BUILD_WITH_INSTALL_RPATH TRUE) #(not use)


install(TARGETS blink_led led
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib  
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  # DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
