cmake_minimum_required(VERSION 3.5)
project(flexiv_gripper)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(flexiv_hardware REQUIRED)
find_package(flexiv_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(gripper_action_server
  SHARED
  src/gripper_action_server.cpp)

target_include_directories(gripper_action_server PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../flexiv_hardware/rdk/include)

ament_target_dependencies(gripper_action_server
  flexiv_hardware
  flexiv_msgs
  rclcpp
  rclcpp_action
  rclcpp_components
  sensor_msgs
  std_srvs)

rclcpp_components_register_nodes(gripper_action_server
  PLUGIN "flexiv_gripper::GripperActionServer"
  EXECUTABLE flexiv_gripper_node)

install(TARGETS gripper_action_server
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
