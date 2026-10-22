cmake_minimum_required(VERSION 3.16)
project(flexiv_gripper)

# ROS 2 Lyrical requires C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(flexiv_hardware REQUIRED)
find_package(flexiv_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(flexiv_rdk REQUIRED)
find_package(flexiv_drdk QUIET)

add_library(gripper_action_server
  SHARED
  src/gripper_action_server.cpp)

add_executable(wait_for_gripper_ready
  src/wait_for_gripper_ready.cpp)

target_include_directories(gripper_action_server PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(gripper_action_server PUBLIC
  flexiv::flexiv_rdk
  control_msgs::control_msgs
  flexiv_hardware::flexiv_hardware
  flexiv_msgs::flexiv_msgs
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_components::component
  sensor_msgs::sensor_msgs
  std_msgs::std_msgs
  std_srvs::std_srvs
)

target_link_libraries(wait_for_gripper_ready PUBLIC
  rclcpp::rclcpp
  std_msgs::std_msgs
)

rclcpp_components_register_node(gripper_action_server
  PLUGIN "flexiv_gripper::GripperActionServer"
  EXECUTABLE flexiv_gripper_node)

install(TARGETS gripper_action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS wait_for_gripper_ready
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
