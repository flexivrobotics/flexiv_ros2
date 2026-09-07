cmake_minimum_required(VERSION 3.16)
project(flexiv_robot_states_broadcaster LANGUAGES CXX)

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
find_package(builtin_interfaces REQUIRED)
find_package(controller_interface REQUIRED)
find_package(rcutils REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(flexiv_hardware REQUIRED)
find_package(flexiv_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(flexiv_rdk REQUIRED)
find_package(flexiv_drdk QUIET)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  builtin_interfaces
  controller_interface
  flexiv_hardware
  flexiv_msgs
  generate_parameter_library
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  rcutils
  realtime_tools
)

generate_parameter_library(flexiv_robot_states_broadcaster_parameters
  src/flexiv_robot_states_broadcaster_parameters.yaml
)

add_library(flexiv_robot_states_broadcaster
  SHARED
  src/flexiv_robot_states_broadcaster.cpp
)

target_include_directories(flexiv_robot_states_broadcaster PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(flexiv_robot_states_broadcaster PUBLIC
  flexiv_robot_states_broadcaster_parameters
  flexiv::flexiv_rdk
)

target_link_libraries(flexiv_robot_states_broadcaster PUBLIC
  builtin_interfaces::builtin_interfaces
  controller_interface::controller_interface
  flexiv_hardware::flexiv_hardware
  flexiv_msgs::flexiv_msgs
  geometry_msgs::geometry_msgs
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp_lifecycle::rclcpp_lifecycle
  rcutils::rcutils
  realtime_tools::realtime_tools
)

# prevent pluginlib from using boost
target_compile_definitions(flexiv_robot_states_broadcaster PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface flexiv_robot_states_broadcaster.xml)

install(
  DIRECTORY include/
  DESTINATION include/flexiv_robot_states_broadcaster
)

install(
  TARGETS
    flexiv_robot_states_broadcaster
    flexiv_robot_states_broadcaster_parameters
  EXPORT export_flexiv_robot_states_broadcaster
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_flexiv_robot_states_broadcaster HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
