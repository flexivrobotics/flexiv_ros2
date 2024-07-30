cmake_minimum_required(VERSION 3.5)
project(tcp_pose_state_broadcaster LANGUAGES CXX)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(flexiv_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  builtin_interfaces
  controller_interface
  generate_parameter_library
  flexiv_msgs
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  rcutils
  realtime_tools
)

generate_parameter_library(tcp_pose_state_broadcaster_parameters
  src/tcp_pose_state_broadcaster_parameters.yaml
)

add_library(tcp_pose_state_broadcaster
  SHARED
  src/tcp_pose_state_broadcaster.cpp
)

target_include_directories(tcp_pose_state_broadcaster PRIVATE include)

target_link_libraries(tcp_pose_state_broadcaster PUBLIC
  tcp_pose_state_broadcaster_parameters
)

ament_target_dependencies(tcp_pose_state_broadcaster PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# prevent pluginlib from using boost
target_compile_definitions(tcp_pose_state_broadcaster PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface tcp_pose_state_broadcaster.xml)

install(
  DIRECTORY include/
  DESTINATION include/tcp_pose_state_broadcaster
)

install(
  TARGETS
    tcp_pose_state_broadcaster
    tcp_pose_state_broadcaster_parameters
  EXPORT export_tcp_pose_state_broadcaster
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_tcp_pose_state_broadcaster HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
