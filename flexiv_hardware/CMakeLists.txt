cmake_minimum_required(VERSION 3.5)
project(flexiv_hardware)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(BUILD_FOR_ARM64 "Link to RDK library for arm64 processor, otherwise link to x64" OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(flexiv_msgs REQUIRED)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/flexiv_hardware_interface.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/rdk/include
)

# Link arm64 or x64 version of libFlexivRdk
if (${BUILD_FOR_ARM64})
  target_link_libraries(${PROJECT_NAME}
    ${CMAKE_CURRENT_SOURCE_DIR}/rdk/lib/libFlexivRdk.arm64-darwin.a)
else()
  target_link_libraries(${PROJECT_NAME}
    ${CMAKE_CURRENT_SOURCE_DIR}/rdk/lib/libFlexivRdk.x86_64-linux-gnu.a)
endif()

# Link ROS packages
ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  flexiv_msgs
)

pluginlib_export_plugin_description_file(hardware_interface flexiv_hardware_interface_plugin.xml)

# INSTALL
install(
  TARGETS
    ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

# EXPORTS
ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  flexiv_msgs
)

ament_package()
