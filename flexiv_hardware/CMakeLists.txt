cmake_minimum_required(VERSION 3.5)
project(flexiv_hardware)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(flexiv_rdk REQUIRED)
find_package(flexiv_drdk QUIET)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/flexiv_hardware_interface.cpp
)

if(flexiv_drdk_FOUND)
  target_sources(${PROJECT_NAME} PRIVATE src/flexiv_dual_hardware_interface.cpp)
endif()

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}
  flexiv::flexiv_rdk
)

if(flexiv_drdk_FOUND)
  target_link_libraries(${PROJECT_NAME}
    flexiv::flexiv_drdk
  )
  target_compile_definitions(${PROJECT_NAME} PRIVATE FLEXIV_DRDK_AVAILABLE)
  message(STATUS "Flexiv DRDK library found and linked.")
endif()

# Link ROS packages
ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
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
  rclcpp_lifecycle
)

ament_package()
