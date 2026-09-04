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
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(flexiv_msgs REQUIRED)
find_package(flexiv_rdk REQUIRED)
find_package(flexiv_drdk QUIET)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/flexiv_hardware_interface.cpp
  src/fault_recovery.cpp
  src/recovery_node.cpp
  src/joint_impedance_config_node.cpp
)

if(flexiv_drdk_FOUND)
  target_sources(${PROJECT_NAME} PRIVATE src/flexiv_dual_hardware_interface.cpp)
endif()

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
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
else()
  message(STATUS
    "Flexiv DRDK library not found: FlexivDualHardwareInterface is not built, so the "
    "paired robot types (Rizon-Dual and the AICO2 platforms) cannot run on real "
    "hardware. They still work with use_fake_hardware:=true.")
endif()

# Link ROS packages
ament_target_dependencies(
  ${PROJECT_NAME}
  flexiv_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface flexiv_hardware_interface_plugin.xml)

# FlexivDualHardwareInterface only exists when DRDK was found. Declaring it
# unconditionally makes pluginlib resolve a class that is absent from the
# library, which fails in the class loader with no usable diagnostic.
if(flexiv_drdk_FOUND)
  pluginlib_export_plugin_description_file(hardware_interface flexiv_dual_hardware_interface_plugin.xml)
endif()

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

install(
  DIRECTORY ros2_control urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_fault_recovery test/test_fault_recovery.cpp)
  target_include_directories(test_fault_recovery PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_fault_recovery ${PROJECT_NAME} flexiv::flexiv_rdk)

  ament_add_gtest(test_joint_impedance_config_node test/test_joint_impedance_config_node.cpp)
  target_include_directories(test_joint_impedance_config_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_joint_impedance_config_node ${PROJECT_NAME} flexiv::flexiv_rdk)
endif()

# EXPORTS
ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  flexiv_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
)

ament_package()
