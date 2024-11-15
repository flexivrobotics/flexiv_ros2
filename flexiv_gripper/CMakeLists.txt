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
find_package(control_msgs REQUIRED)
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
  ${CMAKE_CURRENT_SOURCE_DIR}/../flexiv_hardware/rdk/include)

# Set static library
message("OS: ${CMAKE_SYSTEM_NAME}")
message("Processor: ${CMAKE_SYSTEM_PROCESSOR}")
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
    set(RDK_STATIC_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/../flexiv_hardware/rdk/lib/libflexiv_rdk.x86_64-linux-gnu.a")
  elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
    set(RDK_STATIC_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/../flexiv_hardware/rdk/lib/libflexiv_rdk.aarch64-linux-gnu.a")
  else()
    message(FATAL_ERROR "Linux with ${CMAKE_SYSTEM_PROCESSOR} processor is currently not supported.")
  endif()
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm64")
    set(RDK_STATIC_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/../flexiv_hardware/rdk/lib/libflexiv_rdk.arm64-darwin.a")
  else()
    message(FATAL_ERROR "Mac with ${CMAKE_SYSTEM_PROCESSOR} processor is currently not supported.")
  endif()
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "AMD64")
    set(RDK_STATIC_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/../flexiv_hardware/rdk/lib/flexiv_rdk.win_amd64.lib")
  else()
    message(FATAL_ERROR "Windows with ${CMAKE_SYSTEM_PROCESSOR} processor is currently not supported.")
  endif()
endif()

target_link_libraries(gripper_action_server
  ${RDK_STATIC_LIBRARY})

ament_target_dependencies(gripper_action_server
  control_msgs
  flexiv_hardware
  flexiv_msgs
  rclcpp
  rclcpp_action
  rclcpp_components
  sensor_msgs
  std_srvs)

rclcpp_components_register_node(gripper_action_server
  PLUGIN "flexiv_gripper::GripperActionServer"
  EXECUTABLE flexiv_gripper_node)

install(TARGETS gripper_action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
