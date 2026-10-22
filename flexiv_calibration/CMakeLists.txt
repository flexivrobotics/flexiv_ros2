cmake_minimum_required(VERSION 3.16)
project(flexiv_calibration)

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
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(flexiv_rdk REQUIRED)

add_executable(calibration_correction
  src/calibration_correction.cpp)

target_link_libraries(calibration_correction PRIVATE
  ament_index_cpp::ament_index_cpp
  flexiv::flexiv_rdk
  rclcpp::rclcpp
)

install(TARGETS calibration_correction
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
