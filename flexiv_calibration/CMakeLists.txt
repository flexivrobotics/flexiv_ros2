cmake_minimum_required(VERSION 3.5)
project(flexiv_calibration)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# The RDK archive for ROS 2 is built against the ROS and system copies of fastcdr and spdlog,
# but the RDK install prefix also ships older ones. Claim both imported targets up front, before
# any transitive find_dependency() can define them from that prefix. NO_CMAKE_PATH skips it,
# since it is the only entry in the CMAKE_PREFIX_PATH variable.
find_package(fastcdr REQUIRED NO_CMAKE_PATH)
find_package(spdlog REQUIRED NO_CMAKE_PATH)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(flexiv_rdk REQUIRED)

add_executable(calibration_correction
  src/calibration_correction.cpp)

target_link_libraries(calibration_correction
  flexiv::flexiv_rdk
  fastcdr
  spdlog::spdlog
)

ament_target_dependencies(calibration_correction
  ament_index_cpp
  rclcpp)

install(TARGETS calibration_correction
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
