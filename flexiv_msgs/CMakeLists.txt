cmake_minimum_required(VERSION 3.5)
project(flexiv_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
  msg/Digital.msg
  msg/GPIOStates.msg
  msg/JointPosVel.msg
  msg/Mode.msg
  msg/RobotStates.msg
)

# set(srv_files
# )

# set(action_files
#   action/SetMode.action
# )

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  # ${srv_files}
  # ${action_files}
  DEPENDENCIES
    std_msgs
    geometry_msgs
    builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
