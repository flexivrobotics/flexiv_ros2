cmake_minimum_required(VERSION 3.5)
project(flexiv_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# This package is built against the Flexiv RDK install prefix, which also provides
# a newer Fast-CDR than ROS 2 Humble's FastRTPS generators expect. The driver is
# CycloneDDS-only, so generate introspection typesupport and skip FastRTPS code.
set(STATIC_ROSIDL_TYPESUPPORT_C rosidl_typesupport_introspection_c)
set(STATIC_ROSIDL_TYPESUPPORT_CPP rosidl_typesupport_introspection_cpp)
set(_flexiv_msgs_rosidl_extensions "")
foreach(_extension ${AMENT_EXTENSIONS_rosidl_generate_idl_interfaces})
  if(NOT _extension MATCHES "^rosidl_typesupport_fastrtps_(c|cpp):")
    list(APPEND _flexiv_msgs_rosidl_extensions "${_extension}")
  endif()
endforeach()
set(AMENT_EXTENSIONS_rosidl_generate_idl_interfaces ${_flexiv_msgs_rosidl_extensions})
unset(_flexiv_msgs_rosidl_extensions)
unset(_extension)

set(msg_files
  msg/Digital.msg
  msg/GPIOStates.msg
  msg/JointPosVel.msg
  msg/Mode.msg
  msg/RobotStates.msg
)

# set(srv_files
# )

set(action_files
  action/Grasp.action
  action/Move.action
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  # ${srv_files}
  ${action_files}
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
