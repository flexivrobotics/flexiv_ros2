#ifndef FLEXIV_HARDWARE__FLEXIV_ROBOT_STATES_HANDLE_HPP_
#define FLEXIV_HARDWARE__FLEXIV_ROBOT_STATES_HANDLE_HPP_

#include "flexiv/rdk/data.hpp"

namespace flexiv_hardware {

// ros2_control state interfaces only transport doubles, so robot-state handles are exported as
// exact positive integer IDs and resolved through a registry instead of bit-casting raw pointers.
double register_robot_states_handle(flexiv::rdk::RobotStates* robot_states);

void unregister_robot_states_handle(double encoded_handle);

flexiv::rdk::RobotStates* resolve_robot_states_handle(double encoded_handle);

} // namespace flexiv_hardware

#endif /* FLEXIV_HARDWARE__FLEXIV_ROBOT_STATES_HANDLE_HPP_ */
