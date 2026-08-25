# Maximum contact torques of the robot's joint motion controller, used in the joint impedance
# control modes. Mirrors the argument of flexiv::rdk::Robot::SetMaxContactTorque().

# Header
std_msgs/Header header

# Joint names in URDF order. Every array below is indexed by this.
string[] joint_names

# Maximum contact torque the controller regulates its output to maintain. [Nm]
float64[] max_contact_torques

# Upper software limits of joint torques, the upper bound of max_contact_torques. [Nm]
float64[] tau_max

# Whether these values are in effect on the robot. False while the robot is not in a joint impedance
# control mode: they are held and applied on the next controller start.
bool in_effect
