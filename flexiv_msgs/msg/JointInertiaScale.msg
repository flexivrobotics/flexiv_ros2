# Inertia shaping scales of the robot's joint motion controller, used in the joint impedance control
# modes. Mirrors the argument of flexiv::rdk::Robot::SetJointInertiaScale().

# Header
std_msgs/Header header

# Joint names in URDF order. Every array below is indexed by this.
string[] joint_names

# Inertia shaping scale sigma_q, dimensionless. Valid range [0.75, 1.0]. The nominal value 1.0 means
# no shaping; a smaller scale makes the joint behave as if it were lighter.
float64[] inertia_scales

# Whether these values are in effect on the robot. False while the robot is not in a joint impedance
# control mode: they are held and applied on the next controller start.
bool in_effect
