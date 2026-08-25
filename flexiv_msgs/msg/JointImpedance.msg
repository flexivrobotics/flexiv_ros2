# Impedance properties of the robot's joint motion controller, used in the joint impedance control
# modes. Mirrors the arguments of flexiv::rdk::Robot::SetJointImpedance().

# Header
std_msgs/Header header

# Joint names in URDF order. Every array below is indexed by this.
string[] joint_names

# Joint motion stiffness K_q. A joint at 0 is free-floating. [Nm/rad]
float64[] k_q

# Joint motion damping ratio Z_q, dimensionless. Valid range [0.3, 0.8], nominal 0.7
float64[] z_q

# Nominal joint motion stiffness, the upper bound of k_q. [Nm/rad]
float64[] k_q_nom

# Whether these values are in effect on the robot. False while the robot is not in a joint impedance
# control mode: they are held and applied on the next controller start.
bool in_effect
