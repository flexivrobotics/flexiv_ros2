# Header
std_msgs/Header header

# Robot system timestamp
builtin_interfaces/Time robot_timestamp

# Measured joint positions using link-side encoder
float64[] q

# Measured joint positions using motor-side encoder
float64[] theta

# Measured joint velocities using link-side encoder
float64[] dq

# Measured joint velocities using motor-side encoder
float64[] dtheta

# Measured joint torques
float64[] tau

# Numerical derivative of measured joint torques
float64[] tau_dot

# Estimated external joint torques
float64[] tau_ext

# Estimated interaction joint torques
float64[] tau_interact

# Measured joint temperatures
float64[] temperature

# Measured TCP pose expressed in world frame
geometry_msgs/PoseStamped tcp_pose

# Measured TCP twist expressed in world frame
geometry_msgs/TwistStamped tcp_twist

# Measured flange pose expressed in world frame
geometry_msgs/PoseStamped flange_pose

# Raw force-torque (FT) sensor reading in flange frame
geometry_msgs/WrenchStamped raw_ft_sensor

# Estimated external wrench applied on TCP and expressed in TCP frame
geometry_msgs/WrenchStamped tcp_wrench_local

# Estimated external wrench applied on TCP and expressed in world frame
geometry_msgs/WrenchStamped tcp_wrench

# Unfiltered external wrench applied on TCP and expressed in TCP frame
geometry_msgs/WrenchStamped raw_tcp_wrench_local

# Unfiltered external wrench applied on TCP and expressed in world frame
geometry_msgs/WrenchStamped raw_tcp_wrench
