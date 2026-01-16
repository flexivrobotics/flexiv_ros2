# Header
std_msgs/Header header

# Robot system timestamp
builtin_interfaces/Time robot_timestamp

# Measured joint positions using link-side encoder
float64[7] q

# Measured joint positions using motor-side encoder
float64[7] theta

# Measured joint velocities using link-side encoder
float64[7] dq

# Measured joint velocities using motor-side encoder
float64[7] dtheta

# Measured joint torques
float64[7] tau

# Desired joint torques
float64[7] tau_des

# Numerical derivative of measured joint torques
float64[7] tau_dot

# Estimated external joint torques
float64[7] tau_ext

# Estimated interaction joint torques
float64[7] tau_interact

# Measured joint temperatures
float64[7] temperature

# Measured TCP pose expressed in world frame
geometry_msgs/PoseStamped tcp_pose

# Measured TCP velocity expressed in world frame
geometry_msgs/AccelStamped tcp_vel

# Measured flange pose expressed in world frame
geometry_msgs/PoseStamped flange_pose

# Force-torque (FT) sensor raw reading in flange frame
geometry_msgs/WrenchStamped ft_sensor_raw

# Estimated external wrench applied on TCP and expressed in TCP frame
geometry_msgs/WrenchStamped ext_wrench_in_tcp

# Estimated external wrench applied on TCP and expressed in world frame
geometry_msgs/WrenchStamped ext_wrench_in_world

# Unfiltered external wrench applied on TCP and expressed in TCP frame
geometry_msgs/WrenchStamped ext_wrench_in_tcp_raw

# Unfiltered external wrench applied on TCP and expressed in world frame
geometry_msgs/WrenchStamped ext_wrench_in_world_raw
