# Cartesian motion-force command for one robot, consumed by cartesian_motion_force_controller and
# forwarded to flexiv::rdk::Robot::SendCartesianMotionForce().

# Header. frame_id is ignored: the pose is always expressed in the robot's world frame.
std_msgs/Header header

# Target TCP pose in world frame. The orientation is normalized by the controller. [m]
geometry_msgs/Pose pose

# Target TCP wrench in the force control frame, only tracked on axes enabled for force control. It
# is the wrench sensed at the TCP, not exerted by it: +5 N in world Z pushes the TCP down. [N]:[Nm]
geometry_msgs/Wrench wrench

# Target TCP velocity in world frame when reaching the pose. Leave 0 for the most robust
# behaviour. [m/s]:[rad/s]
geometry_msgs/Twist velocity
