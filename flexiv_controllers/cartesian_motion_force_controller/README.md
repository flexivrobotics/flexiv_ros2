# cartesian_motion_force_controller

Controller forwarding Cartesian motion-force commands to the Flexiv hardware interface, which sends them to the robot with `SendCartesianMotionForce()`.

- **Parameter** `prefixes`: the joint prefix of each robot to command, naming its `<prefix>tcp` interfaces. One entry for a single robot (`<robot_sn>_`), left then right (`left_<robot_sn>_`, `right_<robot_sn>_`) for a robot pair.
- **Command interfaces** per robot: `cartesian_pose_{x,y,z,qw,qx,qy,qz}`, `cartesian_wrench_{fx,fy,fz,mx,my,mz}`, `cartesian_velocity_{vx,vy,vz,wx,wy,wz}`.
- **State interfaces** per robot: `cartesian_pose_{x,y,z,qw,qx,qy,qz}`.
- **Topic** `~/cartesian_motion_force` (`flexiv_msgs/msg/CartesianMotionForce`). For a robot pair, one per robot: `~/<prefix>/cartesian_motion_force`, without the trailing `_` and with `-` replaced by `_`.

On activation the controller holds the measured TCP pose, and afterwards the last valid command; there is no timeout. A message with a non-finite field or a zero quaternion is dropped as a whole, and the quaternion is normalized. `header.frame_id` is ignored: the pose is always in the world frame.

The force control settings (axes, frame, impedance, contact wrench, null space, motion limits) are services of the hardware interface, see [`flexiv_hardware`](../../flexiv_hardware/README.md#cartesian-motion-force-configuration).
