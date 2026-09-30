#!/usr/bin/env python3
"""Cartesian motion-force control through cartesian_motion_force_controller.

The ROS 2 counterpart of the RDK examples
intermediate5_realtime_cartesian_pure_motion_control.cpp (mode:=pure_motion) and
intermediate6_realtime_cartesian_motion_force_control.cpp (mode:=motion_force), for a
single Rizon arm driven with robot_controller:=cartesian_motion_force_controller.
"""

import math

import rclpy
from rclpy.node import Node

from flexiv_msgs.msg import CartesianMotionForce, RobotStates
from flexiv_msgs.srv import (
    SetCartesianImpedance,
    SetCartesianMotionLimits,
    SetForceControlAxis,
    SetForceControlFrame,
    SetMaxContactWrench,
    SetNullSpacePosture,
)

SWING_AMP = 0.1  # TCP sine-sweep amplitude [m]
SWING_FREQ = 0.3  # TCP sine-sweep frequency [Hz]
PRESSING_FORCE = 5.0  # Force applied during motion-force control [N]
SEARCH_VELOCITY = 0.02  # Linear velocity used to search for contact [m/s]
SEARCH_DISTANCE = 1.0  # Maximum distance to travel when searching for contact [m]
MAX_WRENCH_FOR_CONTACT_SEARCH = [10.0, 10.0, 10.0, 3.0, 3.0, 3.0]
DEFAULT_MOTION_LIMITS = [
    0.5,
    1.0,
    2.0,
    5.0,
]  # RDK defaults: lin vel, ang vel, lin acc, ang acc
PREFERRED_POSTURE_A = [0.938, -1.108, -1.254, 1.464, 1.073, 0.278, -0.658]
PREFERRED_POSTURE_B = [-0.938, -1.108, 1.254, 1.464, -1.073, 0.278, 0.658]
SERVICE_TIMEOUT = 10.0  # [s]


class CartesianMotionForceExample(Node):
    def __init__(self):
        super().__init__("cartesian_motion_force_example")

        self.declare_parameter("robot_sn", "Rizon4-000000")
        self.declare_parameter("controller_name", "cartesian_motion_force_controller")
        self.declare_parameter("mode", "pure_motion")
        self.declare_parameter("hold", False)
        self.declare_parameter("polish", False)
        self.declare_parameter("force_frame", "world")
        self.declare_parameter("publish_rate", 100.0)

        robot_sn = self.get_parameter("robot_sn").value.replace("-", "_")
        controller_name = self.get_parameter("controller_name").value
        self.mode = self.get_parameter("mode").value
        self.hold = self.get_parameter("hold").value
        self.polish = self.get_parameter("polish").value
        self.force_frame = self.get_parameter("force_frame").value
        self.rate = self.get_parameter("publish_rate").value
        if self.mode not in ("pure_motion", "motion_force"):
            raise ValueError(
                f"mode must be pure_motion or motion_force, got '{self.mode}'"
            )
        if self.force_frame not in ("world", "tcp"):
            raise ValueError(
                f"force_frame must be world or tcp, got '{self.force_frame}'"
            )

        self.states = None
        self.create_subscription(
            RobotStates, f"/{robot_sn}/flexiv_robot_states", self.states_callback, 10
        )
        self.command_publisher = self.create_publisher(
            CartesianMotionForce, f"/{controller_name}/cartesian_motion_force", 10
        )
        self.config_ns = f"/{robot_sn}/flexiv_cartesian_motion_force_config_node"
        self.clients_by_name = {}

        self.loop_counter = 0
        self.init_pose = None
        self.init_q = None
        self.k_x_nom = None
        self.target_wrench = [0.0] * 6

    def states_callback(self, msg):
        self.states = msg

    # ---------------------------------------------------------------- services
    def request(self, srv_type, name, **fields):
        """Send a request and return its future, logging the outcome."""
        client = self.clients_by_name.get(name)
        if client is None:
            client = self.create_client(srv_type, f"{self.config_ns}/{name}")
            self.clients_by_name[name] = client
        if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            raise RuntimeError(f"Service {self.config_ns}/{name} is not available")
        future = client.call_async(srv_type.Request(**fields))
        future.add_done_callback(lambda f: self.log_response(name, f))
        return future

    def request_and_wait(self, srv_type, name, **fields):
        """Blocking variant of request(), for the setup steps."""
        future = self.request(srv_type, name, **fields)
        rclpy.spin_until_future_complete(self, future, timeout_sec=SERVICE_TIMEOUT)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(f"{name} failed")
        return response

    def log_response(self, name, future):
        response = future.result()
        if response is None:
            self.get_logger().error(f"{name}: no response")
        elif response.success:
            self.get_logger().info(f"{name}: {response.message}")
        else:
            self.get_logger().error(f"{name}: {response.message}")

    # ----------------------------------------------------------------- helpers
    def wait_for_states(self):
        self.get_logger().info("Waiting for robot states ...")
        while rclpy.ok() and self.states is None:
            rclpy.spin_once(self, timeout_sec=0.1)

    def tcp_pose(self):
        pose = self.states.tcp_pose.pose
        p, q = pose.position, pose.orientation
        return [p.x, p.y, p.z, q.w, q.x, q.y, q.z]

    def external_force_norm(self):
        force = self.states.ext_wrench_in_world.wrench.force
        return math.sqrt(force.x**2 + force.y**2 + force.z**2)

    def publish(self, pose, wrench):
        msg = CartesianMotionForce()
        msg.header.stamp = self.get_clock().now().to_msg()
        p, q = msg.pose.position, msg.pose.orientation
        p.x, p.y, p.z, q.w, q.x, q.y, q.z = pose
        f, m = msg.wrench.force, msg.wrench.torque
        f.x, f.y, f.z, m.x, m.y, m.z = wrench
        self.command_publisher.publish(msg)

    def swept_pose(self):
        """Initial pose, swept along world Y unless holding."""
        pose = list(self.init_pose)
        if not self.hold:
            t = self.loop_counter / self.rate
            pose[1] += SWING_AMP * math.sin(2 * math.pi * SWING_FREQ * t)
        return pose

    # ------------------------------------------------------------- pure motion
    def setup_pure_motion(self):
        self.get_logger().info(
            "Pure motion control: "
            + ("holding the TCP" if self.hold else "TCP sine-sweep")
        )
        # All axes motion-controlled, and the nominal stiffness reported back for later scaling.
        self.request_and_wait(
            SetForceControlAxis, "set_force_control_axis", enabled_axes=[False] * 6
        )
        self.k_x_nom = list(
            self.request_and_wait(
                SetCartesianImpedance, "set_cartesian_impedance"
            ).k_x_nom
        )
        self.init_pose = self.tcp_pose()
        self.init_q = list(self.states.q)
        self.create_timer(1.0 / self.rate, self.pure_motion_step)

    def pure_motion_step(self):
        self.publish(self.swept_pose(), [0.0] * 6)

        # The same online changes as the RDK example, every 20 seconds
        second = int(self.rate)
        step = self.loop_counter % (20 * second)
        if step == 3 * second:
            self.request(
                SetNullSpacePosture,
                "set_null_space_posture",
                ref_positions=PREFERRED_POSTURE_A,
            )
        elif step == 6 * second:
            half = [k * 0.5 for k in self.k_x_nom]
            self.request(SetCartesianImpedance, "set_cartesian_impedance", k_x=half)
        elif step == 9 * second:
            self.request(
                SetNullSpacePosture,
                "set_null_space_posture",
                ref_positions=PREFERRED_POSTURE_B,
            )
        elif step == 12 * second:
            self.request(
                SetCartesianImpedance, "set_cartesian_impedance", k_x=self.k_x_nom
            )
        elif step == 14 * second:
            self.request(
                SetNullSpacePosture, "set_null_space_posture", ref_positions=self.init_q
            )
        elif step == 16 * second:
            self.request(
                SetMaxContactWrench,
                "set_max_contact_wrench",
                max_wrench=[10.0, 10.0, 10.0, 2.0, 2.0, 2.0],
            )
        elif step == 19 * second:
            self.request(
                SetMaxContactWrench, "set_max_contact_wrench", max_wrench=[math.inf] * 6
            )
        self.loop_counter += 1

    # ------------------------------------------------------------ motion force
    def setup_motion_force(self):
        self.get_logger().warn(
            "The driver zeroed the force/torque sensor when the controller started. Force control "
            "is only accurate if nothing was in contact with the robot then."
        )

        # Search for contact: move down slowly, with a small contact wrench for a soft contact.
        self.get_logger().info("Searching for contact ...")
        self.request_and_wait(
            SetMaxContactWrench,
            "set_max_contact_wrench",
            max_wrench=MAX_WRENCH_FOR_CONTACT_SEARCH,
        )
        limits = list(DEFAULT_MOTION_LIMITS)
        limits[0] = SEARCH_VELOCITY
        self.set_motion_limits(limits)

        target = self.tcp_pose()
        target[2] -= SEARCH_DISTANCE
        while rclpy.ok() and self.external_force_norm() <= PRESSING_FORCE:
            self.publish(target, [0.0] * 6)
            rclpy.spin_once(self, timeout_sec=1.0 / self.rate)
        self.get_logger().info("Contact detected at robot TCP")
        self.publish(self.tcp_pose(), [0.0] * 6)
        self.set_motion_limits(DEFAULT_MOTION_LIMITS)

        # Force control along Z of the chosen frame, motion control in all other axes.
        root_coord = SetForceControlFrame.Request.WORLD
        if self.force_frame == "tcp":
            root_coord = SetForceControlFrame.Request.TCP
        self.request_and_wait(
            SetForceControlFrame, "set_force_control_frame", root_coord=[root_coord]
        )
        self.request_and_wait(
            SetForceControlAxis,
            "set_force_control_axis",
            enabled_axes=[False, False, True, False, False, False],
        )
        # Only after Z is force-controlled, so the contact force does not spike once released.
        self.request_and_wait(
            SetMaxContactWrench, "set_max_contact_wrench", max_wrench=[math.inf] * 6
        )

        # Sensed-wrench convention: +Z in world presses down, while TCP Z points into the part.
        fz = PRESSING_FORCE if self.force_frame == "world" else -PRESSING_FORCE
        self.target_wrench = [0.0, 0.0, fz, 0.0, 0.0, 0.0]
        self.hold = not self.polish
        self.init_pose = self.tcp_pose()
        self.get_logger().info(
            f"Pressing with {PRESSING_FORCE} N along {self.force_frame} Z"
            + (", polishing along world Y" if self.polish else "")
        )
        self.create_timer(1.0 / self.rate, self.motion_force_step)

    def motion_force_step(self):
        self.publish(self.swept_pose(), self.target_wrench)
        self.loop_counter += 1

    def set_motion_limits(self, limits):
        self.request_and_wait(
            SetCartesianMotionLimits,
            "set_cartesian_motion_limits",
            max_linear_vel=[limits[0]],
            max_angular_vel=[limits[1]],
            max_linear_acc=[limits[2]],
            max_angular_acc=[limits[3]],
        )

    def run(self):
        self.wait_for_states()
        if self.mode == "pure_motion":
            self.setup_pure_motion()
        else:
            self.setup_motion_force()
        rclpy.spin(self)


def main(args=None):
    rclpy.init(args=args)
    node = CartesianMotionForceExample()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
