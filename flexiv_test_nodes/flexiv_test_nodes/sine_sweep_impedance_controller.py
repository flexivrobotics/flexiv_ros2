import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from flexiv_msgs.msg import JointPosVel

# Joint sine-sweep amplitude [rad]
SWING_AMP = 0.035

# TCP sine-sweep frequency [Hz]
SWING_FREQ = 0.3


class SineSweepImpedance(Node):
    def __init__(self):
        super().__init__("sine_sweep_impedance_controller")
        # Declare all parameters
        self.declare_parameter("controller_name", "joint_impedance_controller")
        self.declare_parameter("joints", [""])
        self.declare_parameter("wait_sec_between_publish", 0.001)
        self.declare_parameter("speed_scaling", 1.0)

        # Read parameters
        controller_name = self.get_parameter("controller_name").value
        self.joints = self.get_parameter("joints").value
        wait_sec_between_publish = self.get_parameter("wait_sec_between_publish").value
        self.speed_scaling = self.get_parameter("speed_scaling").value

        if self.joints is None or len(self.joints) == 0:
            raise Exception('"joints" parameter is not set!')

        publish_topic = "/" + controller_name + "/" + "commands"

        self.init_pos = [0.0] * len(self.joints)

        self.publisher_ = self.create_publisher(JointPosVel, publish_topic, 1)
        self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)
        self.joint_state_sub = self.create_subscription(
            JointState, "joint_states", self.joint_state_callback, 10
        )
        self.joint_state_msg_received = False
        self.loop_time = 0.0
        self.period = wait_sec_between_publish

    def timer_callback(self):
        if self.joint_state_msg_received:
            target_pos = self.init_pos.copy()
            for i in range(len(self.joints)):
                target_pos[i] = self.init_pos[i] + SWING_AMP * math.sin(
                    2 * math.pi * SWING_FREQ * self.speed_scaling * self.loop_time
                )
            msg = JointPosVel()
            msg.positions = target_pos
            self.publisher_.publish(msg)
            self.loop_time += self.period

    def joint_state_callback(self, msg):
        if not self.joint_state_msg_received:
            # retrieve joint states by the order of the joint names
            for name in self.joints:
                if name not in msg.name:
                    raise Exception(f"Joint {name} not found in joint_states!")
                joint_msg_index = msg.name.index(name)
                joint_position = msg.position[joint_msg_index]
                index = self.joints.index(name)
                self.init_pos[index] = joint_position
            self.joint_state_msg_received = True
        else:
            return


def main(args=None):
    rclpy.init(args=args)

    sine_sweep_impedance = SineSweepImpedance()

    rclpy.spin(sine_sweep_impedance)
    sine_sweep_impedance.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
