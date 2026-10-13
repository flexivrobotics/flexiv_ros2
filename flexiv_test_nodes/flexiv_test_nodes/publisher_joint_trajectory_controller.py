# Copyright 2022 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.parameter import Parameter


class PublisherJointTrajectory(Node):
    def __init__(self):
        super().__init__("publisher_position_trajectory_controller")
        # Declare all parameters
        self.declare_parameter("controller_name", "joint_trajectory_controller")
        self.declare_parameter("wait_sec_between_publish", 6)
        self.declare_parameter("goal_names", ["pos1", "pos2"])
        self.declare_parameter("joints", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("check_starting_point", False)
        self.declare_parameter("starting_point_limits", False)

        # Read parameters
        controller_name = self.get_parameter("controller_name").value
        wait_sec_between_publish = self.get_parameter("wait_sec_between_publish").value
        goal_names = self.get_parameter("goal_names").value
        self.joints = self.get_parameter_or("joints").value
        self.check_starting_point = self.get_parameter("check_starting_point").value
        self.starting_point = {}

        if not self.joints or not all(self.joints):
            raise Exception('"joints" parameter is not set!')

        # starting point stuff
        if self.check_starting_point:
            # declare nested params
            for name in self.joints:
                param_name_tmp = "starting_point_limits" + "." + name
                self.declare_parameter(param_name_tmp, [-2 * 3.14159, 2 * 3.14159])
                self.starting_point[name] = self.get_parameter(param_name_tmp).value

            for name in self.joints:
                if len(self.starting_point[name]) != 2:
                    raise Exception('"starting_point" parameter is not set correctly!')
            self.joint_state_sub = self.create_subscription(
                JointState, "joint_states", self.joint_state_callback, 10
            )
        # initialize starting point status
        self.starting_point_ok = not self.check_starting_point

        self.joint_state_msg_received = False

        # Read all positions from parameters
        self.goals = []
        for name in goal_names:
            self.declare_parameter(
                name, descriptor=ParameterDescriptor(dynamic_typing=True)
            )
            goal = self.get_parameter(name).value
            if goal is None or len(goal) == 0:
                raise Exception(f'Values for goal "{name}" not set!')

            float_goal = []
            for value in goal:
                float_goal.append(float(value))
            self.goals.append(float_goal)

        publish_topic = "/" + controller_name + "/" + "joint_trajectory"

        self.get_logger().info(
            'Publishing {} goals on topic "{}" every {} s'.format(
                len(goal_names), publish_topic, wait_sec_between_publish
            )
        )

        self.publisher_ = self.create_publisher(JointTrajectory, publish_topic, 1)
        self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        if self.starting_point_ok:
            traj = JointTrajectory()
            traj.joint_names = self.joints
            point = JointTrajectoryPoint()
            point.positions = self.goals[self.i]
            point.time_from_start = Duration(sec=1)

            traj.points.append(point)
            self.publisher_.publish(traj)

            self.i += 1
            self.i %= len(self.goals)

        elif self.check_starting_point and not self.joint_state_msg_received:
            self.get_logger().warning(
                'Start configuration could not be checked! Check "joint_state" topic!'
            )
        else:
            self.get_logger().warning(
                "Start configuration is not within configured limits!"
            )

    def joint_state_callback(self, msg):
        if self.joint_state_msg_received:
            return
        positions = dict(zip(msg.name, msg.position))
        # Other joints, e.g. a gripper's, are ignored; wait for one with all of ours
        if any(name not in positions for name in self.joints):
            return

        limit_exceeded = False
        for name in self.joints:
            lower, upper = self.starting_point[name]
            if not lower <= positions[name] <= upper:
                self.get_logger().warning(
                    f"Starting point limits exceeded for joint {name} !"
                )
                limit_exceeded = True

        self.starting_point_ok = not limit_exceeded
        self.joint_state_msg_received = True


def main(args=None):
    rclpy.init(args=args)

    publisher_joint_trajectory = PublisherJointTrajectory()

    rclpy.spin(publisher_joint_trajectory)
    publisher_joint_trajectory.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
