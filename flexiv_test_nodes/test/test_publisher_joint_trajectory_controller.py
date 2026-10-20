import pytest
import rclpy
from sensor_msgs.msg import JointState

from flexiv_test_nodes.publisher_joint_trajectory_controller import (
    PublisherJointTrajectory,
)

GOAL_ARGS = ["-p", "goal_names:=[pos1]", "-p", "pos1:=[0.0, 0.0]"]


def make_node(extra_args):
    rclpy.init(args=["--ros-args", *GOAL_ARGS, *extra_args])
    try:
        return PublisherJointTrajectory()
    except Exception:
        rclpy.shutdown()
        raise


@pytest.fixture
def node():
    node = make_node(["-p", "joints:=[j1, j2]", "-p", "check_starting_point:=true"])
    yield node
    node.destroy_node()
    rclpy.shutdown()


def joint_state(names, positions):
    return JointState(name=names, position=positions)


def test_extra_joints_are_ignored(node):
    node.joint_state_callback(
        joint_state(["finger_width_joint", "j2", "j1"], [0.05, 0.1, 0.2])
    )
    assert node.joint_state_msg_received
    assert node.starting_point_ok


def test_waits_for_all_configured_joints(node):
    node.joint_state_callback(joint_state(["finger_width_joint", "j1"], [0.05, 0.0]))
    assert not node.joint_state_msg_received
    assert not node.starting_point_ok


def test_limits_are_checked_per_joint_name(node):
    node.joint_state_callback(joint_state(["j2", "j1"], [7.0, 0.0]))
    assert node.joint_state_msg_received
    assert not node.starting_point_ok


@pytest.mark.parametrize("joint_args", [[], ["-p", "joints:=['']"]])
def test_unset_joints_are_rejected(joint_args):
    with pytest.raises(Exception, match="joints"):
        make_node(joint_args)
