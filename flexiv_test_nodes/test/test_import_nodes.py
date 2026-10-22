"""Smoke test: every node module imports cleanly.

This catches syntax errors and missing message-package dependencies
without needing a robot or a running ROS graph.
"""

import importlib

import pytest

# robot_states_publisher imports flexivrdk, the RDK Python binding, which is installed alongside
# flexiv_rdk rather than through rosdep.
MODULES = [
    "flexiv_test_nodes.cartesian_motion_force_example",
    "flexiv_test_nodes.publisher_joint_trajectory_controller",
    "flexiv_test_nodes.robot_states_monitor",
]
MODULES_NEEDING_RDK = ["flexiv_test_nodes.robot_states_publisher"]


@pytest.mark.parametrize("module_name", MODULES)
def test_module_imports(module_name):
    assert importlib.import_module(module_name) is not None


@pytest.mark.parametrize("module_name", MODULES_NEEDING_RDK)
def test_module_imports_with_rdk(module_name):
    pytest.importorskip(
        "flexivrdk", reason="flexivrdk is installed with flexiv_rdk, not rosdep"
    )
    assert importlib.import_module(module_name) is not None
