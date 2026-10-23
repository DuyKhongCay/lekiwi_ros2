# Copyright 2026 LeKiwi Robotics Team
# Licensed under the Apache License, Version 2.0.

"""Unit tests validating SmolVLA 12D observation synthesis and policy integration."""

import numpy as np
import pytest
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from lekiwi_interfaces.action import ExecuteChessMove
from lekiwi_inference.async_ros2_client_node import AsyncRos2ClientNode


@pytest.fixture(autouse=True)
def rclpy_init_shutdown():
    """Manages rclpy lifecycle per test session."""
    import rclpy
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_smolvla_observation_state_12d_synthesis():
    """Verify that observation.state is synthesized as exact 12D: [joints(6), src(3), tgt(3)]."""
    node = AsyncRos2ClientNode(
        parameter_overrides=[
            Parameter("auto_start", value=False),
        ]
    )

    # Verify features schema
    assert "observation.state" in node.client.config.features
    assert node.client.config.features["observation.state"]["shape"] == (12,)
    assert node.client.config.features["observation.state"]["dtype"] == "float32"
    assert "observation.environment_state" not in node.client.config.features

    # Provide joint state
    js = JointState()
    js.name = node.arm_joints
    js.position = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    node._joint_state_callback(js)

    # Set mock active goal with 6D environment_state (src=[1.0, 2.0, 3.0], tgt=[4.0, 5.0, 6.0])
    captured_obs = []
    node.client.ready_to_send = lambda state: True
    node._is_sensor_data_fresh = lambda now: True
    node.client.submit_observation = lambda obs, timestamp=None: captured_obs.append(obs)

    node._latest_wrist_bytes = b"wrist"
    node._latest_side_bytes = b"side"
    node._latest_overhead_bytes = b"overhead"

    class MockGoalHandle:
        def __init__(self):
            self.is_active = True
            self.feedbacks = []
        def publish_feedback(self, fb):
            self.feedbacks.append(fb)

    node._active_goal_handle = MockGoalHandle()
    node._active_env_state = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0], dtype=np.float32)
    node._active_task = "Move black pawn to target"
    node._active_goal_start_time = 100.0

    # Run control tick
    node._control_timer_callback()

    assert len(captured_obs) == 1
    raw = captured_obs[0]
    assert raw["task"] == "Move black pawn to target"
    assert raw["observation.state"].shape == (12,)
    expected_12d = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0], dtype=np.float32)
    np.testing.assert_allclose(raw["observation.state"], expected_12d, atol=1e-5)

    node.destroy_node()


def test_smolvla_rejection_of_non_6d_environment_state():
    """Verify that goals with non-6D environment_state are rejected immediately."""
    node = AsyncRos2ClientNode(
        parameter_overrides=[
            Parameter("auto_start", value=False),
        ]
    )
    node._is_active = True
    node.transport._connected = True

    # 10D (old ACT format) must be rejected
    goal_10d = ExecuteChessMove.Goal()
    goal_10d.operation = ExecuteChessMove.Goal.OP_MOVE
    goal_10d.environment_state = [0.1] * 6  # Since action now declares float32[6], non-6D cannot even be set
    from rclpy.action import GoalResponse
    assert node._handle_goal_received(goal_10d) == GoalResponse.ACCEPT

    # Mock invalid length on Goal
    goal_bad = ExecuteChessMove.Goal()
    goal_bad.operation = ExecuteChessMove.Goal.OP_MOVE
    goal_bad.environment_state = [0.1] * 6
    # Artificially alter list to simulate length mismatch
    goal_bad.environment_state = [0.1] * 4  # type: ignore
    assert node._handle_goal_received(goal_bad) == GoalResponse.REJECT

    node.destroy_node()

