# Copyright 2026 LeKiwi Robotics Team
# Licensed under the Apache License, Version 2.0.

"""Unit tests validating SmolVLA 12D observation synthesis and policy integration."""

import numpy as np
import pytest
from geometry_msgs.msg import Point, TransformStamped
from rclpy.action import GoalResponse
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


def _mock_tf_transform(x: float = 0.15, y: float = 0.25, z: float = 0.35) -> TransformStamped:
    tf_stamped = TransformStamped()
    tf_stamped.transform.translation.x = float(x)
    tf_stamped.transform.translation.y = float(y)
    tf_stamped.transform.translation.z = float(z)
    return tf_stamped


def test_smolvla_observation_state_12d_synthesis():
    """Verify that observation.state is synthesized as exact 12D: [joints(6), ee(3), delta(3)]."""
    node = AsyncRos2ClientNode(
        parameter_overrides=[
            Parameter("auto_start", value=False),
        ]
    )

    # 1. Verify features schema
    assert "observation.state" in node.client.config.features
    assert node.client.config.features["observation.state"]["shape"] == (12,)
    assert node.client.config.features["observation.state"]["dtype"] == "float32"
    assert "observation.environment_state" not in node.client.config.features

    # 2. Provide joint state
    js = JointState()
    js.name = node.arm_joints
    js.position = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    node._joint_state_callback(js)

    # 3. Mock TF buffer for tool_frame ("gripper") -> returns ee_xyz = [0.15, 0.25, 0.35]
    node.tf_buffer.lookup_transform = lambda base, tool, time, **kwargs: _mock_tf_transform(0.15, 0.25, 0.35)

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

    # Active goal pointing to [0.20, 0.25, 0.35]
    # delta = goal - ee = [0.05, 0.0, 0.0]
    node._active_goal_handle = MockGoalHandle()
    node._active_task = "Move black pawn to target"
    node._active_goal_start_time = 100.0
    node._active_op = ExecuteChessMove.Goal.OP_MOVE
    node._active_subtask_phase = 0
    node._active_goal_xyz = np.array([0.20, 0.25, 0.35], dtype=np.float32)
    node._active_p_pick = np.array([0.20, 0.25, 0.35], dtype=np.float32)
    node._active_p_place = np.array([0.30, 0.35, 0.40], dtype=np.float32)

    # Run control tick
    node._control_timer_callback()

    assert len(captured_obs) == 1
    raw = captured_obs[0]
    assert raw["task"] == "Move black pawn to target"
    assert raw["observation.state"].shape == (12,)
    # joints(6) + ee(3) + delta(3)
    expected_12d = np.array(
        [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.15, 0.25, 0.35, 0.05, 0.0, 0.0],
        dtype=np.float32,
    )
    np.testing.assert_allclose(raw["observation.state"], expected_12d, atol=1e-5)

    node.destroy_node()


def test_smolvla_observation_delta_zero_at_target():
    """Verify that when TCP reaches the target coordinate, delta is exactly [0, 0, 0]."""
    node = AsyncRos2ClientNode(
        parameter_overrides=[
            Parameter("auto_start", value=False),
        ]
    )

    js = JointState()
    js.name = node.arm_joints
    js.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.30]  # Gripper open approaching pick target
    node._joint_state_callback(js)

    # TCP at [0.25, -0.10, 0.05]
    target = np.array([0.25, -0.10, 0.05], dtype=np.float32)
    node.tf_buffer.lookup_transform = lambda base, tool, time, **kwargs: _mock_tf_transform(*target)

    captured_obs = []
    node.client.ready_to_send = lambda state: True
    node._is_sensor_data_fresh = lambda now: True
    node.client.submit_observation = lambda obs, timestamp=None: captured_obs.append(obs)
    node._latest_wrist_bytes = b"w"
    node._latest_side_bytes = b"s"
    node._latest_overhead_bytes = b"o"

    class MockGoalHandle:
        def __init__(self):
            self.is_active = True
        def publish_feedback(self, fb):
            pass

    node._active_goal_handle = MockGoalHandle()
    node._active_task = "Precision alignment"
    node._active_goal_start_time = 100.0
    node._active_op = ExecuteChessMove.Goal.OP_MOVE
    node._active_subtask_phase = 0
    node._active_goal_xyz = target.copy()
    node._active_p_pick = target.copy()
    node._active_p_place = np.array([0.3, 0.3, 0.1], dtype=np.float32)

    node._control_timer_callback()

    assert len(captured_obs) == 1
    state = captured_obs[0]["observation.state"]
    # ee_xyz is at indices 6:9, delta is at 9:12
    np.testing.assert_allclose(state[6:9], target, atol=1e-5)
    np.testing.assert_allclose(state[9:12], np.zeros(3, dtype=np.float32), atol=1e-5)

    node.destroy_node()


def test_subtask_goal_transition_dual_condition():
    """Verify dual condition for phase transition: gripper closure/opening AND Euclidean delta distance."""
    node = AsyncRos2ClientNode(
        parameter_overrides=[
            Parameter("auto_start", value=False),
            Parameter("delta_reach_threshold_m", value=0.035),
        ]
    )

    class MockGoalHandle:
        def __init__(self):
            self.is_active = True
        def publish_feedback(self, fb):
            pass

    node._active_goal_handle = MockGoalHandle()
    node._active_task = "Move pawn"
    node._active_goal_start_time = 100.0
    node._active_op = ExecuteChessMove.Goal.OP_MOVE
    node._active_subtask_phase = 0

    p_pick = np.array([0.20, 0.0, 0.05], dtype=np.float32)
    p_place = np.array([0.20, 0.15, 0.05], dtype=np.float32)
    node._active_goal_xyz = p_pick.copy()
    node._active_p_pick = p_pick.copy()
    node._active_p_place = p_place.copy()

    # Case 1: Gripper is closed (grip=0.08 < 0.12) BUT arm is far away (dist=0.10 > 0.035)
    # MUST NOT transition to phase 1!
    node.tf_buffer.lookup_transform = lambda b, t, tm, **kw: _mock_tf_transform(0.10, 0.0, 0.05)
    js = JointState()
    js.name = node.arm_joints
    js.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.08]
    node._joint_state_callback(js)
    node._control_timer_callback()

    assert node._active_subtask_phase == 0
    np.testing.assert_allclose(node._active_goal_xyz, p_pick)

    # Case 2: Arm is at pick point (dist=0.005 < 0.035) BUT gripper is still open (grip=0.30 > 0.12)
    # MUST NOT transition to phase 1!
    node.tf_buffer.lookup_transform = lambda b, t, tm, **kw: _mock_tf_transform(0.198, 0.0, 0.05)
    js.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.30]
    node._joint_state_callback(js)
    node._control_timer_callback()

    assert node._active_subtask_phase == 0
    np.testing.assert_allclose(node._active_goal_xyz, p_pick)

    # Case 3: Arm is at pick point (dist < 0.035) AND gripper is closed (grip=0.08 < 0.12)
    # Requires dwell time persistence (10 ticks) to confirm transition!
    js.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.08]
    node._joint_state_callback(js)
    # 1 tick is not enough due to dwell filtering
    node._control_timer_callback()
    assert node._active_subtask_phase == 0

    # Hold condition for required dwell ticks
    for _ in range(node._required_dwell_ticks - 1):
        node._control_timer_callback()

    assert node._active_subtask_phase == 1
    np.testing.assert_allclose(node._active_goal_xyz, p_place)

    # Case 4: In phase 1, arm reaches place point (dist < 0.035) AND gripper opens (grip=0.25 > 0.22)
    # Requires dwell time persistence to transition to phase 2 (landmark_stow)
    node.tf_buffer.lookup_transform = lambda b, t, tm, **kw: _mock_tf_transform(0.20, 0.149, 0.05)
    js.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.25]
    node._joint_state_callback(js)
    for _ in range(node._required_dwell_ticks):
        node._control_timer_callback()

    assert node._active_subtask_phase == 2
    np.testing.assert_allclose(node._active_goal_xyz, node.landmark_stow)

    node.destroy_node()


def test_smolvla_rejection_of_empty_instruction():
    """Verify that goals with empty or whitespace-only instruction are rejected immediately."""
    node = AsyncRos2ClientNode(
        parameter_overrides=[
            Parameter("auto_start", value=False),
        ]
    )
    node._is_active = True
    node.transport._connected = True

    # Valid goal with instruction
    valid_goal = ExecuteChessMove.Goal()
    valid_goal.operation = ExecuteChessMove.Goal.OP_MOVE
    valid_goal.instruction = "Move white pawn e2e4"
    assert node._handle_goal_received(valid_goal) == GoalResponse.ACCEPT

    # Empty instruction
    empty_goal = ExecuteChessMove.Goal()
    empty_goal.operation = ExecuteChessMove.Goal.OP_MOVE
    empty_goal.instruction = ""
    assert node._handle_goal_received(empty_goal) == GoalResponse.REJECT

    # Whitespace instruction
    ws_goal = ExecuteChessMove.Goal()
    ws_goal.operation = ExecuteChessMove.Goal.OP_MOVE
    ws_goal.instruction = "   \t\n "
    assert node._handle_goal_received(ws_goal) == GoalResponse.REJECT

    # Unknown operation enum
    unknown_op_goal = ExecuteChessMove.Goal()
    unknown_op_goal.operation = 99
    unknown_op_goal.instruction = "Move white pawn"
    assert node._handle_goal_received(unknown_op_goal) == GoalResponse.REJECT

    node.destroy_node()
