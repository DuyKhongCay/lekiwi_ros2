# Copyright 2026 LeKiwi Robotics Team
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

"""Integration tests validating AsyncRos2ClientNode parameter initialization, interfaces, and action handling."""

import time

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticStatusWrapper
from geometry_msgs.msg import Point, TransformStamped
from lekiwi_interfaces.action import ExecuteChessMove
import numpy as np
import pytest
import rclpy
from rclpy.action import GoalResponse
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool, Trigger

from lekiwi_inference.async_ros2_client_node import AsyncRos2ClientNode


@pytest.fixture(autouse=True)
def rclpy_init_shutdown():
    """Manages rclpy lifecycle per test session."""
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


def test_goal_validation():
    """Validates goal acceptance/rejection based on readiness, instruction, and operation enum."""
    node = AsyncRos2ClientNode(
        parameter_overrides=[
            Parameter("auto_start", value=False),
            Parameter("enable_actuation_smoothing", value=False),
        ]
    )

    req = ExecuteChessMove.Goal()
    req.operation = ExecuteChessMove.Goal.OP_MOVE
    req.instruction = "Move white pawn e2e4"

    # 1. Zero backward-compatible: Reject goal if client is inactive/disconnected
    assert node._handle_goal_received(req) == GoalResponse.REJECT

    # Simulate client active & transport connected
    node._is_active = True
    node.transport._connected = True

    # 2. Valid goals across all supported operations
    operations = [
        ExecuteChessMove.Goal.OP_MOVE,
        ExecuteChessMove.Goal.OP_CLEAR_LEFT,
        ExecuteChessMove.Goal.OP_CLEAR_RIGHT,
        ExecuteChessMove.Goal.OP_PICK,
        ExecuteChessMove.Goal.OP_PLACE,
    ]
    for op in operations:
        req_valid = ExecuteChessMove.Goal()
        req_valid.operation = op
        req_valid.instruction = f"Execute operation {op}"
        assert node._handle_goal_received(req_valid) == GoalResponse.ACCEPT

    # 3. Invalid operation enum
    invalid_req = ExecuteChessMove.Goal()
    invalid_req.operation = 99
    invalid_req.instruction = "Invalid operation"
    assert node._handle_goal_received(invalid_req) == GoalResponse.REJECT

    # 4. Empty or whitespace instruction rejected
    empty_req = ExecuteChessMove.Goal()
    empty_req.operation = ExecuteChessMove.Goal.OP_MOVE
    empty_req.instruction = "   "
    assert node._handle_goal_received(empty_req) == GoalResponse.REJECT

    # 5. Verify 12D schema declared for observation.state and no environment_state feature
    assert node.client.config.features["observation.state"]["shape"] == (12,)
    assert node.client.config.features["observation.state"]["dtype"] == "float32"
    assert "observation.environment_state" not in node.client.config.features

    node.destroy_node()


def test_node_lifecycle_and_interfaces():
    """Validates node initialization, idle silence, forward command output, and service registration."""
    node = AsyncRos2ClientNode(
        parameter_overrides=[
            Parameter("auto_start", value=False),
            Parameter("enable_actuation_smoothing", value=False),
        ]
    )

    received_cmds = []

    def cmd_cb(msg: Float64MultiArray):
        received_cmds.append(list(msg.data))

    test_sub = node.create_subscription(
        Float64MultiArray,
        node.fwd_cmd_topic,
        cmd_cb,
        1,
    )

    # Wait for pub/sub discovery
    start_discovery = time.time()
    while node.cmd_pub.get_subscription_count() == 0 and (time.time() - start_discovery) < 1.0:
        rclpy.spin_once(node, timeout_sec=0.05)

    # Publish mock joint state
    js_msg = JointState()
    js_msg.name = node.arm_joints
    js_msg.position = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    node._joint_state_callback(js_msg)

    # Step A: Idle Silence check
    # When no goal is active, control loop MUST NOT publish commands
    node._control_timer_callback()
    start_spin = time.time()
    while (time.time() - start_spin) < 0.2:
        rclpy.spin_once(node, timeout_sec=0.05)
    assert len(received_cmds) == 0, "Node must be silent and NOT publish commands when idle"

    # Step B: Active goal execution -> outputs candidate action
    class MockGoalHandle:
        def __init__(self):
            self.is_active = True
        def publish_feedback(self, fb):
            pass

    node.tf_buffer.lookup_transform = lambda b, t, tm, **kw: _mock_tf_transform(0.1, 0.2, 0.3)
    node._active_goal_handle = MockGoalHandle()
    node._active_task = "Active task"
    node._active_goal_start_time = time.time()
    node._active_op = ExecuteChessMove.Goal.OP_MOVE
    node._active_subtask_phase = 0
    node._active_goal_xyz = np.array([0.2, 0.2, 0.2], dtype=np.float32)
    node._active_p_pick = np.array([0.2, 0.2, 0.2], dtype=np.float32)
    node._active_p_place = np.array([0.3, 0.3, 0.3], dtype=np.float32)

    target_action = np.array([0.15, 0.25, 0.35, 0.45, 0.55, 0.65], dtype=np.float32)
    node.client.actions_available = lambda: True
    node.client.pop_action = lambda: target_action

    node._control_timer_callback()

    start_spin = time.time()
    while not received_cmds and (time.time() - start_spin) < 1.0:
        rclpy.spin_once(node, timeout_sec=0.05)

    assert len(received_cmds) >= 1
    np.testing.assert_allclose(received_cmds[0], target_action, atol=1e-4)

    # Test enable service handler
    req = SetBool.Request()
    req.data = False
    resp = SetBool.Response()
    node._handle_enable_service(req, resp)
    assert resp.success is True

    # Test estop service handler
    estop_req = Trigger.Request()
    estop_resp = Trigger.Response()
    node._handle_estop_service(estop_req, estop_resp)
    assert estop_resp.success is True
    assert node._estop_active is True

    node.destroy_node()


def test_diagnostic_updater_integration_and_zero_backward_compat():
    """Validates diagnostic_updater registration, zero backward compatibility, and telemetry generation."""
    node = AsyncRos2ClientNode(
        parameter_overrides=[
            Parameter("auto_start", value=False),
            Parameter("enable_actuation_smoothing", value=False),
        ]
    )

    # 1. Zero backward compatibility: ensure old custom topic/publisher/timer are gone
    assert not hasattr(node, "diag_pub"), "Legacy diag_pub publisher must be completely removed"
    assert not hasattr(node, "diag_timer"), "Legacy diag_timer must be completely removed"
    assert not hasattr(node, "_diagnostic_timer_callback"), "Legacy _diagnostic_timer_callback must be removed"

    publishers = node.get_publisher_names_and_types_by_node(node.get_name(), node.get_namespace())
    topic_names = [name for name, _ in publishers]
    assert "/lekiwi/inference/status" not in topic_names

    # 2. Verify updater exists and is attached
    assert hasattr(node, "updater")

    # 3. Test _produce_diagnostics callback under disconnected state
    stat = DiagnosticStatusWrapper()
    result = node._produce_diagnostics(stat)
    assert result.level == DiagnosticStatus.ERROR
    assert "Disconnected" in result.message

    val_dict = {kv.key: kv.value for kv in result.values}
    assert "transport_connected" in val_dict
    assert "inference_active" in val_dict
    assert "estop_active" in val_dict
    assert "queue_size" in val_dict
    assert "obs_filtered_count" in val_dict
    assert "rtt_latency_ms" in val_dict
    assert val_dict["obs_filtered_count"] == "0"
    assert val_dict["transport_connected"] == "False"
    assert val_dict["estop_active"] == "False"
    assert val_dict["active_goal"] == "False"

    # Verify zero backward-compatible default parameters
    assert node.chunk_size_threshold == 0.35
    assert node.observation_filter_epsilon == 0.05
    assert node.client.config.chunk_size_threshold == 0.35
    assert node.client.config.observation_filter_epsilon == 0.05

    # 4. Test _produce_diagnostics when E-Stop is active
    node._estop_active = True
    stat_estop = DiagnosticStatusWrapper()
    result_estop = node._produce_diagnostics(stat_estop)
    assert result_estop.level == DiagnosticStatus.WARN
    assert "Emergency Stop" in result_estop.message
    val_dict_estop = {kv.key: kv.value for kv in result_estop.values}
    assert val_dict_estop["estop_active"] == "True"

    # 5. Test _produce_diagnostics when connected
    node._estop_active = False
    node.transport._connected = True
    node.transport.ping = lambda: 5.2
    stat_conn = DiagnosticStatusWrapper()
    result_conn = node._produce_diagnostics(stat_conn)
    assert result_conn.level == DiagnosticStatus.OK
    assert "Idle" in result_conn.message

    node._is_active = True
    stat_active = DiagnosticStatusWrapper()
    result_active = node._produce_diagnostics(stat_active)
    assert result_active.level == DiagnosticStatus.OK
    assert "Streaming Active" in result_active.message

    node.destroy_node()


def test_stream_starvation_watchdog_and_strict_convergence():
    """Validates stream starvation, non-stow rejection, stow convergence, and timeout watchdog."""
    stow_val = [1.56, -1.74, 1.55, 0.77, -2.78, 0.08]
    node = AsyncRos2ClientNode(
        parameter_overrides=[
            Parameter("auto_start", value=False),
            Parameter("stream_timeout_s", value=0.2),
            Parameter("convergence.starvation_timeout_s", value=0.2),
            Parameter("named_poses.stow", value=stow_val),
            Parameter("convergence.min_actions", value=5),
            Parameter("convergence.stow_tolerance_rad", value=0.20),
            Parameter("convergence.settle_steps", value=3),
            Parameter("convergence.max_duration_s", value=10.0),
        ]
    )

    node._is_active = True
    node.transport._connected = True
    node.tf_buffer.lookup_transform = lambda b, t, tm, **kw: _mock_tf_transform(0.1, 0.2, 0.3)

    # Initial joint state (reaching away from stow)
    js = JointState()
    js.name = node.arm_joints
    js.position = [0.17, 0.50, 0.14, 0.67, -1.52, 0.42]  # Mid-reach pose
    node._joint_state_callback(js)

    # Mock an active goal
    class MockGoalHandle:
        def __init__(self):
            self.is_active = True
            self.feedbacks = []
        def publish_feedback(self, fb):
            self.feedbacks.append(fb)

    mock_handle = MockGoalHandle()
    node._active_goal_handle = mock_handle
    node._active_task = "Test Task"
    node._active_goal_start_time = time.time()
    node._active_op = ExecuteChessMove.Goal.OP_MOVE
    node._active_subtask_phase = 0
    node._active_goal_xyz = np.array([0.2, 0.0, 0.1], dtype=np.float32)
    node._active_p_pick = np.array([0.2, 0.0, 0.1], dtype=np.float32)
    node._active_p_place = np.array([0.2, 0.2, 0.1], dtype=np.float32)
    node._active_policy_actions_executed = 0  # No actions executed!

    # Step 1: Control tick with no actions available
    node._control_timer_callback()
    # False convergence MUST NOT happen because no actions were executed
    assert node._active_settle_count == 0
    assert not node._goal_finished_event.is_set()

    # Step 2: Advance time past stream_timeout_s (0.2s)
    node._active_starvation_start_time = time.time() - 0.3
    node._control_timer_callback()

    # Starvation watchdog MUST trigger and abort goal
    assert node._goal_finished_event.is_set()
    assert node._goal_success is False
    assert "stream starved" in node._goal_result_message

    # Step 3: Mid-reach rejection - sufficient actions executed, but arm NOT at stow
    node._goal_finished_event.clear()
    node._goal_success = False
    node._active_policy_actions_executed = 10  # sufficient actions
    node._active_starvation_start_time = None
    node.client.actions_available = lambda: True
    node.client.pop_action = lambda: np.array([0.17, 0.50, 0.14, 0.67, -1.52, 0.42])

    for _ in range(5):
        node._control_timer_callback()

    # MUST NOT converge because arm is at mid-reach pose, far from stow
    assert node._active_settle_count == 0
    assert not node._goal_finished_event.is_set()

    # Step 4: True convergence - arm folds back into stow pose
    js_stow = JointState()
    js_stow.name = node.arm_joints
    js_stow.position = [1.56, -1.74, 1.55, 0.77, -2.78, 0.08]
    node._joint_state_callback(js_stow)
    node.client.pop_action = lambda: np.array([1.56, -1.74, 1.55, 0.77, -2.78, 0.08])

    for _ in range(node.settle_steps):
        node._control_timer_callback()

    assert node._goal_finished_event.is_set()
    assert node._goal_success is True
    assert "converged to stow posture" in node._goal_result_message

    # Step 5: Test episode timeout watchdog
    node._goal_finished_event.clear()
    node._goal_success = False
    node._active_goal_start_time = time.time() - 15.0  # exceeded max_duration_s (10.0s)
    is_term, msg = node._evaluate_convergence(np.zeros(6), 15.0, 10)
    assert is_term is True
    assert "timed out" in msg

    node.destroy_node()
