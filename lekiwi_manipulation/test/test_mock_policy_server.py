# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for MockPolicyServer action execution and feedback lifecycle."""

import asyncio
import pytest
import rclpy
from geometry_msgs.msg import Point
from lekiwi_interfaces.action import ExecuteChessMove
from rclpy.action import CancelResponse, GoalResponse
from sensor_msgs.msg import JointState

from lekiwi_manipulation.mock_policy_server import MockPolicyServer
from lekiwi_manipulation.trajectory_generator import (
    ARM_JOINTS_DEFAULT,
    ManipulationPhase,
)


@pytest.fixture(scope="module")
def ros_context():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_mock_server_initialization(ros_context):
    node = MockPolicyServer(action_name="/test_execute_chess_move")
    try:
        assert node.current_phase == ManipulationPhase.IDLE
        assert not node._is_active
        assert node._phase_duration > 0.0
    finally:
        node.destroy_node()


def test_joint_dict_extraction(ros_context):
    node = MockPolicyServer()
    try:
        # Valid joint state
        js = JointState()
        js.name = ["arm_shoulder_pan", "arm_shoulder_lift"]
        js.position = [0.1, -0.5]
        extracted = node._extract_joint_dict(js)
        assert extracted["arm_shoulder_pan"] == 0.1
        assert extracted["arm_shoulder_lift"] == -0.5

        # Empty joint state falls back to default home pose
        js_empty = JointState()
        fallback = node._extract_joint_dict(js_empty)
        assert "arm_shoulder_pan" in fallback
    finally:
        node.destroy_node()


def test_goal_acceptance_and_busy_rejection(ros_context):
    node = MockPolicyServer()
    try:
        goal_req = ExecuteChessMove.Goal()
        goal_req.instruction = "Pick e2, place e4"
        goal_req.from_square = "e2"
        goal_req.to_square = "e4"

        # Server idle -> ACCEPT
        assert node._handle_goal_request(goal_req) == GoalResponse.ACCEPT

        # Server busy -> REJECT
        node._is_active = True
        assert node._handle_goal_request(goal_req) == GoalResponse.REJECT
    finally:
        node.destroy_node()


class _MockGoalHandle:
    """Mock action goal handle for unit testing _execute_goal."""

    def __init__(self, goal_req, cancel_after_phase=None):
        self.request = goal_req
        self.is_cancel_requested = False
        self.is_succeeded = False
        self.is_canceled = False
        self.feedbacks = []
        self._cancel_after = cancel_after_phase
        self._step = 0

    def publish_feedback(self, fb):
        self.feedbacks.append((fb.current_phase, fb.progress_percent))
        self._step += 1
        if self._cancel_after and self._step >= self._cancel_after:
            self.is_cancel_requested = True

    def succeed(self):
        self.is_succeeded = True

    def canceled(self):
        self.is_canceled = True


def test_goal_execution_flow_fast(ros_context):
    node = MockPolicyServer()
    node._simulate_delay = False  # Instantaneous execution for unit test
    try:
        goal_req = ExecuteChessMove.Goal()
        goal_req.instruction = "Pick e2, place e4"
        goal_req.from_square = "e2"
        goal_req.to_square = "e4"
        goal_req.is_capture = False

        mock_handle = _MockGoalHandle(goal_req)
        result = asyncio.run(node._execute_goal(mock_handle))

        assert result.success is True
        assert mock_handle.is_succeeded is True
        assert not mock_handle.is_canceled
        assert "executed successfully" in result.message

        # Verify all 8 phases emitted feedback
        assert len(mock_handle.feedbacks) == 8
        phases = [p for p, _ in mock_handle.feedbacks]
        assert phases[0] == ManipulationPhase.APPROACH_PICK.value
        assert phases[-1] == ManipulationPhase.RETRACT_STOW.value
    finally:
        node.destroy_node()


def test_goal_preemption_handling(ros_context):
    node = MockPolicyServer()
    node._simulate_delay = False
    try:
        goal_req = ExecuteChessMove.Goal()
        goal_req.instruction = "Pick e2, place e4"
        goal_req.from_square = "e2"
        goal_req.to_square = "e4"

        # Cancel after phase 2
        mock_handle = _MockGoalHandle(goal_req, cancel_after_phase=2)
        result = asyncio.run(node._execute_goal(mock_handle))

        assert result.success is False
        assert mock_handle.is_canceled is True
        assert not mock_handle.is_succeeded
        assert "Preempted and cancelled" in result.message
        assert node.current_phase == ManipulationPhase.IDLE
        assert not node._is_active
    finally:
        node.destroy_node()
