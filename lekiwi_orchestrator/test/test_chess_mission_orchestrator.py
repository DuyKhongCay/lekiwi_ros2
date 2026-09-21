# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for ChessMissionOrchestrator logic and state machine."""

import pytest
import rclpy
from std_msgs.msg import Bool
from lekiwi_interfaces.msg import CameraMode, ChessGameStatus
from lekiwi_interfaces.srv import CheckMoveFeasibility

from lekiwi_orchestrator.chessboard_coordinate_mapper import ChessboardCoordinateMapper
from lekiwi_orchestrator.chess_mission_orchestrator import ChessMissionOrchestrator
from lekiwi_orchestrator.fsm import MissionState


@pytest.fixture(scope="module")
def ros_context():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_orchestrator_initial_state(ros_context):
    mapper = ChessboardCoordinateMapper(board_width=0.20, board_height=0.20)
    node = ChessMissionOrchestrator(coordinate_mapper=mapper)
    try:
        assert node.mission_state == MissionState.WAITING_FOR_TF_READY
        assert node.camera_mode == CameraMode.STANDBY
        assert not node.is_tf_ready
    finally:
        node.destroy_node()


def test_tf_readiness_transition(ros_context):
    mapper = ChessboardCoordinateMapper(board_width=0.20, board_height=0.20)
    node = ChessMissionOrchestrator(coordinate_mapper=mapper)
    try:
        # Emit TF ready
        tf_msg = Bool()
        tf_msg.data = True
        node._on_tf_ready(tf_msg)

        assert node.is_tf_ready
        # Default robot_color="b" waits for White player move
        assert node.mission_state == MissionState.WAITING_FOR_PLAYER_MOVE
        assert node.camera_mode == CameraMode.CHESS_THINKING
    finally:
        node.destroy_node()


def test_tf_readiness_white_robot_starts_first(ros_context):
    mapper = ChessboardCoordinateMapper(board_width=0.20, board_height=0.20)
    node = ChessMissionOrchestrator(coordinate_mapper=mapper)
    try:
        node._robot_color = "w"
        tf_msg = Bool()
        tf_msg.data = True
        node._on_tf_ready(tf_msg)

        assert node.is_tf_ready
        # If White, robot must think and evaluate first move
        assert node.mission_state == MissionState.EVALUATING_BEST_MOVE
        assert node.camera_mode == CameraMode.CHESS_THINKING
    finally:
        node.destroy_node()


def test_game_over_on_checkmate(ros_context):
    mapper = ChessboardCoordinateMapper(board_width=0.20, board_height=0.20)
    node = ChessMissionOrchestrator(coordinate_mapper=mapper)
    try:
        # Mark TF ready
        node._on_tf_ready(Bool(data=True))

        # Emit checkmate game status
        status_msg = ChessGameStatus()
        status_msg.is_checkmate = True
        status_msg.full_fen = (
            "rnb1kbnr/pppp1ppp/8/4p3/6Pq/5P2/PPPPP2P/RNBQKBNR w KQkq - 1 3"
        )
        node._on_game_status(status_msg)

        assert node.mission_state == MissionState.GAME_OVER
        assert node.camera_mode == CameraMode.STANDBY
    finally:
        node.destroy_node()


def test_game_over_on_draw(ros_context):
    mapper = ChessboardCoordinateMapper(board_width=0.20, board_height=0.20)
    node = ChessMissionOrchestrator(coordinate_mapper=mapper)
    try:
        node._on_tf_ready(Bool(data=True))

        status_msg = ChessGameStatus()
        status_msg.is_draw = True
        node._on_game_status(status_msg)

        assert node.mission_state == MissionState.GAME_OVER
        assert node.camera_mode == CameraMode.STANDBY
    finally:
        node.destroy_node()


def test_illegal_state_transition_guard(ros_context):
    mapper = ChessboardCoordinateMapper(board_width=0.20, board_height=0.20)
    node = ChessMissionOrchestrator(coordinate_mapper=mapper)
    try:
        # Currently in WAITING_FOR_TF_READY
        assert node.mission_state == MissionState.WAITING_FOR_TF_READY

        # Attempt illegal jump to EXECUTING_MANIPULATION
        success = node.transition_to(MissionState.EXECUTING_MANIPULATION)
        assert not success
        assert node.mission_state == MissionState.WAITING_FOR_TF_READY  # Unchanged
    finally:
        node.destroy_node()


def test_workflow_dispatch_zero_nav_simulation(ros_context):
    mapper = ChessboardCoordinateMapper(board_width=0.20, board_height=0.20)
    node = ChessMissionOrchestrator(coordinate_mapper=mapper)
    try:
        node._on_tf_ready(Bool(data=True))
        assert node.mission_state == MissionState.WAITING_FOR_PLAYER_MOVE

        # Mock feasibility response directly
        resp = CheckMoveFeasibility.Response()
        resp.feasible = True
        resp.plan_type = CheckMoveFeasibility.Response.PLAN_ZERO_NAV
        resp.message = "PLAN_ZERO_NAV verified"

        details = mapper.parse_uci_move("e7e5")

        class _MockFuture:
            def result(self):
                return resp

        # Simulate feasibility response callback
        node.transition_to(MissionState.CHECKING_REACHABILITY)
        node._on_feasibility_response(_MockFuture(), details)

        # Since manipulation client is None in test, it simulates completion
        # and cycles back to WAITING_FOR_PLAYER_MOVE
        assert node.mission_state == MissionState.WAITING_FOR_PLAYER_MOVE
        assert node.camera_mode == CameraMode.CHESS_THINKING
    finally:
        node.destroy_node()


def test_readiness_lease_expires_without_ros_time(ros_context):
    """A stopped gatekeeper cannot leave cached readiness valid indefinitely."""
    import time
    node = ChessMissionOrchestrator()
    try:
        node._on_tf_ready(Bool(data=True))
        node._last_readiness_heartbeat = time.monotonic() - node._readiness_timeout - 0.1
        assert not node.is_tf_ready
        node._expire_readiness()
        assert not node._tf_ready
    finally:
        node.destroy_node()
