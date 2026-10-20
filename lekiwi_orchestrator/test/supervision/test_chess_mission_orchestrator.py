# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for ChessMissionOrchestrator logic, 2-level HFSM, PerceptionContext, and self-healing."""

from __future__ import annotations

import pytest
import rclpy
from geometry_msgs.msg import Point
from lekiwi_interfaces.msg import (
    ChessGameStatus,
    PerceptionContext,
)
from lekiwi_interfaces.srv import CheckMoveFeasibility
from lekiwi_orchestrator.fsm import (
    MacroMissionState,
    MotionExecutionState,
)
from lekiwi_orchestrator.mission_types import (
    ActionResult,
    ChessMoveGoal,
)
from lekiwi_orchestrator.motion_client import (
    FakeMotionClient,
    MotionClient,
)
from lekiwi_orchestrator.orchestrator_node import (
    ChessMissionOrchestrator,
)
from rclpy.parameter import Parameter
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class TrackingDispatcher(MotionClient):
    """Local test tracking dispatcher for orchestrator tests."""

    def __init__(self):
        self.nav_goals = []
        self.manip_goals = []
        self.cancelled = False

    def check_feasibility(self, goal, timeout_sec=5.0, on_success=None, on_error=None):
        return True

    def send_navigation_goal(self, target_pose, timeout_sec=60.0, on_completed=None):
        self.nav_goals.append(target_pose)
        if on_completed:
            on_completed(ActionResult(success=True, message="OK", execution_time_sec=0.1))
        return True

    def send_manipulation_goal(self, goal, timeout_sec=60.0, on_feedback=None, on_completed=None):
        self.manip_goals.append(goal)
        if on_completed:
            on_completed(ActionResult(success=True, message="OK", execution_time_sec=0.1))
        return True

    def cancel_active_goal(self):
        self.cancelled = True

    def destroy(self):
        pass


@pytest.fixture(scope="module")
def ros_context():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_orchestrator_initial_state(ros_context):
    """Verify initial state of orchestrator node before external signals."""
    node = ChessMissionOrchestrator()
    try:
        assert node.mission_state == MacroMissionState.WAITING_FOR_TF_READY
        assert node.motion_state == MotionExecutionState.IDLE
        assert node.perception_context == PerceptionContext.TF_TRACKING_AND_NAV
        assert not node.is_nav_ready
    finally:
        node.destroy_node()


@pytest.mark.parametrize(
    ("color", "expected_state"),
    [
        ("b", MacroMissionState.WAITING_FOR_PLAYER_MOVE),
        ("w", MacroMissionState.EVALUATING_BEST_MOVE),
    ],
)
def test_game_start_by_player_color(ros_context, color, expected_state):
    """Verify game startup behavior depending on robot assigned color (White vs Black)."""
    node = ChessMissionOrchestrator(parameter_overrides=[Parameter("robot_color", value=color)])
    try:
        node._on_nav_ready(Bool(data=True))
        assert node.is_nav_ready
        assert node.mission_state == expected_state
        assert node.perception_context == PerceptionContext.BOARD_STATE_SCAN
    finally:
        node.destroy_node()


@pytest.mark.parametrize("condition", ["checkmate", "draw"])
def test_game_over_conditions(ros_context, condition):
    """Verify orchestrator transitions to GAME_OVER on checkmate or draw status."""
    node = ChessMissionOrchestrator()
    try:
        node._on_nav_ready(Bool(data=True))

        status_msg = ChessGameStatus()
        if condition == "checkmate":
            status_msg.is_checkmate = True
        else:
            status_msg.is_draw = True
        node._on_game_status(status_msg)

        assert node.mission_state == MacroMissionState.GAME_OVER
        assert node.motion_state == MotionExecutionState.IDLE
        assert node.perception_context == PerceptionContext.IDLE_STANDBY
    finally:
        node.destroy_node()


def test_nav_readiness_gating_behavior(ros_context):
    """Verify state transitions gated strictly on navigation readiness lease."""
    node = ChessMissionOrchestrator()
    try:
        assert node.mission_state == MacroMissionState.WAITING_FOR_TF_READY
        node._on_nav_ready(Bool(data=True))
        assert node.mission_state == MacroMissionState.WAITING_FOR_PLAYER_MOVE

        # Non-critical state: losing nav ready doesn't crash player wait
        node._on_nav_ready(Bool(data=False))
        assert node.mission_state == MacroMissionState.WAITING_FOR_PLAYER_MOVE

        # Critical state: losing nav ready during reachability triggers ERROR_FALLBACK
        node._on_nav_ready(Bool(data=True))
        node.transition_to(MacroMissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MacroMissionState.CHECKING_REACHABILITY)
        assert node.mission_state == MacroMissionState.CHECKING_REACHABILITY

        node._on_nav_ready(Bool(data=False))
        assert node.mission_state == MacroMissionState.ERROR_FALLBACK

        # Recovery from ERROR_FALLBACK back to WAITING_FOR_TF_READY
        node.trigger_recovery()
        assert node.mission_state == MacroMissionState.WAITING_FOR_TF_READY
    finally:
        node.destroy_node()


def test_feasibility_timeout_watchdog(ros_context):
    """Verify reachability query error or watchdog timeout triggers ERROR_FALLBACK."""
    node = ChessMissionOrchestrator()
    try:
        node._on_nav_ready(Bool(data=True))
        node.transition_to(MacroMissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MacroMissionState.CHECKING_REACHABILITY)
        assert node.mission_state == MacroMissionState.CHECKING_REACHABILITY

        node._on_feasibility_error("Workspace feasibility query timed out")
        assert node.mission_state == MacroMissionState.ERROR_FALLBACK
        assert node.motion_state == MotionExecutionState.IDLE
    finally:
        node.destroy_node()


def test_action_timeout_watchdog_cancels_goal(ros_context):
    """Verify action failure triggers ERROR_FALLBACK and recovery cancels active motion goals."""
    dispatcher = TrackingDispatcher()
    node = ChessMissionOrchestrator(motion_client=dispatcher)
    try:
        node._on_nav_ready(Bool(data=True))
        node.transition_to(MacroMissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MacroMissionState.CHECKING_REACHABILITY)
        node.transition_to(MacroMissionState.EXECUTING_MOVE_PIPELINE)
        assert node.mission_state == MacroMissionState.EXECUTING_MOVE_PIPELINE

        node._on_move_pipeline_failed("Action execution timed out")
        assert node.mission_state == MacroMissionState.ERROR_FALLBACK
        assert node.motion_state == MotionExecutionState.IDLE

        node.trigger_recovery()
        assert dispatcher.cancelled
        assert node.mission_state == MacroMissionState.WAITING_FOR_TF_READY
    finally:
        node.destroy_node()


def test_self_healing_recover_service(ros_context):
    """Verify operator service /orchestrator/recover restores system from ERROR_FALLBACK."""
    node = ChessMissionOrchestrator()
    try:
        node._on_nav_ready(Bool(data=True))
        node.transition_to(MacroMissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MacroMissionState.CHECKING_REACHABILITY)
        node.transition_to(MacroMissionState.ERROR_FALLBACK)
        assert node.mission_state == MacroMissionState.ERROR_FALLBACK

        req = Trigger.Request()
        resp = Trigger.Response()
        result_resp = node._handle_recover_service(req, resp)

        assert result_resp.success
        assert node.mission_state == MacroMissionState.WAITING_FOR_TF_READY
        assert node.motion_state == MotionExecutionState.IDLE
        assert node.perception_context == PerceptionContext.TF_TRACKING_AND_NAV
    finally:
        node.destroy_node()


@pytest.mark.parametrize(
    ("plan_type", "is_capture"),
    [
        (CheckMoveFeasibility.Response.PLAN_ZERO_NAV, False),
        (CheckMoveFeasibility.Response.PLAN_CAPTURE_SINGLE_BASE, True),
        (CheckMoveFeasibility.Response.PLAN_CAPTURE_TRIPLE_BASE, True),
    ],
)
def test_workflow_dispatch_sequences(ros_context, plan_type, is_capture):
    """Verify move workflow dispatching across standard and complex capture sequences."""
    dispatcher = TrackingDispatcher()
    node = ChessMissionOrchestrator(
        parameter_overrides=[Parameter("navigation", value=(plan_type != CheckMoveFeasibility.Response.PLAN_ZERO_NAV))],
        motion_client=dispatcher,
    )
    try:
        node._on_nav_ready(Bool(data=True))
        node._on_grasp_ready(Bool(data=True))
        node.transition_to(MacroMissionState.WAITING_FOR_PLAYER_MOVE)

        resp = CheckMoveFeasibility.Response()
        resp.feasible = True
        resp.plan_type = plan_type
        resp.clear_point = Point(x=0.5, y=0.5, z=0.03)

        goal = ChessMoveGoal(
            uci="e7e5" if not is_capture else "e4d5",
            from_square="e7" if not is_capture else "e4",
            to_square="e5" if not is_capture else "d5",
            is_capture=is_capture,
        )

        node.transition_to(MacroMissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MacroMissionState.CHECKING_REACHABILITY)
        node._on_feasibility_response(resp, goal)

        assert node.mission_state == MacroMissionState.POST_MOVE_VERIFYING
        assert node.motion_state == MotionExecutionState.IDLE
    finally:
        node.destroy_node()


def test_recovery_retains_and_resumes_interrupted_move(ros_context):
    """Verify recovery retains interrupted move and resumes execution if board FEN unchanged."""
    dispatcher = TrackingDispatcher()
    node = ChessMissionOrchestrator(motion_client=dispatcher)
    try:
        node._on_nav_ready(Bool(data=True))
        node._on_grasp_ready(Bool(data=True))
        assert node.mission_state == MacroMissionState.WAITING_FOR_PLAYER_MOVE

        node._dispatch_move_workflow("c7c5")
        assert node.current_move_details.uci == "c7c5"
        assert node.mission_state == MacroMissionState.CHECKING_REACHABILITY

        node._on_feasibility_error("Reachability query timeout")
        assert node.mission_state == MacroMissionState.ERROR_FALLBACK

        recovered = node.trigger_recovery(reason="test_recovery")
        assert recovered
        assert node.mission_state == MacroMissionState.WAITING_FOR_TF_READY
        assert node.pending_recovery_move is not None
        assert node.pending_recovery_move.uci == "c7c5"

        # On next nav ready tick, robot resumes interrupted move
        node._on_nav_ready(Bool(data=True))
        assert node.mission_state == MacroMissionState.CHECKING_REACHABILITY
        assert node.current_move_details.uci == "c7c5"
        assert node.pending_recovery_move is None
    finally:
        node.destroy_node()


def test_recovery_discards_stale_move_if_fen_changes(ros_context):
    """Verify pending recovery move is discarded if board FEN changes before resumption."""
    node = ChessMissionOrchestrator()
    try:
        node._on_nav_ready(Bool(data=True))
        node.transition_to(MacroMissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MacroMissionState.CHECKING_REACHABILITY)
        node.transition_to(MacroMissionState.ERROR_FALLBACK)

        goal = ChessMoveGoal(uci="e7e5", from_square="e7", to_square="e5", is_capture=False)
        node._current_move_details = goal
        node.trigger_recovery(reason="test")
        assert node.pending_recovery_move.uci == "e7e5"

        # New game status arrives with different FEN (board modified externally)
        status = ChessGameStatus()
        status.active_color = "b"
        status.full_fen = "rnbqkbnr/pppp1ppp/8/4p3/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2"
        status.is_board_stable = True
        status.game_phase = ChessGameStatus.PHASE_ROBOT_READY
        status.best_move_details.uci = "d7d5"
        node._on_game_status(status)

        # Stale recovery move e7e5 must be discarded
        assert node.pending_recovery_move is None
    finally:
        node.destroy_node()


def test_post_move_verification_watchdog_timeout_repositions(ros_context):
    """Verify that watchdog timeout during POST_MOVE_VERIFYING triggers active repositioning."""
    repositioned_poses = []

    class RepositionTrackingDispatcher(FakeMotionClient):
        def __init__(self):
            super().__init__(None)

        def send_navigation_goal(self, target_pose, timeout_sec=60.0, on_completed=None):
            repositioned_poses.append(target_pose)
            if on_completed:
                on_completed(ActionResult(success=True, message="Reached viewpoint"))
            return True

    dispatcher = RepositionTrackingDispatcher()
    node = ChessMissionOrchestrator(motion_client=dispatcher)
    try:
        node._on_nav_ready(Bool(data=True))
        node.transition_to(MacroMissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MacroMissionState.CHECKING_REACHABILITY)
        node.transition_to(MacroMissionState.EXECUTING_MOVE_PIPELINE)
        node._on_move_pipeline_completed()
        assert node.mission_state == MacroMissionState.POST_MOVE_VERIFYING
        assert node.post_move_verifier.is_active

        # Watchdog timeout fires -> repositions to next vantage point
        node._on_post_move_watchdog_timeout()
        assert len(repositioned_poses) == 1
        assert node.obs_navigator.viewpoint_index == 1
        assert node.post_move_verifier.attempts == 1

        # Board stabilizes
        status = ChessGameStatus()
        status.is_board_stable = True
        status.is_legal_move = True
        status.full_fen = "rnbqkbnr/pppp1ppp/8/4p3/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2"
        node._on_game_status(status)

        assert node.mission_state == MacroMissionState.WAITING_FOR_PLAYER_MOVE
        assert not node.post_move_verifier.is_active
    finally:
        node.destroy_node()


def test_post_move_verification_ignores_illegal_vision(ros_context):
    """Verify that is_board_stable=True with is_legal_move=False does NOT finalize turn."""
    node = ChessMissionOrchestrator()
    try:
        node._on_nav_ready(Bool(data=True))
        node.transition_to(MacroMissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MacroMissionState.CHECKING_REACHABILITY)
        node.transition_to(MacroMissionState.EXECUTING_MOVE_PIPELINE)
        node._on_move_pipeline_completed()
        assert node.mission_state == MacroMissionState.POST_MOVE_VERIFYING
        assert node.post_move_verifier.is_active

        # Occluded vision: stable board but move is illegal
        status = ChessGameStatus()
        status.is_board_stable = True
        status.is_legal_move = False
        node._on_game_status(status)

        assert node.mission_state == MacroMissionState.POST_MOVE_VERIFYING
        assert node.post_move_verifier.is_active
    finally:
        node.destroy_node()


def test_execute_chess_move_invalid_uci_fallback(ros_context):
    """Verify invalid UCI string transitions orchestrator to ERROR_FALLBACK."""
    node = ChessMissionOrchestrator()
    try:
        node._on_nav_ready(Bool(data=True))
        node._on_grasp_ready(Bool(data=True))
        node.transition_to(MacroMissionState.WAITING_FOR_PLAYER_MOVE)
        node._dispatch_move_workflow("xyz")
        assert node.mission_state == MacroMissionState.ERROR_FALLBACK
    finally:
        node.destroy_node()
