# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for ChessMissionOrchestrator logic, 2-level HFSM, PerceptionContext, and self-healing."""

from __future__ import annotations

import pytest
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from lekiwi_interfaces.msg import (
    ChessGameStatus,
    PerceptionContext,
)
from lekiwi_interfaces.srv import CheckMoveFeasibility, SetFen
from lekiwi_orchestrator.core.fsm import (
    MacroMissionState,
    MotionExecutionState,
)
from lekiwi_orchestrator.core.mission_types import (
    ActionResult,
    ChessMoveGoal,
)
from lekiwi_orchestrator.execution.motion_client import (
    FakeMotionClient,
    MotionClient,
)
from lekiwi_orchestrator.supervision.orchestrator_node import (
    ChessMissionOrchestrator,
)
from rclpy.parameter import Parameter
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


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
    """Verify game startup behavior depending on dynamically broadcast robot color (White vs Black)."""
    node = ChessMissionOrchestrator()
    try:
        node._on_nav_ready(Bool(data=True))
        assert node.is_nav_ready
        assert node.mission_state == MacroMissionState.WAITING_FOR_PLAYER_MOVE
        assert node.perception_context == PerceptionContext.BOARD_STATE_SCAN

        # Incoming first game status message from chess master: White to move
        status = ChessGameStatus()
        status.full_fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
        status.active_color = "w"
        status.robot_color = color
        status.is_board_stable = True
        if color == "w":
            status.game_phase = ChessGameStatus.PHASE_ROBOT_THINKING
            status.best_move_details.uci = ""
        else:
            status.game_phase = ChessGameStatus.PHASE_WAITING_PLAYER

        node._on_game_status(status)
        assert node.mission_state == expected_state
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
    dispatcher = FakeMotionClient()
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
    dispatcher = FakeMotionClient()
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
        if is_capture:
            resp.message = "CAPTURE_TEST;bin=clear_left"
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
    dispatcher = FakeMotionClient()
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
        status.robot_color = "b"
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


def test_post_move_verification_repositioning_does_not_trip_disturbance(ros_context):
    """Verify that active base repositioning during POST_MOVE_VERIFYING does not trip disturbance into ERROR_FALLBACK."""
    repositioned_poses = []
    nav_completed_holder = []

    class AsyncRepositionDispatcher(FakeMotionClient):
        def __init__(self):
            super().__init__(None)

        def send_navigation_goal(self, target_pose, timeout_sec=60.0, on_completed=None):
            repositioned_poses.append(target_pose)
            self._is_navigating = True
            if on_completed:
                nav_completed_holder.append(on_completed)
            return True

    dispatcher = AsyncRepositionDispatcher()
    node = ChessMissionOrchestrator(motion_client=dispatcher)
    try:
        # Arm disturbance monitor with global odom
        node._health_supervisor.on_global_odom(Odometry())
        node._on_nav_ready(Bool(data=True))

        node.transition_to(MacroMissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MacroMissionState.CHECKING_REACHABILITY)
        node.transition_to(MacroMissionState.EXECUTING_MOVE_PIPELINE)
        node._on_move_pipeline_completed()
        assert node.mission_state == MacroMissionState.POST_MOVE_VERIFYING
        assert not node.post_move_verifier.is_repositioning

        # Watchdog timeout fires -> dispatches repositioning goal
        node._on_post_move_watchdog_timeout()
        assert len(repositioned_poses) == 1
        assert node.post_move_verifier.is_repositioning

        # Verify snapshot accurately reflects active navigation during repositioning
        snapshot = node._get_orchestrator_snapshot()
        assert snapshot.is_navigating
        assert snapshot.motion_state == MotionExecutionState.NAV_TO_OBS

        # Thân xe lăn bánh với vận tốc lớn (v = 0.20 m/s, wz = -0.34 rad/s)
        odom_moving = Odometry()
        odom_moving.twist.twist.linear.x = 0.20
        odom_moving.twist.twist.angular.z = -0.34
        node._health_supervisor.on_local_odom(odom_moving)

        # Disturbance must NOT be tripped, state must NOT drop into ERROR_FALLBACK
        assert not node._health_supervisor.is_operator_latched
        assert node.mission_state == MacroMissionState.POST_MOVE_VERIFYING

        # Complete Nav2 action
        assert len(nav_completed_holder) == 1
        dispatcher._is_navigating = False
        nav_completed_holder[0](ActionResult(success=True, message="Reached viewpoint"))

        # Chassis settled -> repositioning finishes
        assert not node.post_move_verifier.is_repositioning
        snapshot_after = node._get_orchestrator_snapshot()
        assert not snapshot_after.is_navigating
        assert snapshot_after.motion_state == MotionExecutionState.IDLE
        assert node.mission_state == MacroMissionState.POST_MOVE_VERIFYING
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


def test_disturbance_interrupt_and_one_button_recovery(ros_context):
    """Verify disturbance interrupt latches operator, and one-button recovery restores pipeline."""
    dispatcher = FakeMotionClient()
    node = ChessMissionOrchestrator(motion_client=dispatcher)
    try:
        node._on_nav_ready(Bool(data=True))
        node._on_grasp_ready(Bool(data=True))
        node.transition_to(MacroMissionState.WAITING_FOR_PLAYER_MOVE)

        # Start a chess move
        node._dispatch_move_workflow("e7e5")
        assert node.mission_state == MacroMissionState.CHECKING_REACHABILITY
        assert node.current_move_details.uci == "e7e5"

        # Simulate disturbance interrupt (e.g. uncommanded motion or chassis displacement)
        node._on_disturbance_detected("uncommanded_motion")
        assert node.mission_state == MacroMissionState.ERROR_FALLBACK
        assert node.pending_recovery_move is not None
        assert node.pending_recovery_move.uci == "e7e5"
        assert dispatcher.cancelled

        # Latch operator intervention and verify auto-recovery is inhibited
        node.health_supervisor.set_operator_latched(True)
        node.health_supervisor.schedule_auto_recovery_if_enabled()
        assert node.health_supervisor._recovery_timer is None

        # Operator triggers One-Button Recovery (/orchestrator/recover)
        req = Trigger.Request()
        resp = Trigger.Response()
        res = node._handle_recover_service(req, resp)
        assert res.success
        assert not node.health_supervisor.is_operator_latched
        assert node.mission_state == MacroMissionState.WAITING_FOR_TF_READY
        assert node.perception_context == PerceptionContext.TF_TRACKING_AND_NAV

        # Upon camera relocalization and nav ready, pending move is resumed
        node._on_nav_ready(Bool(data=True))
        assert node.mission_state == MacroMissionState.CHECKING_REACHABILITY
        assert node.current_move_details.uci == "e7e5"
    finally:
        node.destroy_node()


def test_recovery_from_non_error_state(ros_context):
    """Verify operator recovery can be initiated directly from any operational state."""
    node = ChessMissionOrchestrator()
    try:
        node._on_nav_ready(Bool(data=True))
        node.transition_to(MacroMissionState.WAITING_FOR_PLAYER_MOVE)
        assert node.mission_state == MacroMissionState.WAITING_FOR_PLAYER_MOVE

        # Trigger recovery directly from WAITING_FOR_PLAYER_MOVE
        req = Trigger.Request()
        resp = Trigger.Response()
        res = node._handle_recover_service(req, resp)
        assert res.success
        assert node.mission_state == MacroMissionState.WAITING_FOR_TF_READY
    finally:
        node.destroy_node()


def test_export_fen_service(ros_context):
    """Verify /orchestrator/export_fen returns latest full FEN and robot color as JSON."""
    import json
    node = ChessMissionOrchestrator()
    try:
        req = Trigger.Request()
        resp = Trigger.Response()

        # Before any game status received: export fails
        res = node._handle_export_fen_service(req, resp)
        assert not res.success

        # Receive game status with full FEN and robot color
        status = ChessGameStatus()
        status.full_fen = "rnbqkbnr/pppp1ppp/8/4p3/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2"
        status.robot_color = "b"
        node._on_game_status(status)

        res = node._handle_export_fen_service(req, resp)
        assert res.success
        payload = json.loads(res.message)
        assert payload["fen"] == status.full_fen
        assert payload["robot_color"] == "b"
    finally:
        node.destroy_node()


def test_import_fen_service_rejected_during_motion(ros_context):
    """Verify /orchestrator/import_fen is rejected when robot is in motion/planning states."""
    node = ChessMissionOrchestrator()
    try:
        node._on_nav_ready(Bool(data=True))
        node.transition_to(MacroMissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MacroMissionState.CHECKING_REACHABILITY)
        assert node.mission_state == MacroMissionState.CHECKING_REACHABILITY

        # Missing robot_color should be rejected immediately
        req_no_color = SetFen.Request(fen="8/8/8/4k3/8/8/4K3/8 w - - 0 1", robot_color="")
        resp_no_color = SetFen.Response()
        res_no_color = node._handle_import_fen_service(req_no_color, resp_no_color)
        assert not res_no_color.success
        assert "Missing required robot_color" in res_no_color.message

        req = SetFen.Request(fen="8/8/8/4k3/8/8/4K3/8 w - - 0 1", robot_color="b")
        resp = SetFen.Response()
        res = node._handle_import_fen_service(req, resp)

        assert not res.success
        assert "Cannot import FEN" in res.message
        assert node.mission_state == MacroMissionState.CHECKING_REACHABILITY
    finally:
        node.destroy_node()


def test_import_fen_service_accepted_and_sync_workflow(ros_context):
    """Verify import FEN workflow transitions to AWAITING_BOARD_SYNC and exits on stable board."""
    from rclpy.task import Future

    node = ChessMissionOrchestrator()
    try:
        node._on_nav_ready(Bool(data=True))
        assert node.mission_state == MacroMissionState.WAITING_FOR_PLAYER_MOVE

        target_fen = "rnbqkbnr/pppp1ppp/8/4p3/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2"

        # Mock chess_set_fen_client
        node._chess_set_fen_client.service_is_ready = lambda: True

        def mock_call_async(req):
            fut = Future()
            fut.set_result(
                SetFen.Response(
                    success=True,
                    canonical_fen=target_fen,
                    active_color="w",
                    robot_color="b",
                    message="OK",
                )
            )
            return fut

        node._chess_set_fen_client.call_async = mock_call_async

        req = SetFen.Request(fen=target_fen, robot_color="black")
        resp = SetFen.Response()
        res = node._handle_import_fen_service(req, resp)

        assert res.success
        assert res.canonical_fen == target_fen
        assert res.robot_color == "b"
        assert node.mission_state == MacroMissionState.AWAITING_BOARD_SYNC
        assert node.perception_context == PerceptionContext.BOARD_STATE_SCAN

        # Status reported as PHASE_AWAITING_BOARD_SYNC or unstable: remain in AWAITING_BOARD_SYNC
        status = ChessGameStatus()
        status.full_fen = target_fen
        status.active_color = "w"
        status.robot_color = "b"
        status.game_phase = ChessGameStatus.PHASE_AWAITING_BOARD_SYNC
        status.is_board_stable = False
        node._on_game_status(status)
        assert node.mission_state == MacroMissionState.AWAITING_BOARD_SYNC

        # Status reports board synchronized and stable (White turn, robot is Black)
        status.game_phase = ChessGameStatus.PHASE_WAITING_PLAYER
        status.is_board_stable = True
        node._on_game_status(status)

        # Robot is Black, active_color is White -> should transition to WAITING_FOR_PLAYER_MOVE
        assert node.mission_state == MacroMissionState.WAITING_FOR_PLAYER_MOVE
    finally:
        node.destroy_node()
