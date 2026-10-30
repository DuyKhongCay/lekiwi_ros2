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
        ("b", MacroMissionState.WAITING_FOR_NEXT_MOVE),
        ("w", MacroMissionState.EVALUATING_BEST_MOVE),
    ],
)
def test_game_start_by_player_color(ros_context, color, expected_state):
    """Verify game startup behavior depending on configured robot color (White vs Black)."""
    node = ChessMissionOrchestrator(
        parameter_overrides=[
            Parameter("robot_color", value="black" if color == "b" else "white"),
        ]
    )
    try:
        node.engine_client.compute_best_move_async = lambda **kwargs: True
        node._on_nav_ready(Bool(data=True))
        assert node.is_nav_ready
        assert node.mission_state == MacroMissionState.AWAITING_BOARD_SYNC
        assert node.perception_context == PerceptionContext.BOARD_STATE_SCAN

        # Incoming first game status message from chess master: White to move
        status = ChessGameStatus()
        status.full_fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
        status.active_color = "w"
        status.is_board_synced = True

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
        assert node.mission_state == MacroMissionState.AWAITING_BOARD_SYNC

        status_msg = ChessGameStatus()
        status_msg.is_board_synced = True
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
        assert node.mission_state == MacroMissionState.AWAITING_BOARD_SYNC

        # Synchronize board
        sync_status = ChessGameStatus()
        sync_status.is_board_synced = True
        node._on_game_status(sync_status)
        assert node.mission_state == MacroMissionState.WAITING_FOR_NEXT_MOVE

        # Non-critical state: losing nav ready doesn't crash player wait
        node._on_nav_ready(Bool(data=False))
        assert node.mission_state == MacroMissionState.WAITING_FOR_NEXT_MOVE

        # Critical state: losing nav ready during reachability triggers ERROR_FALLBACK
        node._on_nav_ready(Bool(data=True))
        node.transition_to(MacroMissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MacroMissionState.CHECKING_REACHABILITY)
        assert node.mission_state == MacroMissionState.CHECKING_REACHABILITY

        node._on_nav_ready(Bool(data=False))
        assert node.mission_state == MacroMissionState.ERROR_FALLBACK

        # Recovery from ERROR_FALLBACK back to RELOCALIZE
        node.trigger_recovery()
        assert node.mission_state == MacroMissionState.RELOCALIZE
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
        assert node.mission_state == MacroMissionState.RELOCALIZE
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
        assert node.mission_state == MacroMissionState.RELOCALIZE
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
        node.transition_to(MacroMissionState.WAITING_FOR_NEXT_MOVE)

        resp = CheckMoveFeasibility.Response()
        resp.feasible = True
        resp.plan_type = plan_type
        if is_capture:
            resp.message = "CAPTURE_TEST;bin=clear_left"
        resp.clear_point = Point(x=0.5, y=0.5, z=0.03)

        goal = ChessMoveGoal(
            uci="e7e5" if not is_capture else "e4d5",
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
        assert node.mission_state == MacroMissionState.AWAITING_BOARD_SYNC

        status = ChessGameStatus()
        status.is_board_synced = True
        node._on_game_status(status)
        assert node.mission_state == MacroMissionState.WAITING_FOR_NEXT_MOVE

        node._dispatch_move_workflow("c7c5")
        assert node.current_move_details.uci == "c7c5"
        assert node.mission_state == MacroMissionState.CHECKING_REACHABILITY

        node._on_feasibility_error("Reachability query timeout")
        assert node.mission_state == MacroMissionState.ERROR_FALLBACK

        recovered = node.trigger_recovery(reason="test_recovery")
        assert recovered
        assert node.mission_state == MacroMissionState.RELOCALIZE
        assert node.pending_recovery_move is not None
        assert node.pending_recovery_move.uci == "c7c5"

        # On next nav ready tick, robot transitions to AWAITING_BOARD_SYNC
        node._on_nav_ready(Bool(data=True))
        assert node.mission_state == MacroMissionState.AWAITING_BOARD_SYNC
        assert node.pending_recovery_move is not None

        # Once board is synced, interrupted move is resumed
        node._on_game_status(status)
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

        goal = ChessMoveGoal(uci="e7e5", is_capture=False)
        node._current_move_details = goal
        node.trigger_recovery(reason="test")
        assert node.pending_recovery_move.uci == "e7e5"

        # New game status arrives with different FEN (board modified externally)
        status = ChessGameStatus()
        status.active_color = "b"
        status.full_fen = "rnbqkbnr/pppp1ppp/8/4p3/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2"
        status.is_board_synced = True
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
        status.is_legal_move = True
        status.full_fen = "rnbqkbnr/pppp1ppp/8/4p3/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2"
        node._on_game_status(status)

        assert node.mission_state == MacroMissionState.WAITING_FOR_NEXT_MOVE
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
    """Verify that is_legal_move=False does NOT finalize turn."""
    node = ChessMissionOrchestrator()
    try:
        node._on_nav_ready(Bool(data=True))
        node.transition_to(MacroMissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MacroMissionState.CHECKING_REACHABILITY)
        node.transition_to(MacroMissionState.EXECUTING_MOVE_PIPELINE)
        node._on_move_pipeline_completed()
        assert node.mission_state == MacroMissionState.POST_MOVE_VERIFYING
        assert node.post_move_verifier.is_active

        # Occluded vision: move is illegal
        status = ChessGameStatus()
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
        node.transition_to(MacroMissionState.WAITING_FOR_NEXT_MOVE)
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
        node.transition_to(MacroMissionState.WAITING_FOR_NEXT_MOVE)

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
        assert node.mission_state == MacroMissionState.RELOCALIZE
        assert node.perception_context == PerceptionContext.TF_TRACKING_AND_NAV

        # Upon camera relocalization and nav ready, enters AWAITING_BOARD_SYNC
        node._on_nav_ready(Bool(data=True))
        assert node.mission_state == MacroMissionState.AWAITING_BOARD_SYNC

        # Once board is verified synced, pending move is resumed
        sync_status = ChessGameStatus()
        sync_status.is_board_synced = True
        node._on_game_status(sync_status)
        assert node.mission_state == MacroMissionState.CHECKING_REACHABILITY
        assert node.current_move_details.uci == "e7e5"
    finally:
        node.destroy_node()


def test_recovery_from_non_error_state(ros_context):
    """Verify operator recovery can be initiated directly from any operational state."""
    node = ChessMissionOrchestrator()
    try:
        node._on_nav_ready(Bool(data=True))
        node.transition_to(MacroMissionState.WAITING_FOR_NEXT_MOVE)
        assert node.mission_state == MacroMissionState.WAITING_FOR_NEXT_MOVE

        # Trigger recovery directly from WAITING_FOR_NEXT_MOVE
        req = Trigger.Request()
        resp = Trigger.Response()
        res = node._handle_recover_service(req, resp)
        assert res.success
        assert node.mission_state == MacroMissionState.RELOCALIZE
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

        # Receive game status with full FEN and set robot color
        node.game_status_handler.set_robot_color("b")
        status = ChessGameStatus()
        status.full_fen = "rnbqkbnr/pppp1ppp/8/4p3/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2"
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

        # Empty FEN should be rejected immediately
        req_empty_fen = SetFen.Request(fen="")
        resp_empty_fen = SetFen.Response()
        res_empty_fen = node._handle_import_fen_service(req_empty_fen, resp_empty_fen)
        assert not res_empty_fen.success
        assert "Missing required FEN" in res_empty_fen.message

        req = SetFen.Request(fen="8/8/8/4k3/8/8/4K3/8 w - - 0 1")
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
    node.game_status_handler.set_robot_color("b")
    node._health_supervisor.set_robot_color("b")
    try:
        node._on_nav_ready(Bool(data=True))
        assert node.mission_state == MacroMissionState.AWAITING_BOARD_SYNC
        assert node._execute_collection_task_server is not None

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
                    message="OK",
                )
            )
            return fut

        node._chess_set_fen_client.call_async = mock_call_async

        req = SetFen.Request(fen=target_fen)
        resp = SetFen.Response()
        res = node._handle_import_fen_service(req, resp)

        assert res.success
        assert res.canonical_fen == target_fen
        assert node.mission_state == MacroMissionState.AWAITING_BOARD_SYNC
        assert node.perception_context == PerceptionContext.BOARD_STATE_SCAN
        assert node._execute_collection_task_server is not None

        # Status reported as unsynced: remain in AWAITING_BOARD_SYNC
        status = ChessGameStatus()
        status.full_fen = target_fen
        status.active_color = "w"
        status.is_board_synced = False
        node._on_game_status(status)
        assert node.mission_state == MacroMissionState.AWAITING_BOARD_SYNC
        assert node._execute_collection_task_server is not None

        # Status reports board synchronized (White turn, robot is Black)
        status.is_board_synced = True
        node._on_game_status(status)

        # Board synchronized and stable -> transitions to WAITING_FOR_NEXT_MOVE
        assert node.mission_state == MacroMissionState.WAITING_FOR_NEXT_MOVE
        assert node._execute_collection_task_server is not None
    finally:
        node.destroy_node()


def test_import_fen_service_accepted_during_waiting_for_tf_ready(ros_context):
    """Verify /orchestrator/import_fen is accepted during WAITING_FOR_TF_READY without prematurely leaving boot readiness."""
    from rclpy.task import Future

    node = ChessMissionOrchestrator()
    try:
        assert node.mission_state == MacroMissionState.WAITING_FOR_TF_READY

        target_fen = "4k3/8/4n2p/1p2P1bR/7B/7B/q7/4K3 w - - 0 1"

        node._chess_set_fen_client.service_is_ready = lambda: True

        def mock_call_async(req):
            fut = Future()
            fut.set_result(
                SetFen.Response(
                    success=True,
                    canonical_fen=target_fen,
                    active_color="w",
                    message="OK",
                )
            )
            return fut

        node._chess_set_fen_client.call_async = mock_call_async

        req = SetFen.Request(fen=target_fen)
        resp = SetFen.Response()
        res = node._handle_import_fen_service(req, resp)

        assert res.success
        assert res.canonical_fen == target_fen
        # Must maintain WAITING_FOR_TF_READY until Navigation confirms ready
        assert node.mission_state == MacroMissionState.WAITING_FOR_TF_READY

        # Now confirm nav ready -> should transition to AWAITING_BOARD_SYNC
        node._on_nav_ready(Bool(data=True))
        assert node.mission_state == MacroMissionState.AWAITING_BOARD_SYNC
    finally:
        node.destroy_node()


def test_execute_collection_task_action(ros_context):
    """Verify /orchestrator/execute_collection_task action server, goal callback, and execution lifecycle."""
    import threading
    from lekiwi_interfaces.action import ExecuteCollectionTask
    from lekiwi_interfaces.msg import ChessMoveDetails
    from rclpy.action import CancelResponse, GoalResponse

    class MockGoalHandle:
        def __init__(self, request):
            self.request = request
            self.is_active = True
            self.is_cancel_requested = False
            self.succeeded = False
            self.aborted = False
            self.canceled_flag = False
            self.feedback_list = []

        def publish_feedback(self, fb):
            self.feedback_list.append(fb)

        def succeed(self):
            self.succeeded = True
            self.is_active = False

        def abort(self):
            self.aborted = True
            self.is_active = False

        def canceled(self):
            self.canceled_flag = True
            self.is_active = False

    dispatcher = FakeMotionClient()
    node = ChessMissionOrchestrator(motion_client=dispatcher)
    try:
        # 1. Verify Action Server is NOT created while in WAITING_FOR_TF_READY (Deferred Activation)
        assert node.mission_state == MacroMissionState.WAITING_FOR_TF_READY
        assert node._execute_collection_task_server is None

        # 2. Confirm nav ready -> transitions to AWAITING_BOARD_SYNC and activates Action Server
        node._on_nav_ready(Bool(data=True))
        assert node.mission_state == MacroMissionState.AWAITING_BOARD_SYNC
        assert node._execute_collection_task_server is not None

        # 3. Verify goal rejected while in AWAITING_BOARD_SYNC
        goal_req = ExecuteCollectionTask.Goal()
        goal_req.move = ChessMoveDetails(uci="e2e4", piece_type="pawn", piece_color="w")
        goal_resp = node._collection_task_goal_callback(goal_req)
        assert goal_resp == GoalResponse.REJECT

        # Confirm physical board synchronization -> transitions to WAITING_FOR_NEXT_MOVE
        sync_status = ChessGameStatus()
        sync_status.is_board_synced = True
        node._on_game_status(sync_status)
        assert node.mission_state == MacroMissionState.WAITING_FOR_NEXT_MOVE

        # Now goal is accepted
        goal_resp = node._collection_task_goal_callback(goal_req)
        assert goal_resp == GoalResponse.ACCEPT

        # 4. Mock candidate move publisher to verify broadcasts
        candidate_calls = []
        node._candidate_move_pub.publish = lambda msg: candidate_calls.append(msg)

        goal_req.force_pick_place = True
        goal_req.clear_bin_side = 1
        goal_req.bypass_steps = ["MOVE"]

        mock_handle = MockGoalHandle(goal_req)

        # Run execute callback in background thread
        exec_result = []

        def run_execute():
            res = node._execute_collection_task_callback(mock_handle)
            exec_result.append(res)

        th = threading.Thread(target=run_execute)
        th.start()

        # Wait briefly for execute callback to initialize and dispatch
        import time
        for _ in range(50):
            if node._current_goal_move == "e2e4":
                break
            time.sleep(0.02)

        assert node._current_goal_move == "e2e4"
        assert node.mission_state == MacroMissionState.CHECKING_REACHABILITY

        # Verify feedback was published
        assert len(mock_handle.feedback_list) >= 1
        assert mock_handle.feedback_list[0].current_stage == "CHECKING_REACHABILITY"

        # Verify candidate move was published
        assert len(candidate_calls) == 1
        assert candidate_calls[0].uci == "e2e4"

        # Verify second goal rejected while current goal is active
        assert node._collection_task_goal_callback(goal_req) == GoalResponse.REJECT

        # 5. Finalize turn -> completes the action goal
        node._game_status_handler._latest_full_fen = "rnbqkbnr/pppp1ppp/8/4p3/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2"
        node._finalize_turn_completed()

        th.join(timeout=2.0)
        assert not th.is_alive()
        assert len(exec_result) == 1
        assert exec_result[0].success is True
        assert exec_result[0].final_fen == "rnbqkbnr/pppp1ppp/8/4p3/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2"
        assert mock_handle.succeeded is True

        # Verify candidate move cleared on turn completion
        assert len(candidate_calls) == 2
        assert candidate_calls[1].uci == ""

        # 6. Verify cancel callback
        cancel_handle = MockGoalHandle(goal_req)
        cancel_resp = node._collection_task_cancel_callback(cancel_handle)
        assert cancel_resp == CancelResponse.ACCEPT

        # 7. Verify Action Server is maintained alive during RELOCALIZE (no DDS churn), but rejects goals
        node.trigger_recovery(reason="test_recovery")
        assert node.mission_state == MacroMissionState.RELOCALIZE
        assert node._execute_collection_task_server is not None
        assert node._collection_task_goal_callback(goal_req) == GoalResponse.REJECT

        # 8. Verify active goal is aborted cleanly on recovery
        active_handle = MockGoalHandle(goal_req)
        node._active_collection_goal_handle = active_handle
        node.trigger_recovery(reason="abort_test")
        assert active_handle.aborted is True
        assert node._active_collection_goal_handle is None
    finally:
        node.destroy_node()


def test_recovery_resume_preserves_black_move_instructions(ros_context):
    """Regression test: Interrupted black move (e.g. f2h2) preserved and resumed with correct color."""
    motion = FakeMotionClient()
    node = ChessMissionOrchestrator(motion_client=motion)
    try:
        # Simulate an interrupted black move collection task (f2h2: black rook captures white pawn)
        black_goal = ChessMoveGoal(
            uci="f2h2",
            piece_type="rook:pawn",
            piece_color="b",
            is_capture=True,
            force_pick_place=False,
            clear_bin_side="right",
        )
        assert black_goal.color_name == "black"
        assert black_goal.opposing_color_name == "white"

        node._current_move_details = black_goal
        node._current_goal_move = black_goal.uci

        # Simulate trigger recovery into RELOCALIZE
        node.trigger_recovery(reason="nav_failure")
        assert node.mission_state == MacroMissionState.RELOCALIZE
        assert node.pending_recovery_move is not None
        assert node.pending_recovery_move.uci == "f2h2"
        assert node.pending_recovery_move.piece_color == "b"

        # Simulate Nav2 ready confirmation -> Triggers resumption
        node._health_supervisor._is_nav_ready = True
        node._on_nav_ready_confirmed(is_ready=True)

        # Confirm the resumed move remains black and retains all properties
        assert node.current_move_details is not None
        assert node.current_move_details.uci == "f2h2"
        assert node.current_move_details.piece_color == "b"
        assert node.current_move_details.color_name == "black"
        assert node.current_move_details.opposing_color_name == "white"
        assert node.current_move_details.is_capture is True
    finally:
        node.destroy_node()


def test_autonomous_engine_trigger_and_record_mode(ros_context):
    """Verify autonomous mode triggers EngineClient, while record mode waits for external task."""
    # 1. Record mode = True: no engine call on robot's turn
    node_rec = ChessMissionOrchestrator(
        motion_client=FakeMotionClient(),
        parameter_overrides=[
            Parameter("record", value=True),
            Parameter("robot_color", value="white"),
        ],
    )
    try:
        engine_called = []
        node_rec.engine_client.compute_best_move_async = lambda **kwargs: engine_called.append(True)
        node_rec._on_nav_ready(Bool(data=True))

        status = ChessGameStatus()
        status.full_fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
        status.active_color = "w"
        status.is_board_synced = True
        node_rec._on_game_status(status)

        assert len(engine_called) == 0
        assert node_rec.mission_state == MacroMissionState.WAITING_FOR_NEXT_MOVE

        # In record mode, external collection task transitions directly to CHECKING_REACHABILITY
        goal = ChessMoveGoal(uci="e2e4", piece_type="pawn", piece_color="w")
        ok, msg = node_rec._move_workflow.dispatch_collection_task(goal)
        assert ok, msg
        assert node_rec.mission_state == MacroMissionState.CHECKING_REACHABILITY
        assert node_rec.mission_state != MacroMissionState.EVALUATING_BEST_MOVE
    finally:
        node_rec.destroy_node()

    # 2. Record mode = False: engine is invoked on robot's turn
    dispatcher = FakeMotionClient()
    node_auto = ChessMissionOrchestrator(
        motion_client=dispatcher,
        parameter_overrides=[
            Parameter("record", value=False),
            Parameter("robot_color", value="white"),
        ],
    )
    try:
        engine_calls = []

        def mock_engine(fen, think_time_ms, on_complete):
            engine_calls.append((fen, think_time_ms))
            on_complete("e2e4", "OK")
            return True

        node_auto.engine_client.compute_best_move_async = mock_engine
        node_auto._on_nav_ready(Bool(data=True))

        status = ChessGameStatus()
        status.full_fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
        status.active_color = "w"
        status.is_board_synced = True
        node_auto._on_game_status(status)

        assert len(engine_calls) == 1
        assert engine_calls[0][0] == status.full_fen
        # After engine returns move, it transitions to reachability check
        assert node_auto.mission_state == MacroMissionState.CHECKING_REACHABILITY
        assert node_auto.current_move_details.uci == "e2e4"
    finally:
        node_auto.destroy_node()


def test_action_server_lifecycle_and_service_timeout_board_sync(ros_context):
    """Verify ExecuteCollectionTask action server is maintained in AWAITING_BOARD_SYNC and accepts in WAITING_FOR_NEXT_MOVE."""
    from rclpy.task import Future
    from lekiwi_interfaces.action import ExecuteCollectionTask
    from lekiwi_interfaces.msg import ChessMoveDetails
    from rclpy.action import GoalResponse

    node = ChessMissionOrchestrator()
    node.game_status_handler.set_robot_color("b")
    node._health_supervisor.set_robot_color("b")
    try:
        # Check service_timeout_sec parameter loaded
        assert hasattr(node.parameters, "service_timeout_sec")
        assert node.parameters.service_timeout_sec == 5.0
        assert not hasattr(node.parameters, "feasibility_timeout_sec")

        # In WAITING_FOR_TF_READY, action server is not active
        assert node.mission_state == MacroMissionState.WAITING_FOR_TF_READY
        assert node._execute_collection_task_server is None

        # Transition to WAITING_FOR_NEXT_MOVE path -> action server activated
        node._on_nav_ready(Bool(data=True))
        assert node.mission_state == MacroMissionState.AWAITING_BOARD_SYNC
        assert node._execute_collection_task_server is not None

        # Import FEN -> transitions to AWAITING_BOARD_SYNC
        node._chess_set_fen_client.service_is_ready = lambda: True
        target_fen = "rnbqkbnr/pppp1ppp/8/4p3/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2"

        def mock_call_async(req):
            fut = Future()
            fut.set_result(
                SetFen.Response(
                    success=True,
                    canonical_fen=target_fen,
                    active_color="w",
                    message="OK",
                )
            )
            return fut

        node._chess_set_fen_client.call_async = mock_call_async
        req = SetFen.Request(fen=target_fen)
        resp = SetFen.Response()
        res = node._handle_import_fen_service(req, resp)
        assert res.success

        # State is AWAITING_BOARD_SYNC -> Action server maintained alive (no churn), but rejects goals
        assert node.mission_state == MacroMissionState.AWAITING_BOARD_SYNC
        assert node._execute_collection_task_server is not None

        # Goal callback rejects any goal while in AWAITING_BOARD_SYNC
        goal_req = ExecuteCollectionTask.Goal()
        goal_req.move = ChessMoveDetails(uci="e2e4", piece_type="pawn", piece_color="w")
        goal_resp = node._collection_task_goal_callback(goal_req)
        assert goal_resp == GoalResponse.REJECT

        # Board is confirmed synced -> transitions to WAITING_FOR_NEXT_MOVE
        status = ChessGameStatus()
        status.full_fen = target_fen
        status.active_color = "w"
        status.is_board_synced = True
        node._on_game_status(status)

        assert node.mission_state == MacroMissionState.WAITING_FOR_NEXT_MOVE
        assert node._execute_collection_task_server is not None

        # Now goal callback accepts goal
        goal_resp_synced = node._collection_task_goal_callback(goal_req)
        assert goal_resp_synced == GoalResponse.ACCEPT
    finally:
        node.destroy_node()


