# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for ChessMissionOrchestrator logic, 2-level HFSM, PerceptionContext, and self-healing."""

from __future__ import annotations

import math

import pytest
import rclpy
from lekiwi_interfaces.msg import (
    ChessGameStatus,
    ChessMoveDetails,
    PerceptionContext,
)
from lekiwi_interfaces.srv import CheckMoveFeasibility
from lekiwi_orchestrator.orchestrator_node import (
    ChessMissionOrchestrator,
)
from lekiwi_orchestrator.fsm import (
    MacroMissionState,
    MotionExecutionState,
)
from lekiwi_orchestrator.mission_types import (
    ActionResult,
    ChessMoveGoal,
    ObservationIntent,
)
from lekiwi_orchestrator.motion_client import (
    FakeMotionClient,
    MotionClient,
)
from rclpy.parameter import Parameter
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class TrackingDispatcher(MotionClient):
    """Local test tracking dispatcher for orchestrator tests."""

    def __init__(self):
        self.nav_goals = []
        self.manip_goals = []

    def check_feasibility(
        self,
        goal,
        timeout_sec=5.0,
        on_success=None,
        on_error=None,
    ):
        return True

    def send_navigation_goal(
        self,
        target_pose,
        timeout_sec=60.0,
        on_completed=None,
    ):
        self.nav_goals.append(target_pose)
        if on_completed:
            on_completed(
                ActionResult(success=True, message="OK", execution_time_sec=0.1)
            )
        return True

    def send_manipulation_goal(
        self,
        goal,
        timeout_sec=60.0,
        on_feedback=None,
        on_completed=None,
    ):
        self.manip_goals.append(goal)
        if on_completed:
            on_completed(
                ActionResult(success=True, message="Done", execution_time_sec=0.5)
            )
        return True

    def cancel_active_goal(self):
        pass

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
    node = ChessMissionOrchestrator()
    try:
        assert node.mission_state == MacroMissionState.WAITING_FOR_TF_READY
        assert node.motion_state == MotionExecutionState.IDLE
        assert node.perception_context == PerceptionContext.TF_TRACKING_AND_NAV
        assert not node.is_nav_ready
    finally:
        node.destroy_node()


def test_nav_readiness_starts_game_for_black_robot(ros_context):
    node = ChessMissionOrchestrator()
    try:
        # Emit Nav ready
        nav_msg = Bool()
        nav_msg.data = True
        node._on_nav_ready(nav_msg)

        assert node.is_nav_ready
        # Default robot_color="b" waits for White player move
        assert node.mission_state == MacroMissionState.WAITING_FOR_PLAYER_MOVE
        assert node.perception_context == PerceptionContext.BOARD_STATE_SCAN
    finally:
        node.destroy_node()


def test_nav_readiness_white_robot_starts_evaluating(ros_context):
    node = ChessMissionOrchestrator(
        parameter_overrides=[Parameter("robot_color", value="w")]
    )
    try:
        nav_msg = Bool()
        nav_msg.data = True
        node._on_nav_ready(nav_msg)

        assert node.is_nav_ready
        # If White, robot must think and evaluate first move
        assert node.mission_state == MacroMissionState.EVALUATING_BEST_MOVE
        assert node.perception_context == PerceptionContext.BOARD_STATE_SCAN
    finally:
        node.destroy_node()


def test_game_over_on_checkmate(ros_context):
    node = ChessMissionOrchestrator()
    try:
        node._on_nav_ready(Bool(data=True))

        status_msg = ChessGameStatus()
        status_msg.is_checkmate = True
        status_msg.full_fen = (
            "rnb1kbnr/pppp1ppp/8/4p3/6Pq/5P2/PPPPP2P/RNBQKBNR w KQkq - 1 3"
        )
        node._on_game_status(status_msg)

        assert node.mission_state == MacroMissionState.GAME_OVER
        assert node.motion_state == MotionExecutionState.IDLE
        assert node.perception_context == PerceptionContext.IDLE_STANDBY
    finally:
        node.destroy_node()


def test_game_over_on_draw(ros_context):
    node = ChessMissionOrchestrator()
    try:
        node._on_nav_ready(Bool(data=True))

        status_msg = ChessGameStatus()
        status_msg.is_draw = True
        node._on_game_status(status_msg)

        assert node.mission_state == MacroMissionState.GAME_OVER
        assert node.motion_state == MotionExecutionState.IDLE
        assert node.perception_context == PerceptionContext.IDLE_STANDBY
    finally:
        node.destroy_node()


def test_illegal_state_transition_guard(ros_context):
    node = ChessMissionOrchestrator()
    try:
        assert node.mission_state == MacroMissionState.WAITING_FOR_TF_READY
        success = node.transition_to(MacroMissionState.EXECUTING_MOVE_PIPELINE)
        assert not success
        assert node.mission_state == MacroMissionState.WAITING_FOR_TF_READY
    finally:
        node.destroy_node()


def test_workflow_dispatch_zero_nav_simulation(ros_context):
    sim_dispatcher = FakeMotionClient(None)
    node = ChessMissionOrchestrator(
        parameter_overrides=[Parameter("navigation", value=False)],
        motion_client=sim_dispatcher,
    )
    sim_dispatcher._node = node
    try:
        node._on_nav_ready(Bool(data=True))
        node._on_grasp_ready(Bool(data=True))
        assert node.mission_state == MacroMissionState.WAITING_FOR_PLAYER_MOVE

        resp = CheckMoveFeasibility.Response()
        resp.feasible = True
        resp.plan_type = CheckMoveFeasibility.Response.PLAN_ZERO_NAV
        resp.message = "PLAN_ZERO_NAV verified"

        details = ChessMoveGoal(
            uci="e7e5",
            from_square="e7",
            to_square="e5",
            promotion=None,
            is_capture=False,
        )

        node.transition_to(MacroMissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MacroMissionState.CHECKING_REACHABILITY)
        node._on_feasibility_response(resp, details)

        assert node.mission_state == MacroMissionState.POST_MOVE_VERIFYING
        assert node.motion_state == MotionExecutionState.IDLE
        assert node.perception_context == PerceptionContext.BOARD_STATE_SCAN

        # Board verified
        status = ChessGameStatus()
        status.is_board_stable = True
        status.is_legal_move = True
        status.full_fen = "rnbqkbnr/pppp1ppp/8/4p3/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2"
        node._on_game_status(status)

        assert node.mission_state == MacroMissionState.WAITING_FOR_PLAYER_MOVE
    finally:
        node.destroy_node()


def test_move_details_dispatching(ros_context):
    """Verify ChessMoveDetails message populates ChessMoveGoal correctly."""
    node = ChessMissionOrchestrator()
    try:
        node._on_nav_ready(Bool(data=True))

        move_details = ChessMoveDetails()
        move_details.uci = "e5f6"
        move_details.san = "exf6"
        move_details.from_square = "e5"
        move_details.to_square = "f6"
        move_details.piece_type = "pawn"
        move_details.is_capture = True
        move_details.is_en_passant = True
        move_details.captured_square = "f5"

        status_msg = ChessGameStatus()
        status_msg.active_color = "b"  # robot is "b"
        status_msg.full_fen = (
            "rnbqkbnr/ppp1p1pp/8/3pPp2/8/8/PPPP1PPP/RNBQKBNR w KQkq f6 0 3"
        )
        status_msg.best_move_details = move_details
        status_msg.is_board_stable = True
        status_msg.game_phase = ChessGameStatus.PHASE_ROBOT_READY

        node._on_game_status(status_msg)

        assert node.current_move_details is not None
        assert node.current_move_details.uci == "e5f6"
        assert node.current_move_details.from_square == "e5"
        assert node.current_move_details.to_square == "f6"
        assert node.current_move_details.is_capture is True
        assert node.current_move_details.captured_square == "f5"
    finally:
        node.destroy_node()


def test_nav_readiness_gating_behavior(ros_context):
    node = ChessMissionOrchestrator()
    try:
        # 1. Critical state: WAITING_FOR_TF_READY
        assert node.mission_state == MacroMissionState.WAITING_FOR_TF_READY
        node._on_nav_ready(Bool(data=True))
        assert node.mission_state == MacroMissionState.WAITING_FOR_PLAYER_MOVE

        # 2. Un-gated state: WAITING_FOR_PLAYER_MOVE
        node._on_nav_ready(Bool(data=False))
        assert node.mission_state == MacroMissionState.WAITING_FOR_PLAYER_MOVE

        # Re-enable Nav ready
        node._on_nav_ready(Bool(data=True))
        assert node.mission_state == MacroMissionState.WAITING_FOR_PLAYER_MOVE

        # 3. Critical state: CHECKING_REACHABILITY
        node.transition_to(MacroMissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MacroMissionState.CHECKING_REACHABILITY)
        assert node.mission_state == MacroMissionState.CHECKING_REACHABILITY
        node._on_nav_ready(Bool(data=False))
        assert node.mission_state == MacroMissionState.ERROR_FALLBACK

        # 4. Recover from ERROR_FALLBACK to WAITING_FOR_TF_READY
        node.trigger_recovery()
        assert node.mission_state == MacroMissionState.WAITING_FOR_TF_READY

        node._on_nav_ready(Bool(data=True))
        node.transition_to(MacroMissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MacroMissionState.CHECKING_REACHABILITY)
        node.transition_to(MacroMissionState.EXECUTING_MOVE_PIPELINE)
        assert node.mission_state == MacroMissionState.EXECUTING_MOVE_PIPELINE

        # 5. Un-gated state: EXECUTING_MOVE_PIPELINE
        node._on_nav_ready(Bool(data=False))
        assert node.mission_state == MacroMissionState.EXECUTING_MOVE_PIPELINE
    finally:
        node.destroy_node()


def test_feasibility_timeout_watchdog(ros_context):
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
    class _MockDispatcher(TrackingDispatcher):
        def __init__(self):
            super().__init__()
            self.cancelled = False

        def cancel_active_goal(self):
            self.cancelled = True

    mock_dispatcher = _MockDispatcher()
    node = ChessMissionOrchestrator(motion_client=mock_dispatcher)
    try:
        node._on_nav_ready(Bool(data=True))
        node.transition_to(MacroMissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MacroMissionState.CHECKING_REACHABILITY)
        node.transition_to(MacroMissionState.EXECUTING_MOVE_PIPELINE)
        assert node.mission_state == MacroMissionState.EXECUTING_MOVE_PIPELINE

        # When action execution fails or times out, orchestrator transitions to ERROR_FALLBACK
        node._on_move_pipeline_failed("Action execution timed out")
        assert node.mission_state == MacroMissionState.ERROR_FALLBACK
        assert node.motion_state == MotionExecutionState.IDLE

        # Triggering recovery cancels active goals on dispatcher and resets pipeline
        node.trigger_recovery()
        assert mock_dispatcher.cancelled
        assert node.mission_state == MacroMissionState.WAITING_FOR_TF_READY
        assert node.motion_state == MotionExecutionState.IDLE
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


def test_auto_recovery_timer_triggers_recovery(ros_context):
    """Verify automatic recovery timer transitions node back to WAITING_FOR_TF_READY."""
    node = ChessMissionOrchestrator(
        parameter_overrides=[
            Parameter("recovery.auto_recovery_enabled", value=True),
            Parameter("recovery.auto_recovery_timeout_sec", value=0.05),
            Parameter("recovery.max_recovery_attempts", value=3),
        ]
    )
    try:
        node._on_nav_ready(Bool(data=True))
        node.transition_to(MacroMissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MacroMissionState.CHECKING_REACHABILITY)
        node.transition_to(MacroMissionState.ERROR_FALLBACK)
        assert node.mission_state == MacroMissionState.ERROR_FALLBACK
        assert node.health_supervisor.recovery_attempts == 1

        # Fire auto recovery timer directly via health monitor
        node.health_supervisor._on_auto_recovery_timer_fired()
        assert node.mission_state == MacroMissionState.WAITING_FOR_TF_READY
        assert node.motion_state == MotionExecutionState.IDLE
    finally:
        node.destroy_node()


def test_motion_client_dependency_injection(ros_context):
    """Verify custom MotionClient implementation can be cleanly injected."""

    class CustomDispatcher(MotionClient):
        def __init__(self):
            self.nav_called = False

        def check_feasibility(
            self,
            goal,
            timeout_sec=5.0,
            on_success=None,
            on_error=None,
        ):
            return True

        def send_navigation_goal(
            self,
            target_pose,
            timeout_sec=60.0,
            on_completed=None,
        ):
            self.nav_called = True
            return True

        def send_manipulation_goal(
            self,
            goal,
            timeout_sec=60.0,
            on_feedback=None,
            on_completed=None,
        ):
            return True

        def cancel_active_goal(self):
            pass

        def destroy(self):
            pass

    custom_dispatcher = CustomDispatcher()
    node = ChessMissionOrchestrator(motion_client=custom_dispatcher)
    try:
        assert node.motion_client is custom_dispatcher
    finally:
        node.destroy_node()


def test_workflow_dispatch_capture_single_base_sequence(ros_context):
    """Verify capture single base executes Clear (capture=True) then Move (capture=False)."""
    dispatcher = TrackingDispatcher()
    node = ChessMissionOrchestrator(motion_client=dispatcher)
    try:
        node._on_nav_ready(Bool(data=True))
        node._on_grasp_ready(Bool(data=True))

        resp = CheckMoveFeasibility.Response()
        resp.feasible = True
        resp.plan_type = CheckMoveFeasibility.Response.PLAN_CAPTURE_SINGLE_BASE
        resp.clear_base_pose.pose.position.x = 0.5
        resp.pick_base_pose.pose.position.x = 0.5
        resp.place_base_pose.pose.position.x = 0.5

        details = ChessMoveGoal(
            uci="e4d5",
            from_square="e4",
            to_square="d5",
            is_capture=True,
            captured_square="d5",
        )

        node.transition_to(MacroMissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MacroMissionState.CHECKING_REACHABILITY)
        node._on_feasibility_response(resp, details)

        # 2 Nav goals: common standoff + post-move observation standoff retreat
        assert len(dispatcher.nav_goals) == 2
        assert dispatcher.nav_goals[0].pose.position.x == 0.5
        assert math.isclose(dispatcher.nav_goals[1].pose.position.x, 0.65, abs_tol=1e-3)
        assert math.isclose(dispatcher.nav_goals[1].pose.position.y, 0.0, abs_tol=1e-3)
        # 2 Manipulation goals: Clear d5 (is_capture=True), then Move e4->d5 (is_capture=False)
        assert len(dispatcher.manip_goals) == 2
        assert dispatcher.manip_goals[0].is_capture is True
        assert dispatcher.manip_goals[0].from_square == "d5"
        assert dispatcher.manip_goals[1].is_capture is False
        assert dispatcher.manip_goals[1].from_square == "e4"
        assert dispatcher.manip_goals[1].to_square == "d5"
        assert node.mission_state == MacroMissionState.POST_MOVE_VERIFYING
        assert node.motion_state == MotionExecutionState.IDLE

        status = ChessGameStatus()
        status.is_board_stable = True
        status.is_legal_move = True
        status.full_fen = "rnbqkbnr/pppp1ppp/8/4p3/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2"
        node._on_game_status(status)

        assert node.mission_state == MacroMissionState.WAITING_FOR_PLAYER_MOVE
    finally:
        node.destroy_node()


def test_workflow_dispatch_capture_triple_base_sequence(ros_context):
    """Verify capture triple base executes Clear -> Pick -> Place across 3 base standoffs."""
    dispatcher = TrackingDispatcher()
    node = ChessMissionOrchestrator(motion_client=dispatcher)
    try:
        node._on_nav_ready(Bool(data=True))
        node._on_grasp_ready(Bool(data=True))

        resp = CheckMoveFeasibility.Response()
        resp.feasible = True
        resp.plan_type = CheckMoveFeasibility.Response.PLAN_CAPTURE_TRIPLE_BASE
        resp.clear_base_pose.pose.position.x = 1.0
        resp.pick_base_pose.pose.position.x = 2.0
        resp.place_base_pose.pose.position.x = 1.0

        details = ChessMoveGoal(
            uci="a1h8",
            from_square="a1",
            to_square="h8",
            is_capture=True,
            captured_square="h8",
        )

        node.transition_to(MacroMissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MacroMissionState.CHECKING_REACHABILITY)
        node._on_feasibility_response(resp, details)

        # 4 Nav goals: clear_base -> pick_base -> place_base -> observation standoff retreat
        assert len(dispatcher.nav_goals) == 4
        assert dispatcher.nav_goals[0].pose.position.x == 1.0
        assert dispatcher.nav_goals[1].pose.position.x == 2.0
        assert dispatcher.nav_goals[2].pose.position.x == 1.0
        assert math.isclose(dispatcher.nav_goals[3].pose.position.x, 0.65, abs_tol=1e-3)
        assert math.isclose(dispatcher.nav_goals[3].pose.position.y, 0.0, abs_tol=1e-3)

        # 3 Manipulation goals: Clear h8 -> Pick a1 -> Place h8
        assert len(dispatcher.manip_goals) == 3
        assert dispatcher.manip_goals[0].is_capture is True
        assert dispatcher.manip_goals[0].from_square == "h8"
        assert dispatcher.manip_goals[1].is_capture is False
        assert dispatcher.manip_goals[1].from_square == "a1"
        assert dispatcher.manip_goals[2].is_capture is False
        assert dispatcher.manip_goals[2].to_square == "h8"

        assert node.mission_state == MacroMissionState.POST_MOVE_VERIFYING
        assert node.motion_state == MotionExecutionState.IDLE

        status = ChessGameStatus()
        status.is_board_stable = True
        status.is_legal_move = True
        status.full_fen = "rnbqkbnr/pppp1ppp/8/4p3/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2"
        node._on_game_status(status)

        assert node.mission_state == MacroMissionState.WAITING_FOR_PLAYER_MOVE
    finally:
        node.destroy_node()


def test_2level_hierarchical_fsm_micro_stage_transitions(ros_context):
    """Verify Level 2 MotionExecutionState and PerceptionContext transitions during stage progression."""

    class StateTrackingDispatcher(MotionClient):
        def __init__(self, node):
            self.node = node
            self.snapshots = []

        def check_feasibility(
            self, goal, timeout_sec=5.0, on_success=None, on_error=None
        ):
            return True

        def send_navigation_goal(
            self, target_pose, timeout_sec=60.0, on_completed=None
        ):
            self.snapshots.append(
                ("NAV", self.node.motion_state, self.node.perception_context)
            )
            if on_completed:
                on_completed(
                    ActionResult(
                        success=True, message="Nav reached", execution_time_sec=0.1
                    )
                )
            return True

        def send_manipulation_goal(
            self, goal, timeout_sec=60.0, on_feedback=None, on_completed=None
        ):
            self.snapshots.append(
                ("MANIP", self.node.motion_state, self.node.perception_context)
            )
            if on_completed:
                on_completed(
                    ActionResult(
                        success=True, message="Manip done", execution_time_sec=0.2
                    )
                )
            return True

        def cancel_active_goal(self):
            pass

        def destroy(self):
            pass

    dispatcher = StateTrackingDispatcher(None)
    node = ChessMissionOrchestrator(motion_client=dispatcher)
    dispatcher.node = node
    try:
        node._on_nav_ready(Bool(data=True))
        node._on_grasp_ready(Bool(data=True))

        resp = CheckMoveFeasibility.Response()
        resp.feasible = True
        resp.plan_type = CheckMoveFeasibility.Response.PLAN_DUAL_BASE
        resp.pick_base_pose.pose.position.x = 1.0
        resp.place_base_pose.pose.position.x = 2.0

        details = ChessMoveGoal(
            uci="e2e4",
            from_square="e2",
            to_square="e4",
            promotion=None,
            is_capture=False,
        )

        node.transition_to(MacroMissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MacroMissionState.CHECKING_REACHABILITY)
        node._on_feasibility_response(resp, details)

        # 5 actions dispatched: NAV to pick, MANIP pick, NAV to place, MANIP place, NAV to obs
        assert len(dispatcher.snapshots) == 5
        # 1. Nav to pick
        assert dispatcher.snapshots[0] == (
            "NAV",
            MotionExecutionState.NAV_TO_PICK,
            PerceptionContext.TF_TRACKING_AND_NAV,
        )
        # 2. Pick piece
        assert dispatcher.snapshots[1] == (
            "MANIP",
            MotionExecutionState.PICKING_PIECE,
            PerceptionContext.MANIPULATION_ACTOR,
        )
        # 3. Nav to place
        assert dispatcher.snapshots[2] == (
            "NAV",
            MotionExecutionState.NAV_TO_PLACE,
            PerceptionContext.TF_TRACKING_AND_NAV,
        )
        # 4. Place piece
        assert dispatcher.snapshots[3] == (
            "MANIP",
            MotionExecutionState.PLACING_PIECE,
            PerceptionContext.MANIPULATION_ACTOR,
        )
        # 5. Nav to observation standoff
        assert dispatcher.snapshots[4] == (
            "NAV",
            MotionExecutionState.NAV_TO_OBS,
            PerceptionContext.TF_TRACKING_AND_NAV,
        )

        # After final physical stage finishes, pipeline completes and transitions into Level 1 POST_MOVE_VERIFYING with active watchdog
        assert node.motion_state == MotionExecutionState.IDLE
        assert node.mission_state == MacroMissionState.POST_MOVE_VERIFYING
        assert node.perception_context == PerceptionContext.BOARD_STATE_SCAN
        assert node.post_move_verifier.is_active

        # When referee confirms stable and legal board, transitions to WAITING_FOR_PLAYER_MOVE
        status = ChessGameStatus()
        status.is_board_stable = True
        status.is_legal_move = True
        status.full_fen = "rnbqkbnr/pppp1ppp/8/4p3/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2"
        node._on_game_status(status)

        assert node.mission_state == MacroMissionState.WAITING_FOR_PLAYER_MOVE
        assert not node.post_move_verifier.is_active
    finally:
        node.destroy_node()


def test_dispatch_move_workflow_blocked_when_nav_unready(ros_context):
    """Verify _dispatch_move_workflow is gated on nav_ready."""
    node = ChessMissionOrchestrator()
    try:
        assert not node.is_nav_ready
        node._dispatch_move_workflow("e2e4")
        assert node.mission_state == MacroMissionState.WAITING_FOR_TF_READY
    finally:
        node.destroy_node()


def test_recovery_retains_and_resumes_interrupted_move(ros_context):
    """Verify recovery retains interrupted move and resumes execution instead of falling back to player wait."""
    dispatcher = TrackingDispatcher()
    node = ChessMissionOrchestrator(motion_client=dispatcher)
    try:
        # 1. Nav ready -> Black robot waits for player move
        node._on_nav_ready(Bool(data=True))
        node._on_grasp_ready(Bool(data=True))
        assert node.mission_state == MacroMissionState.WAITING_FOR_PLAYER_MOVE

        # 2. Dispatch a move for the robot
        node._dispatch_move_workflow("c7c5")
        assert node.current_move_details.uci == "c7c5"
        assert node.mission_state == MacroMissionState.CHECKING_REACHABILITY

        # 3. Simulate failure during reachability / execution
        node._on_feasibility_error("Reachability query timeout")
        assert node.mission_state == MacroMissionState.ERROR_FALLBACK

        # 4. Trigger recovery
        recovered = node.trigger_recovery(reason="test_recovery")
        assert recovered
        assert node.mission_state == MacroMissionState.WAITING_FOR_TF_READY
        assert node.pending_recovery_move is not None
        assert node.pending_recovery_move.uci == "c7c5"

        # 5. Nav ready confirmed (e.g. next tick of /system/nav_ready)
        node._on_nav_ready(Bool(data=True))

        # 6. Crucial check: Robot MUST NOT forget its move and MUST NOT transition to WAITING_FOR_PLAYER_MOVE!
        assert node.mission_state != MacroMissionState.WAITING_FOR_PLAYER_MOVE
        # It must have resumed execution workflow (now in CHECKING_REACHABILITY)
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

        goal = ChessMoveGoal(
            uci="e7e5", from_square="e7", to_square="e5", is_capture=False
        )
        node._current_move_details = goal
        node.trigger_recovery(reason="test")
        assert node.pending_recovery_move.uci == "e7e5"

        # New game status arrives with different FEN (e.g. board modified)
        status = ChessGameStatus()
        status.active_color = "b"
        status.full_fen = "rnbqkbnr/pppp1ppp/8/4p3/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2"
        status.is_board_stable = True
        status.game_phase = ChessGameStatus.PHASE_ROBOT_READY
        status.best_move_details.uci = "d7d5"
        node._on_game_status(status)

        # Stale recovery move e7e5 should be cleared
        assert node.pending_recovery_move is None
    finally:
        node.destroy_node()


def test_post_move_verification_watchdog_timeout_repositions(ros_context):
    """Verify that watchdog timeout during POST_MOVE_VERIFYING triggers active observation repositioning."""
    repositioned_poses = []

    class RepositionTrackingDispatcher(FakeMotionClient):
        def __init__(self):
            super().__init__(None)

        def send_navigation_goal(
            self, target_pose, timeout_sec=60.0, on_completed=None
        ):
            repositioned_poses.append(target_pose)
            if on_completed:
                on_completed(ActionResult(success=True, message="Reached viewpoint"))
            return True

    dispatcher = RepositionTrackingDispatcher()
    node = ChessMissionOrchestrator(motion_client=dispatcher)
    try:
        node._on_nav_ready(Bool(data=True))
        assert node.obs_navigator.viewpoint_index == 0

        # Simulate pipeline completion -> enters POST_MOVE_VERIFYING
        node.transition_to(MacroMissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MacroMissionState.CHECKING_REACHABILITY)
        node.transition_to(MacroMissionState.EXECUTING_MOVE_PIPELINE)
        node._on_move_pipeline_completed()
        assert node.mission_state == MacroMissionState.POST_MOVE_VERIFYING
        assert node.post_move_verifier.is_active

        # Simulate timeout firing -> reposition to next vantage point
        node._on_post_move_watchdog_timeout()
        assert len(repositioned_poses) == 1
        assert node.obs_navigator.viewpoint_index == 1
        assert node.post_move_verifier.attempts == 1
        assert node.mission_state == MacroMissionState.POST_MOVE_VERIFYING

        # Board is now confirmed stable and legal
        status = ChessGameStatus()
        status.is_board_stable = True
        status.is_legal_move = True
        status.full_fen = "rnbqkbnr/pppp1ppp/8/4p3/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2"
        node._on_game_status(status)

        assert node.mission_state == MacroMissionState.WAITING_FOR_PLAYER_MOVE
        assert not node.post_move_verifier.is_active
    finally:
        node.destroy_node()


def test_post_move_verification_max_attempts_finalizes_turn(ros_context):
    """Verify that reaching max repositioning attempts during POST_MOVE_VERIFYING finalizes turn gracefully."""
    dispatcher = FakeMotionClient(None)
    node = ChessMissionOrchestrator(motion_client=dispatcher)
    dispatcher._node = node
    try:
        node._on_nav_ready(Bool(data=True))
        node.transition_to(MacroMissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MacroMissionState.CHECKING_REACHABILITY)
        node.transition_to(MacroMissionState.EXECUTING_MOVE_PIPELINE)
        node._on_move_pipeline_completed()
        assert node.mission_state == MacroMissionState.POST_MOVE_VERIFYING

        max_attempts = node.obs_navigator.get_max_attempts(
            ObservationIntent.POST_MOVE_VERIFY
        )
        for i in range(max_attempts):
            node._on_post_move_watchdog_timeout()
            assert node.post_move_verifier.attempts == i + 1
            assert node.mission_state == MacroMissionState.POST_MOVE_VERIFYING

        # Final timeout reaches max attempts -> finalizes turn
        node._on_post_move_watchdog_timeout()
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

        # Noisy or occluded vision: stable board but move is illegal / not recognized
        status = ChessGameStatus()
        status.is_board_stable = True
        status.is_legal_move = False
        status.full_fen = "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq - 0 1"
        node._on_game_status(status)

        # Must stay in POST_MOVE_VERIFYING and watchdog must stay alive
        assert node.mission_state == MacroMissionState.POST_MOVE_VERIFYING
        assert node.post_move_verifier.is_active
    finally:
        node.destroy_node()


def test_trigger_recovery_guard_when_not_in_error_fallback(ros_context):
    """Verify trigger_recovery safely rejects when node is not in ERROR_FALLBACK."""
    node = ChessMissionOrchestrator()
    try:
        assert node.mission_state == MacroMissionState.WAITING_FOR_TF_READY
        recovered = node.trigger_recovery()
        assert recovered is False
        assert node.mission_state == MacroMissionState.WAITING_FOR_TF_READY
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


def test_feasibility_rejected_transitions_error_fallback(ros_context):
    """Verify unfeasible response triggers ERROR_FALLBACK."""
    node = ChessMissionOrchestrator()
    try:
        node._on_nav_ready(Bool(data=True))
        node._on_grasp_ready(Bool(data=True))
        node.transition_to(MacroMissionState.CHECKING_REACHABILITY)

        resp = CheckMoveFeasibility.Response()
        resp.feasible = False
        resp.message = "Arm out of workspace reach"
        goal = ChessMoveGoal(uci="e2e4", from_square="e2", to_square="e4")

        node._on_feasibility_response(resp, goal)
        assert node.mission_state == MacroMissionState.ERROR_FALLBACK
    finally:
        node.destroy_node()


def test_post_move_reposition_dispatch_failure_finalizes_turn(ros_context):
    """Verify active perception dispatch failure finalizes turn without hanging."""
    class RejectNavDisp(TrackingDispatcher):
        def send_navigation_goal(self, *args, **kwargs):
            return False

    dispatcher = RejectNavDisp()
    node = ChessMissionOrchestrator(motion_client=dispatcher)
    try:
        node._on_nav_ready(Bool(data=True))
        node.transition_to(MacroMissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MacroMissionState.CHECKING_REACHABILITY)
        node.transition_to(MacroMissionState.EXECUTING_MOVE_PIPELINE)
        node.transition_to(MacroMissionState.POST_MOVE_VERIFYING)

        node._on_post_move_watchdog_timeout()
        assert node.mission_state == MacroMissionState.WAITING_FOR_PLAYER_MOVE
    finally:
        node.destroy_node()


def test_orchestrator_subsystem_properties_and_delegations(ros_context):
    """Verify public subsystem properties and delegated helper methods."""
    node = ChessMissionOrchestrator()
    try:
        assert node.move_sequencer is not None
        assert node.perception_manager is not None
        assert node.game_status_handler is not None
        assert node.move_workflow is not None
        assert node.post_move_verifier is not None
        assert node.obs_navigator is not None
        assert node.transition_motion_to(MotionExecutionState.IDLE)
        assert node.set_perception_context(PerceptionContext.IDLE_STANDBY)
        node.publish_diagnostics_snapshot()
        node.start_post_move_watchdog()
        node._start_post_move_watchdog()
        node._stop_post_move_watchdog()
        node._on_nav_ready_confirmed(is_ready=False)
        # Test recovery with string move
        node.transition_to(MacroMissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MacroMissionState.CHECKING_REACHABILITY)
        node.transition_to(MacroMissionState.ERROR_FALLBACK)
        node._current_goal_move = "e2e4"
        recovered = node.trigger_recovery()
        assert recovered
        assert node.pending_recovery_move.uci == "e2e4"
    finally:
        node.destroy_node()


def test_main_spin_and_shutdown(monkeypatch, ros_context):
    from unittest.mock import MagicMock
    from lekiwi_orchestrator.orchestrator_node import main
    mock_executor = MagicMock()
    mock_executor.spin.side_effect = KeyboardInterrupt
    monkeypatch.setattr(
        "lekiwi_orchestrator.orchestrator_node.MultiThreadedExecutor",
        lambda: mock_executor,
    )
    monkeypatch.setattr("rclpy.init", lambda *a, **k: None)
    monkeypatch.setattr("rclpy.shutdown", lambda *a, **k: None)
    main([])




