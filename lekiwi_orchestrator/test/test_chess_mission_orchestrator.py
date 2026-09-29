# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for ChessMissionOrchestrator logic, 2-level HFSM, PerceptionContext, and self-healing."""

from __future__ import annotations

import math

import pytest
import rclpy
from lekiwi_orchestrator.chess_mission_orchestrator import (
    ChessMissionOrchestrator,
)
from lekiwi_orchestrator.fsm import (
    MissionState,
    MotionExecutionState,
)
from lekiwi_orchestrator.motion_dispatcher import (
    ActionDispatcherInterface,
    ActionResult,
)
from lekiwi_orchestrator.move_pipeline import (
    ChessMoveGoal,
)
from rclpy.parameter import Parameter
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from lekiwi_interfaces.msg import (
    ChessGameStatus,
    ChessMoveDetails,
    PerceptionContext,
)
from lekiwi_interfaces.srv import CheckMoveFeasibility


class TrackingDispatcher(ActionDispatcherInterface):
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
        assert node.mission_state == MissionState.WAITING_FOR_TF_READY
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
        assert node.mission_state == MissionState.WAITING_FOR_PLAYER_MOVE
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
        assert node.mission_state == MissionState.EVALUATING_BEST_MOVE
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

        assert node.mission_state == MissionState.GAME_OVER
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

        assert node.mission_state == MissionState.GAME_OVER
        assert node.motion_state == MotionExecutionState.IDLE
        assert node.perception_context == PerceptionContext.IDLE_STANDBY
    finally:
        node.destroy_node()


def test_illegal_state_transition_guard(ros_context):
    node = ChessMissionOrchestrator()
    try:
        assert node.mission_state == MissionState.WAITING_FOR_TF_READY
        success = node.transition_to(MissionState.EXECUTING_MOVE_PIPELINE)
        assert not success
        assert node.mission_state == MissionState.WAITING_FOR_TF_READY
    finally:
        node.destroy_node()


def test_workflow_dispatch_zero_nav_simulation(ros_context):
    node = ChessMissionOrchestrator(
        parameter_overrides=[Parameter("skip_navigation", value=True)]
    )
    try:
        node._on_nav_ready(Bool(data=True))
        node._on_grasp_ready(Bool(data=True))
        assert node.mission_state == MissionState.WAITING_FOR_PLAYER_MOVE

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

        node.transition_to(MissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MissionState.CHECKING_REACHABILITY)
        node._on_feasibility_response(resp, details)

        assert node.mission_state == MissionState.WAITING_FOR_PLAYER_MOVE
        assert node.motion_state == MotionExecutionState.IDLE
        assert node.perception_context == PerceptionContext.BOARD_STATE_SCAN
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
        assert node.mission_state == MissionState.WAITING_FOR_TF_READY
        node._on_nav_ready(Bool(data=True))
        assert node.mission_state == MissionState.WAITING_FOR_PLAYER_MOVE

        # 2. Un-gated state: WAITING_FOR_PLAYER_MOVE
        node._on_nav_ready(Bool(data=False))
        assert node.mission_state == MissionState.WAITING_FOR_PLAYER_MOVE

        # Re-enable Nav ready
        node._on_nav_ready(Bool(data=True))
        assert node.mission_state == MissionState.WAITING_FOR_PLAYER_MOVE

        # 3. Critical state: CHECKING_REACHABILITY
        node.transition_to(MissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MissionState.CHECKING_REACHABILITY)
        assert node.mission_state == MissionState.CHECKING_REACHABILITY
        node._on_nav_ready(Bool(data=False))
        assert node.mission_state == MissionState.ERROR_FALLBACK

        # 4. Recover from ERROR_FALLBACK to WAITING_FOR_TF_READY
        node.trigger_recovery()
        assert node.mission_state == MissionState.WAITING_FOR_TF_READY

        node._on_nav_ready(Bool(data=True))
        node.transition_to(MissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MissionState.CHECKING_REACHABILITY)
        node.transition_to(MissionState.EXECUTING_MOVE_PIPELINE)
        assert node.mission_state == MissionState.EXECUTING_MOVE_PIPELINE

        # 5. Un-gated state: EXECUTING_MOVE_PIPELINE
        node._on_nav_ready(Bool(data=False))
        assert node.mission_state == MissionState.EXECUTING_MOVE_PIPELINE
    finally:
        node.destroy_node()


def test_feasibility_timeout_watchdog(ros_context):
    node = ChessMissionOrchestrator()
    try:
        node._on_nav_ready(Bool(data=True))
        node.transition_to(MissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MissionState.CHECKING_REACHABILITY)
        assert node.mission_state == MissionState.CHECKING_REACHABILITY

        node._on_feasibility_error("Workspace feasibility query timed out")
        assert node.mission_state == MissionState.ERROR_FALLBACK
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
    node = ChessMissionOrchestrator(action_dispatcher=mock_dispatcher)
    try:
        node._on_nav_ready(Bool(data=True))
        node.transition_to(MissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MissionState.CHECKING_REACHABILITY)
        node.transition_to(MissionState.EXECUTING_MOVE_PIPELINE)
        assert node.mission_state == MissionState.EXECUTING_MOVE_PIPELINE

        # When action execution fails or times out, orchestrator transitions to ERROR_FALLBACK
        node._on_move_pipeline_failed("Action execution timed out")
        assert node.mission_state == MissionState.ERROR_FALLBACK
        assert node.motion_state == MotionExecutionState.IDLE

        # Triggering recovery cancels active goals on dispatcher and resets pipeline
        node.trigger_recovery()
        assert mock_dispatcher.cancelled
        assert node.mission_state == MissionState.WAITING_FOR_TF_READY
        assert node.motion_state == MotionExecutionState.IDLE
    finally:
        node.destroy_node()


def test_self_healing_recover_service(ros_context):
    """Verify operator service /orchestrator/recover restores system from ERROR_FALLBACK."""
    node = ChessMissionOrchestrator()
    try:
        node._on_nav_ready(Bool(data=True))
        node.transition_to(MissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MissionState.CHECKING_REACHABILITY)
        node.transition_to(MissionState.ERROR_FALLBACK)
        assert node.mission_state == MissionState.ERROR_FALLBACK

        req = Trigger.Request()
        resp = Trigger.Response()
        result_resp = node._handle_recover_service(req, resp)

        assert result_resp.success
        assert node.mission_state == MissionState.WAITING_FOR_TF_READY
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
        node.transition_to(MissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MissionState.CHECKING_REACHABILITY)
        node.transition_to(MissionState.ERROR_FALLBACK)
        assert node.mission_state == MissionState.ERROR_FALLBACK
        assert node.health_monitor.recovery_attempts == 1

        # Fire auto recovery timer directly via health monitor
        node.health_monitor._on_auto_recovery_timer_fired()
        assert node.mission_state == MissionState.WAITING_FOR_TF_READY
        assert node.motion_state == MotionExecutionState.IDLE
    finally:
        node.destroy_node()


def test_action_dispatcher_dependency_injection(ros_context):
    """Verify custom ActionDispatcherInterface implementation can be cleanly injected."""

    class CustomDispatcher(ActionDispatcherInterface):
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
    node = ChessMissionOrchestrator(action_dispatcher=custom_dispatcher)
    try:
        assert node.dispatcher is custom_dispatcher
    finally:
        node.destroy_node()


def test_workflow_dispatch_capture_single_base_sequence(ros_context):
    """Verify capture single base executes Clear (capture=True) then Move (capture=False)."""

    class TrackingDispatcher(ActionDispatcherInterface):
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

    dispatcher = TrackingDispatcher()
    node = ChessMissionOrchestrator(action_dispatcher=dispatcher)
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

        node.transition_to(MissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MissionState.CHECKING_REACHABILITY)
        node._on_feasibility_response(resp, details)

        # 1 Nav goal to common standoff
        assert len(dispatcher.nav_goals) == 1
        # 2 Manipulation goals: Clear d5 (is_capture=True), then Move e4->d5 (is_capture=False)
        assert len(dispatcher.manip_goals) == 2
        assert dispatcher.manip_goals[0].is_capture is True
        assert dispatcher.manip_goals[0].from_square == "d5"
        assert dispatcher.manip_goals[1].is_capture is False
        assert dispatcher.manip_goals[1].from_square == "e4"
        assert dispatcher.manip_goals[1].to_square == "d5"
        assert node.mission_state == MissionState.WAITING_FOR_PLAYER_MOVE
        assert node.motion_state == MotionExecutionState.IDLE
    finally:
        node.destroy_node()


def test_workflow_dispatch_capture_triple_base_sequence(ros_context):
    """Verify capture triple base executes Clear -> Pick -> Place across 3 base standoffs."""

    class TrackingDispatcher(ActionDispatcherInterface):
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

    dispatcher = TrackingDispatcher()
    node = ChessMissionOrchestrator(action_dispatcher=dispatcher)
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

        node.transition_to(MissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MissionState.CHECKING_REACHABILITY)
        node._on_feasibility_response(resp, details)

        # 3 Nav goals: clear_base -> pick_base -> place_base
        assert len(dispatcher.nav_goals) == 3
        assert dispatcher.nav_goals[0].pose.position.x == 1.0
        assert dispatcher.nav_goals[1].pose.position.x == 2.0
        assert dispatcher.nav_goals[2].pose.position.x == 1.0

        # 3 Manipulation goals: Clear h8 -> Pick a1 -> Place h8
        assert len(dispatcher.manip_goals) == 3
        assert dispatcher.manip_goals[0].is_capture is True
        assert dispatcher.manip_goals[0].from_square == "h8"
        assert dispatcher.manip_goals[1].is_capture is False
        assert dispatcher.manip_goals[1].from_square == "a1"
        assert dispatcher.manip_goals[2].is_capture is False
        assert dispatcher.manip_goals[2].to_square == "h8"

        assert node.mission_state == MissionState.WAITING_FOR_PLAYER_MOVE
        assert node.motion_state == MotionExecutionState.IDLE
    finally:
        node.destroy_node()


def test_2level_hierarchical_fsm_micro_stage_transitions(ros_context):
    """Verify Level 2 MotionExecutionState and PerceptionContext transitions during stage progression."""

    class StateTrackingDispatcher(ActionDispatcherInterface):
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
    node = ChessMissionOrchestrator(action_dispatcher=dispatcher)
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

        node.transition_to(MissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MissionState.CHECKING_REACHABILITY)
        node._on_feasibility_response(resp, details)

        # 4 actions dispatched: NAV to pick, MANIP pick, NAV to place, MANIP place
        assert len(dispatcher.snapshots) == 4
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

        # After final stage finishes, transitions through POST_MOVE_VERIFYING then to IDLE
        assert node.motion_state == MotionExecutionState.IDLE
        assert node.mission_state == MissionState.WAITING_FOR_PLAYER_MOVE
        assert node.perception_context == PerceptionContext.BOARD_STATE_SCAN
    finally:
        node.destroy_node()


def test_reposition_to_next_observation_viewpoint(ros_context):
    class NavTrackingDispatcher(TrackingDispatcher):
        def __init__(self):
            super().__init__()
            self.dispatched_poses = []

        def send_navigation_goal(
            self, target_pose, timeout_sec=60.0, on_completed=None
        ):
            self.dispatched_poses.append(target_pose)
            if on_completed:
                on_completed(
                    ActionResult(
                        success=True, message="Reached standoff", execution_time_sec=0.1
                    )
                )
            return True

    nav_dispatcher = NavTrackingDispatcher()
    node = ChessMissionOrchestrator(action_dispatcher=nav_dispatcher)
    try:
        node._on_nav_ready(Bool(data=True))
        assert node.perception_context == PerceptionContext.BOARD_STATE_SCAN

        # Trigger active repositioning
        success = node.reposition_to_next_observation_viewpoint(
            board_x=1.0, board_y=1.0
        )
        assert success
        assert len(nav_dispatcher.dispatched_poses) == 1
        # Target pose standoff ~ 0.65m away from (1.0, 1.0)
        pose = nav_dispatcher.dispatched_poses[0]
        dx = pose.pose.position.x - 1.0
        dy = pose.pose.position.y - 1.0
        assert (
            abs(math.hypot(dx, dy) - node.config.observation_standoff_distance) < 1e-4
        )

        # Context returns to BOARD_STATE_SCAN after reaching observation viewpoint
        assert node.perception_context == PerceptionContext.BOARD_STATE_SCAN
    finally:
        node.destroy_node()


def test_dispatch_move_workflow_blocked_when_nav_unready(ros_context):
    """Verify _dispatch_move_workflow is gated on nav_ready."""
    node = ChessMissionOrchestrator()
    try:
        assert not node.is_nav_ready
        node._dispatch_move_workflow("e2e4")
        assert node.mission_state == MissionState.WAITING_FOR_TF_READY
    finally:
        node.destroy_node()
