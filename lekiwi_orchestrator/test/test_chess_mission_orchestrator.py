# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for ChessMissionOrchestrator logic, self-healing, and state machine."""

from __future__ import annotations

import time

import pytest
import rclpy
from lekiwi_orchestrator.action_dispatcher import (
    ActionDispatcherInterface,
    SimulatedActionDispatcher,
)
from lekiwi_orchestrator.chess_mission_orchestrator import (
    ChessMissionOrchestrator,
    ChessMoveGoal,
)
from lekiwi_orchestrator.fsm import MissionState
from rclpy.parameter import Parameter
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from lekiwi_interfaces.msg import CameraMode, ChessGameStatus, ChessMoveDetails
from lekiwi_interfaces.srv import CheckMoveFeasibility


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
        assert node.camera_mode == CameraMode.STANDBY
        assert not node.is_tf_ready
    finally:
        node.destroy_node()


def test_tf_readiness_transition(ros_context):
    node = ChessMissionOrchestrator()
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
    node = ChessMissionOrchestrator(
        parameter_overrides=[Parameter("robot_color", value="w")]
    )
    try:
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
    node = ChessMissionOrchestrator()
    try:
        node._on_tf_ready(Bool(data=True))

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
    node = ChessMissionOrchestrator()
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
    node = ChessMissionOrchestrator()
    try:
        assert node.mission_state == MissionState.WAITING_FOR_TF_READY
        success = node.transition_to(MissionState.EXECUTING_MANIPULATION)
        assert not success
        assert node.mission_state == MissionState.WAITING_FOR_TF_READY
    finally:
        node.destroy_node()


def test_workflow_dispatch_zero_nav_simulation(ros_context):
    node = ChessMissionOrchestrator()
    try:
        node._on_tf_ready(Bool(data=True))
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

        class _MockFuture:
            def result(self):
                return resp

        node.transition_to(MissionState.CHECKING_REACHABILITY)
        node._on_feasibility_response(_MockFuture(), details)

        assert node.mission_state == MissionState.WAITING_FOR_PLAYER_MOVE
        assert node.camera_mode == CameraMode.CHESS_THINKING
    finally:
        node.destroy_node()


def test_readiness_lease_expires_without_ros_time(ros_context):
    """A stopped gatekeeper cannot leave cached readiness valid indefinitely."""
    node = ChessMissionOrchestrator()
    try:
        node._on_tf_ready(Bool(data=True))
        node._last_readiness_heartbeat = (
            time.monotonic() - node.config.readiness_timeout_sec - 0.1
        )
        assert not node.is_tf_ready
        node._expire_readiness()
        assert not node._tf_ready
    finally:
        node.destroy_node()


def test_workflow_dispatch_uci_with_motion_response(ros_context):
    node = ChessMissionOrchestrator()
    try:
        node._on_tf_ready(Bool(data=True))
        assert node.mission_state == MissionState.WAITING_FOR_PLAYER_MOVE

        resp = CheckMoveFeasibility.Response()
        resp.feasible = True
        resp.plan_type = CheckMoveFeasibility.Response.PLAN_ZERO_NAV
        resp.message = "Feasible in lekiwi_motion"

        details = ChessMoveGoal(
            uci="e2e4",
            from_square="e2",
            to_square="e4",
            promotion=None,
            is_capture=False,
        )

        class _MockFuture:
            def result(self):
                return resp

        node.transition_to(MissionState.CHECKING_REACHABILITY)
        node._on_feasibility_response(_MockFuture(), details)

        assert node.mission_state == MissionState.WAITING_FOR_PLAYER_MOVE
    finally:
        node.destroy_node()


def test_move_details_dispatching(ros_context):
    """Verify ChessMoveDetails message populates ChessMoveGoal correctly."""
    node = ChessMissionOrchestrator()
    try:
        node._on_tf_ready(Bool(data=True))

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


def test_selective_tf_gating_critical_vs_ungated(ros_context):
    node = ChessMissionOrchestrator()
    try:
        # 1. Critical state: WAITING_FOR_TF_READY
        assert node.mission_state == MissionState.WAITING_FOR_TF_READY
        node._on_tf_ready(Bool(data=True))
        assert node.mission_state == MissionState.WAITING_FOR_PLAYER_MOVE

        # 2. Un-gated state: WAITING_FOR_PLAYER_MOVE
        node._on_tf_ready(Bool(data=False))
        assert node.mission_state == MissionState.WAITING_FOR_PLAYER_MOVE

        # Re-enable TF ready
        node._on_tf_ready(Bool(data=True))
        assert node.mission_state == MissionState.WAITING_FOR_PLAYER_MOVE

        # Advance to CHECKING_REACHABILITY (critical state)
        node.transition_to(MissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MissionState.CHECKING_REACHABILITY)
        assert node.mission_state == MissionState.CHECKING_REACHABILITY

        # 3. Critical state: CHECKING_REACHABILITY
        node._on_tf_ready(Bool(data=False))
        assert node.mission_state == MissionState.ERROR_FALLBACK

        # 4. Recover from ERROR_FALLBACK to WAITING_FOR_TF_READY
        node.trigger_recovery()
        assert node.mission_state == MissionState.WAITING_FOR_TF_READY

        node._on_tf_ready(Bool(data=True))
        node.transition_to(MissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MissionState.CHECKING_REACHABILITY)
        node.transition_to(MissionState.NAVIGATING_TO_STANDOFF)
        assert node.mission_state == MissionState.NAVIGATING_TO_STANDOFF

        # 5. Un-gated state: NAVIGATING_TO_STANDOFF
        node._on_tf_ready(Bool(data=False))
        assert node.mission_state == MissionState.NAVIGATING_TO_STANDOFF

        # 6. Un-gated state: EXECUTING_MANIPULATION
        node.transition_to(MissionState.EXECUTING_MANIPULATION)
        assert node.mission_state == MissionState.EXECUTING_MANIPULATION
        node._on_tf_ready(Bool(data=False))
        assert node.mission_state == MissionState.EXECUTING_MANIPULATION
    finally:
        node.destroy_node()


def test_workflow_dispatch_dual_base_simulation(ros_context):
    node = ChessMissionOrchestrator(
        parameter_overrides=[Parameter("skip_navigation", value=True)]
    )
    try:
        node._on_tf_ready(Bool(data=True))
        assert node.mission_state == MissionState.WAITING_FOR_PLAYER_MOVE

        resp = CheckMoveFeasibility.Response()
        resp.feasible = True
        resp.plan_type = CheckMoveFeasibility.Response.PLAN_DUAL_BASE
        resp.message = "PLAN_DUAL_BASE test"
        resp.pick_base_pose.pose.position.x = 1.0
        resp.place_base_pose.pose.position.x = 2.0

        details = ChessMoveGoal(
            uci="e2e4",
            from_square="e2",
            to_square="e4",
            promotion=None,
            is_capture=False,
        )

        class _MockFuture:
            def result(self):
                return resp

        node.transition_to(MissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MissionState.CHECKING_REACHABILITY)

        node._on_feasibility_response(_MockFuture(), details)

        assert node.mission_state == MissionState.WAITING_FOR_PLAYER_MOVE
        assert node.camera_mode == CameraMode.CHESS_THINKING
    finally:
        node.destroy_node()


def test_manipulation_safe_attributes_no_attribute_error(ros_context):
    node = ChessMissionOrchestrator(
        parameter_overrides=[Parameter("skip_navigation", value=True)]
    )
    try:
        node._on_tf_ready(Bool(data=True))
        node.transition_to(MissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MissionState.CHECKING_REACHABILITY)

        resp = CheckMoveFeasibility.Response()
        resp.feasible = True
        resp.plan_type = CheckMoveFeasibility.Response.PLAN_ZERO_NAV
        assert not hasattr(resp, "pick_ik_solution")
        assert not hasattr(resp, "place_ik_solution")

        details = ChessMoveGoal(
            uci="g1f3",
            from_square="g1",
            to_square="f3",
            promotion=None,
            is_capture=False,
        )

        node._execute_manipulation_step(details, resp)
        assert node.mission_state == MissionState.WAITING_FOR_PLAYER_MOVE
    finally:
        node.destroy_node()


def test_feasibility_timeout_watchdog(ros_context):
    node = ChessMissionOrchestrator()
    try:
        node._on_tf_ready(Bool(data=True))
        node.transition_to(MissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MissionState.CHECKING_REACHABILITY)
        assert node.mission_state == MissionState.CHECKING_REACHABILITY

        node._on_feasibility_timeout()
        assert node.mission_state == MissionState.ERROR_FALLBACK
    finally:
        node.destroy_node()


def test_action_timeout_watchdog_cancels_goal(ros_context):
    class _MockDispatcher(SimulatedActionDispatcher):
        def __init__(self, node):
            super().__init__(node)
            self.cancelled = False

        def cancel_active_goal(self):
            self.cancelled = True

    node = ChessMissionOrchestrator()
    mock_dispatcher = _MockDispatcher(node)
    node._dispatcher = mock_dispatcher
    try:
        node._on_tf_ready(Bool(data=True))
        node.transition_to(MissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MissionState.CHECKING_REACHABILITY)
        node.transition_to(MissionState.NAVIGATING_TO_STANDOFF)
        assert node.mission_state == MissionState.NAVIGATING_TO_STANDOFF

        node._on_action_timeout()
        assert mock_dispatcher.cancelled
        assert node.mission_state == MissionState.ERROR_FALLBACK
    finally:
        node.destroy_node()


def test_self_healing_recover_service(ros_context):
    """Verify operator service /orchestrator/recover restores system from ERROR_FALLBACK."""
    node = ChessMissionOrchestrator()
    try:
        node._on_tf_ready(Bool(data=True))
        node.transition_to(MissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MissionState.CHECKING_REACHABILITY)
        node.transition_to(MissionState.ERROR_FALLBACK)
        assert node.mission_state == MissionState.ERROR_FALLBACK

        req = Trigger.Request()
        resp = Trigger.Response()
        result_resp = node._handle_recover_service(req, resp)

        assert result_resp.success
        assert node.mission_state == MissionState.WAITING_FOR_TF_READY
        assert node.camera_mode == CameraMode.STANDBY
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
        node._on_tf_ready(Bool(data=True))
        node.transition_to(MissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MissionState.CHECKING_REACHABILITY)
        node.transition_to(MissionState.ERROR_FALLBACK)
        assert node.mission_state == MissionState.ERROR_FALLBACK
        assert node._recovery_attempts == 1

        # Fire auto recovery timer directly
        node._on_auto_recovery_timer()
        assert node.mission_state == MissionState.WAITING_FOR_TF_READY
    finally:
        node.destroy_node()


def test_action_dispatcher_dependency_injection(ros_context):
    """Verify custom ActionDispatcherInterface implementation can be cleanly injected."""

    class CustomDispatcher(ActionDispatcherInterface):
        def __init__(self):
            self.nav_called = False

        def send_navigation_goal(self, target_pose, on_accepted, on_completed):
            self.nav_called = True
            return True

        def send_manipulation_goal(self, goal, on_feedback, on_accepted, on_completed):
            return True

        def cancel_active_goal(self):
            pass

        def destroy(self):
            pass

    custom_dispatcher = CustomDispatcher()
    node = ChessMissionOrchestrator(action_dispatcher=custom_dispatcher)
    try:
        assert node._dispatcher is custom_dispatcher
    finally:
        node.destroy_node()


def test_workflow_dispatch_capture_single_base_sequence(ros_context):
    """Verify capture single base executes Clear (capture=True) then Move (capture=False)."""

    class TrackingDispatcher(ActionDispatcherInterface):
        def __init__(self):
            self.nav_goals = []
            self.manip_goals = []

        def send_navigation_goal(self, target_pose, on_accepted, on_completed):
            self.nav_goals.append(target_pose)
            on_accepted(object())
            on_completed(None)
            return True

        def send_manipulation_goal(self, goal, on_feedback, on_accepted, on_completed):
            self.manip_goals.append(goal)
            on_accepted(object())

            class _MockRes:
                success = True
                execution_time_sec = 0.5
                message = "Done"

            on_completed(_MockRes())
            return True

        def cancel_active_goal(self):
            pass

        def destroy(self):
            pass

    dispatcher = TrackingDispatcher()
    node = ChessMissionOrchestrator(action_dispatcher=dispatcher)
    try:
        node._on_tf_ready(Bool(data=True))

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

        class _MockFuture:
            def result(self):
                return resp

        node.transition_to(MissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MissionState.CHECKING_REACHABILITY)
        node._on_feasibility_response(_MockFuture(), details)

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
    finally:
        node.destroy_node()


def test_workflow_dispatch_capture_triple_base_sequence(ros_context):
    """Verify capture triple base executes Clear -> Pick -> Place across 3 base standoffs."""

    class TrackingDispatcher(ActionDispatcherInterface):
        def __init__(self):
            self.nav_goals = []
            self.manip_goals = []

        def send_navigation_goal(self, target_pose, on_accepted, on_completed):
            self.nav_goals.append(target_pose)
            on_accepted(object())
            on_completed(None)
            return True

        def send_manipulation_goal(self, goal, on_feedback, on_accepted, on_completed):
            self.manip_goals.append(goal)
            on_accepted(object())

            class _MockRes:
                success = True
                execution_time_sec = 0.5
                message = "Done"

            on_completed(_MockRes())
            return True

        def cancel_active_goal(self):
            pass

        def destroy(self):
            pass

    dispatcher = TrackingDispatcher()
    node = ChessMissionOrchestrator(action_dispatcher=dispatcher)
    try:
        node._on_tf_ready(Bool(data=True))

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

        class _MockFuture:
            def result(self):
                return resp

        node.transition_to(MissionState.EVALUATING_BEST_MOVE)
        node.transition_to(MissionState.CHECKING_REACHABILITY)
        node._on_feasibility_response(_MockFuture(), details)

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
    finally:
        node.destroy_node()
