# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for MovePlanBuilder and MoveSequencer."""

from __future__ import annotations

import math
from unittest.mock import Mock

import pytest
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from lekiwi_interfaces.srv import CheckMoveFeasibility
from lekiwi_orchestrator.fsm import MotionExecutionState
from lekiwi_orchestrator.mission_types import (
    ActionResult,
    ChessMoveGoal,
    ObservationIntent,
)
from lekiwi_orchestrator.motion_client import MotionClient
from lekiwi_orchestrator.move_planner import (
    MovePlanBuilder,
    StepKind,
)
from lekiwi_orchestrator.move_sequencer import MoveSequencer
from lekiwi_orchestrator.obs_navigator import ObsNavigator
from lekiwi_orchestrator.perception_context import PerceptionContextManager
from lekiwi_orchestrator.turn_workflow import MoveWorkflow
from rclpy.node import Node


class DummyPipelineDispatcher(MotionClient):
    """Local test stub for testing move pipeline execution in isolation."""

    def check_feasibility(self, goal, timeout_sec=5.0, on_success=None, on_error=None) -> bool:
        if on_success:
            resp = CheckMoveFeasibility.Response()
            resp.feasible = True
            on_success(resp)
        return True

    def send_navigation_goal(self, target_pose, timeout_sec=60.0, on_completed=None) -> bool:
        if on_completed:
            on_completed(ActionResult(success=True, message="Mock nav ok"))
        return True

    def send_manipulation_goal(
        self, goal, timeout_sec=60.0, on_feedback=None, on_completed=None
    ) -> bool:
        if on_completed:
            on_completed(ActionResult(success=True, message="Mock manip ok"))
        return True

    def cancel_active_goal(self) -> None:
        pass

    def destroy(self) -> None:
        pass


@pytest.fixture(scope="module")
def ros_context():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def base_goal() -> ChessMoveGoal:
    return ChessMoveGoal(
        uci="e2e4",
        from_square="e2",
        to_square="e4",
        promotion=None,
        is_capture=False,
    )


@pytest.fixture
def capture_goal() -> ChessMoveGoal:
    return ChessMoveGoal(
        uci="e4d5",
        from_square="e4",
        to_square="d5",
        promotion=None,
        is_capture=True,
        captured_square="d5",
    )


@pytest.mark.parametrize(
    ("plan_type", "has_obs_pose", "expected_stage_names"),
    [
        (CheckMoveFeasibility.Response.PLAN_ZERO_NAV, False, ["MOVE"]),
        (CheckMoveFeasibility.Response.PLAN_SINGLE_BASE, False, ["MOVE"]),
        (CheckMoveFeasibility.Response.PLAN_DUAL_BASE, False, ["PICK", "PLACE"]),
        (CheckMoveFeasibility.Response.PLAN_SINGLE_BASE, True, ["MOVE", "OBSERVATION"]),
    ],
)
def test_build_stages_matrix(base_goal, plan_type, has_obs_pose, expected_stage_names):
    """Verify MovePlanBuilder builds proper stage sequences across standard plan types."""
    resp = CheckMoveFeasibility.Response()
    resp.plan_type = plan_type
    resp.pick_base_pose.pose.position.x = 1.0
    resp.place_base_pose.pose.position.x = 2.0

    obs_pose = PoseStamped() if has_obs_pose else None
    if obs_pose:
        obs_pose.pose.position.x = -0.65

    stages = MovePlanBuilder.build_steps(resp, base_goal, observation_pose=obs_pose)
    stage_names = [s.name for s in stages]
    assert stage_names == expected_stage_names
    if has_obs_pose:
        assert stages[-1].nav_motion_state == MotionExecutionState.NAV_TO_OBS


@pytest.mark.parametrize(
    ("plan_type", "expected_names"),
    [
        (CheckMoveFeasibility.Response.PLAN_CAPTURE_ZERO_NAV, ["CLEAR", "MOVE"]),
        (CheckMoveFeasibility.Response.PLAN_CAPTURE_SINGLE_BASE, ["CLEAR", "MOVE"]),
        (CheckMoveFeasibility.Response.PLAN_CAPTURE_DUAL_BASE, ["CLEAR", "PICK", "PLACE"]),
        (CheckMoveFeasibility.Response.PLAN_CAPTURE_TRIPLE_BASE, ["CLEAR", "PICK", "PLACE"]),
    ],
)
def test_build_capture_stages_matrix(capture_goal, plan_type, expected_names):
    """Verify MovePlanBuilder correctly handles piece capture staging across all variants."""
    resp = CheckMoveFeasibility.Response()
    resp.plan_type = plan_type
    resp.clear_base_pose.pose.position.x = 0.5
    resp.pick_base_pose.pose.position.x = 1.0
    resp.place_base_pose.pose.position.x = 2.0

    stages = MovePlanBuilder.build_steps(resp, capture_goal)
    assert [s.name for s in stages] == expected_names
    assert stages[0].motion_state == MotionExecutionState.CLEARING_PIECE
    assert stages[0].is_capture is True


def test_capture_triple_base_observation_pose_anchored_to_place_base(ros_context):
    """Verify regression: in CAPTURE_TRIPLE_BASE, observation pose is anchored radially to place_base_pose."""
    node = Node("test_capture_obs_anchor_node")
    dispatcher = DummyPipelineDispatcher()
    try:
        obs_nav = ObsNavigator(
            node=node,
            dispatcher=dispatcher,
            map_frame="map",
            board_frame="chessboard_frame",
            standoff_distance=0.569,
            angle_offsets_map={ObservationIntent.POST_MOVE_VERIFY: [-math.pi / 2, 0.0, math.pi / 2]},
            tf_buffer=None,
        )

        resp = CheckMoveFeasibility.Response()
        resp.feasible = True
        resp.plan_type = CheckMoveFeasibility.Response.PLAN_CAPTURE_TRIPLE_BASE
        resp.clear_base_pose.header.frame_id = "chessboard_frame"
        resp.clear_base_pose.pose.position.x = -0.3
        resp.clear_base_pose.pose.position.y = 0.3

        resp.pick_base_pose.header.frame_id = "chessboard_frame"
        resp.pick_base_pose.pose.position.x = -0.1
        resp.pick_base_pose.pose.position.y = 0.2

        resp.place_base_pose.header.frame_id = "chessboard_frame"
        resp.place_base_pose.pose.position.x = 0.0
        resp.place_base_pose.pose.position.y = -0.4

        last_base_pose = MoveWorkflow.extract_valid_base_pose(resp)
        assert last_base_pose is resp.place_base_pose

        obs_pose = obs_nav.compute_observation_pose_for_base(last_base_pose)
        assert obs_pose is not None
        assert obs_pose.pose.position.y < -0.5
        assert abs(obs_pose.pose.position.x) < 0.05

        details = ChessMoveGoal(
            uci="d5e4",
            from_square="d5",
            to_square="e4",
            is_capture=True,
            captured_square="e4",
        )
        steps = MovePlanBuilder.build_steps(resp, details, observation_pose=obs_pose)
        obs_step = steps[-1]
        assert obs_step.name in (StepKind.OBSERVATION, "OBSERVATION")
        assert obs_step.nav_motion_state == MotionExecutionState.NAV_TO_OBS
        assert obs_step.target_pose.pose.position.y < -0.5
    finally:
        node.destroy_node()


def test_chess_move_goal_from_uci():
    """Verify parsing standard and promotion UCI strings into ChessMoveGoal."""
    goal = ChessMoveGoal.from_uci_or_details("e2e4")
    assert goal is not None
    assert goal.uci == "e2e4"
    assert (goal.from_square, goal.to_square) == ("e2", "e4")

    promo = ChessMoveGoal.from_uci_or_details("e7e8q")
    assert promo is not None
    assert promo.promotion == "q"


def test_move_pipeline_executor_lifecycle(ros_context):
    """Verify end-to-end execution of a multi-stage move pipeline and Level 2 transitions."""
    node = Node("test_pipeline_executor_node")
    dispatcher = DummyPipelineDispatcher()
    perception = PerceptionContextManager(
        node=node,
        perception_context_topic="/test/pipeline/perception_context",
        set_perception_service_name="/test/pipeline/set_context",
    )

    completed_events = []
    failed_events = []
    motion_transitions = []

    executor = MoveSequencer(
        node=node,
        dispatcher=dispatcher,
        perception=perception,
        board_frame="chessboard_frame",
        action_timeout_sec=5.0,
        on_pipeline_completed=lambda: completed_events.append(True),
        on_pipeline_failed=lambda err: failed_events.append(err),
        on_motion_state_changed=lambda st: motion_transitions.append(st),
    )

    try:
        resp = CheckMoveFeasibility.Response()
        resp.feasible = True
        resp.plan_type = CheckMoveFeasibility.Response.PLAN_DUAL_BASE
        resp.pick_base_pose.pose.position.x = 1.0
        resp.place_base_pose.pose.position.x = 2.0

        obs_pose = PoseStamped()
        obs_pose.pose.position.x = -0.65

        goal = ChessMoveGoal(uci="e2e4", from_square="e2", to_square="e4")
        stages = MovePlanBuilder.build_steps(resp, goal, observation_pose=obs_pose)

        executor.start_pipeline(stages, resp)

        assert len(completed_events) == 1
        assert len(failed_events) == 0
        assert executor.motion_state == MotionExecutionState.IDLE
        assert not executor.is_active

        expected_seq = [
            MotionExecutionState.NAV_TO_PICK,
            MotionExecutionState.PICKING_PIECE,
            MotionExecutionState.NAV_TO_PLACE,
            MotionExecutionState.PLACING_PIECE,
            MotionExecutionState.NAV_TO_OBS,
            MotionExecutionState.IDLE,
        ]
        assert motion_transitions == expected_seq
    finally:
        perception.destroy()
        dispatcher.destroy()
        node.destroy_node()


def test_move_pipeline_executor_cancellation(ros_context):
    """Verify cancelling an active pipeline immediately resets motion state and halts execution."""
    node = Node("test_pipeline_cancel_node")
    dispatcher = DummyPipelineDispatcher()
    perception = PerceptionContextManager(
        node=node,
        perception_context_topic="/test/cancel/perception_context",
        set_perception_service_name="/test/cancel/set_context",
    )

    executor = MoveSequencer(node=node, dispatcher=dispatcher, perception=perception)
    try:
        resp = CheckMoveFeasibility.Response()
        resp.plan_type = CheckMoveFeasibility.Response.PLAN_SINGLE_BASE
        resp.pick_base_pose.pose.position.x = 1.0
        goal = ChessMoveGoal(uci="e2e4", from_square="e2", to_square="e4")
        stages = MovePlanBuilder.build_steps(resp, goal)

        executor.start_pipeline(stages, resp)
        executor.cancel()

        assert executor.motion_state == MotionExecutionState.IDLE
        assert not executor.is_active
    finally:
        perception.destroy()
        dispatcher.destroy()
        node.destroy_node()


def test_pipeline_cancellation_during_grasp_settle(ros_context):
    """Verify that calling cancel() while waiting for grasp settle aborts without executing manipulation."""
    node = Node("test_cancel_settle_node")
    manip_goals_sent = []

    class SettleCancelDispatcher(DummyPipelineDispatcher):
        def send_manipulation_goal(self, goal, timeout_sec=60.0, on_feedback=None, on_completed=None) -> bool:
            manip_goals_sent.append(goal)
            return super().send_manipulation_goal(goal, timeout_sec, on_feedback, on_completed)

    dispatcher = SettleCancelDispatcher()
    perception = PerceptionContextManager(
        node=node,
        perception_context_topic="/test/cancel_settle/perception_context",
        set_perception_service_name="/test/cancel_settle/set_context",
    )
    failed_events = []
    completed_events = []
    executor = None

    def _readiness_provider():
        if executor is not None:
            executor.cancel()
        return False

    executor = MoveSequencer(
        node=node,
        dispatcher=dispatcher,
        perception=perception,
        grasp_readiness_provider=_readiness_provider,
        pre_grasp_settle_sec=1.0,
        on_pipeline_completed=lambda: completed_events.append(True),
        on_pipeline_failed=lambda err: failed_events.append(err),
    )

    try:
        resp = CheckMoveFeasibility.Response()
        resp.feasible = True
        resp.plan_type = CheckMoveFeasibility.Response.PLAN_ZERO_NAV
        goal = ChessMoveGoal(uci="e2e4", from_square="e2", to_square="e4")
        stages = MovePlanBuilder.build_steps(resp, goal)

        executor.start_pipeline(stages, resp)

        assert len(manip_goals_sent) == 0
        assert len(completed_events) == 0
        assert len(failed_events) == 0
        assert executor.motion_state == MotionExecutionState.IDLE
        assert not executor.is_active
    finally:
        perception.destroy()
        dispatcher.destroy()
        node.destroy_node()


def test_move_pipeline_grasp_readiness_timeout_fails_safely(ros_context):
    """Verify that when grasp readiness times out, failure is safely reported without crashing or hanging."""
    node = Node("test_grasp_readiness_node")
    dispatcher = DummyPipelineDispatcher()
    perception = PerceptionContextManager(
        node=node,
        perception_context_topic="/test/grasp/perception_context",
        set_perception_service_name="/test/grasp/set_context",
    )
    failed_events = []

    executor = MoveSequencer(
        node=node,
        dispatcher=dispatcher,
        perception=perception,
        grasp_readiness_provider=lambda: False,
        pre_grasp_settle_sec=0.1,
        on_pipeline_failed=lambda err: failed_events.append(err),
    )

    try:
        resp = CheckMoveFeasibility.Response()
        resp.feasible = True
        resp.plan_type = CheckMoveFeasibility.Response.PLAN_ZERO_NAV
        goal = ChessMoveGoal(uci="e2e4", from_square="e2", to_square="e4")
        stages = MovePlanBuilder.build_steps(resp, goal)

        executor.start_pipeline(stages, resp)

        assert len(failed_events) == 1
        assert "Grasp readiness check failed" in failed_events[0]
        assert executor.motion_state == MotionExecutionState.IDLE
        assert not executor.is_active
    finally:
        perception.destroy()
        dispatcher.destroy()
        node.destroy_node()


def test_move_pipeline_grasp_readiness_settle_success(ros_context):
    """Verify that when grasp is initially False but becomes True within settle window, execution succeeds."""
    node = Node("test_grasp_settle_node")
    dispatcher = DummyPipelineDispatcher()
    perception = PerceptionContextManager(
        node=node,
        perception_context_topic="/test/grasp_settle/perception_context",
        set_perception_service_name="/test/grasp_settle/set_context",
    )

    completed_events = []
    failed_events = []
    calls = {"count": 0}

    def _flapping_provider():
        calls["count"] += 1
        return calls["count"] > 1

    executor = MoveSequencer(
        node=node,
        dispatcher=dispatcher,
        perception=perception,
        grasp_readiness_provider=_flapping_provider,
        pre_grasp_settle_sec=0.2,
        on_pipeline_completed=lambda: completed_events.append(True),
        on_pipeline_failed=lambda err: failed_events.append(err),
    )

    try:
        resp = CheckMoveFeasibility.Response()
        resp.feasible = True
        resp.plan_type = CheckMoveFeasibility.Response.PLAN_ZERO_NAV
        goal = ChessMoveGoal(uci="e2e4", from_square="e2", to_square="e4")
        stages = MovePlanBuilder.build_steps(resp, goal)

        executor.start_pipeline(stages, resp)

        assert len(failed_events) == 0
        assert len(completed_events) == 1
        assert executor.motion_state == MotionExecutionState.IDLE
    finally:
        perception.destroy()
        dispatcher.destroy()
        node.destroy_node()


def test_active_relocalization_on_grasp_unready_success(ros_context):
    """Verify that when grasp is unready after Nav2, active relocalization repositions and resumes stage."""
    node = Node("test_active_reloc_success_node")
    nav_goals = []

    class RelocTrackingDispatcher(DummyPipelineDispatcher):
        def send_navigation_goal(self, target_pose, timeout_sec=60.0, on_completed=None) -> bool:
            nav_goals.append(target_pose)
            if on_completed:
                on_completed(ActionResult(success=True, message="Nav ok"))
            return True

    dispatcher = RelocTrackingDispatcher()
    perception = PerceptionContextManager(
        node=node,
        perception_context_topic="/test/reloc_succ/perception_context",
        set_perception_service_name="/test/reloc_succ/set_context",
    )
    obs_nav = ObsNavigator(
        node=node,
        dispatcher=dispatcher,
        standoff_distance=0.65,
    )
    completed_events = []
    failed_events = []

    def _readiness_provider():
        return len(nav_goals) >= 2

    executor = MoveSequencer(
        node=node,
        dispatcher=dispatcher,
        perception=perception,
        grasp_readiness_provider=_readiness_provider,
        pre_grasp_settle_sec=0.01,
        observation_navigator=obs_nav,
        on_pipeline_completed=lambda: completed_events.append(True),
        on_pipeline_failed=lambda err: failed_events.append(err),
    )

    try:
        resp = CheckMoveFeasibility.Response()
        resp.feasible = True
        resp.plan_type = CheckMoveFeasibility.Response.PLAN_SINGLE_BASE
        resp.pick_base_pose.pose.position.x = 1.0
        resp.pick_base_pose.header.frame_id = "map"
        goal = ChessMoveGoal(uci="e2e4", from_square="e2", to_square="e4")
        stages = MovePlanBuilder.build_steps(resp, goal)

        executor.start_pipeline(stages, resp)

        assert len(failed_events) == 0
        assert len(completed_events) == 1
        # [pick_standoff(1.0), direct_viewpoint(0.65), resume_pick_standoff(1.0)]
        assert len(nav_goals) == 3
        assert executor.motion_state == MotionExecutionState.IDLE
    finally:
        perception.destroy()
        dispatcher.destroy()
        node.destroy_node()


def test_active_relocalization_on_grasp_unready_exhaustion(ros_context):
    """Verify that when grasp readiness repeatedly fails across all viewpoints, pipeline fails gracefully."""
    node = Node("test_active_reloc_exhaust_node")
    dispatcher = DummyPipelineDispatcher()
    perception = PerceptionContextManager(
        node=node,
        perception_context_topic="/test/reloc_exh/perception_context",
        set_perception_service_name="/test/reloc_exh/set_context",
    )
    obs_nav = ObsNavigator(
        node=node,
        dispatcher=dispatcher,
        standoff_distance=0.65,
    )
    failed_events = []

    executor = MoveSequencer(
        node=node,
        dispatcher=dispatcher,
        perception=perception,
        grasp_readiness_provider=lambda: False,
        pre_grasp_settle_sec=0.01,
        observation_navigator=obs_nav,
        on_pipeline_failed=lambda err: failed_events.append(err),
    )

    try:
        resp = CheckMoveFeasibility.Response()
        resp.feasible = True
        resp.plan_type = CheckMoveFeasibility.Response.PLAN_SINGLE_BASE
        resp.pick_base_pose.pose.position.x = 1.0
        resp.pick_base_pose.header.frame_id = "map"
        goal = ChessMoveGoal(uci="e2e4", from_square="e2", to_square="e4")
        stages = MovePlanBuilder.build_steps(resp, goal)

        executor.start_pipeline(stages, resp)

        assert len(failed_events) == 1
        assert "exhausted all 3 relocalization viewpoints" in failed_events[0]
        assert executor.motion_state == MotionExecutionState.IDLE
    finally:
        perception.destroy()
        dispatcher.destroy()
        node.destroy_node()


@pytest.mark.parametrize(
    ("fail_type", "expected_err_snippet"),
    [
        ("NAV_DISPATCH", "Failed to dispatch Nav2 goal for stage 'MOVE'"),
        ("NAV_ACTION", "Nav2 navigation failed: Obstacle blocked"),
        ("MANIP_DISPATCH", "Failed to dispatch manipulation goal"),
        ("MANIP_ACTION", "Manipulation execution failed: Trajectory aborted"),
    ],
)
def test_stage_pipeline_dispatch_and_execution_failures(ros_context, fail_type, expected_err_snippet):
    """Verify safe failure handling across Nav2 and Manipulation dispatch and execution errors."""
    node = Node(f"test_fail_{fail_type.lower()}_node")

    class FaultInjectionDispatcher(DummyPipelineDispatcher):
        def send_navigation_goal(self, target_pose, timeout_sec=60.0, on_completed=None) -> bool:
            if fail_type == "NAV_DISPATCH":
                return False
            if fail_type == "NAV_ACTION":
                if on_completed:
                    on_completed(ActionResult(success=False, message="Obstacle blocked"))
                return True
            return super().send_navigation_goal(target_pose, timeout_sec, on_completed)

        def send_manipulation_goal(
            self, goal, timeout_sec=60.0, on_feedback=None, on_completed=None
        ) -> bool:
            if fail_type == "MANIP_DISPATCH":
                return False
            if fail_type == "MANIP_ACTION":
                if on_completed:
                    on_completed(ActionResult(success=False, message="Trajectory aborted"))
                return True
            return super().send_manipulation_goal(goal, timeout_sec, on_feedback, on_completed)

    dispatcher = FaultInjectionDispatcher()
    perception = PerceptionContextManager(
        node=node,
        perception_context_topic=f"/test/fail_{fail_type}/perception_context",
        set_perception_service_name=f"/test/fail_{fail_type}/set_context",
    )
    failed_events = []
    executor = MoveSequencer(
        node=node,
        dispatcher=dispatcher,
        perception=perception,
        on_pipeline_failed=lambda err: failed_events.append(err),
    )

    try:
        resp = CheckMoveFeasibility.Response()
        resp.feasible = True
        plan = CheckMoveFeasibility.Response.PLAN_SINGLE_BASE if "NAV" in fail_type else CheckMoveFeasibility.Response.PLAN_ZERO_NAV
        resp.plan_type = plan
        resp.pick_base_pose.pose.position.x = 1.0

        goal = ChessMoveGoal(uci="e2e4", from_square="e2", to_square="e4")
        stages = MovePlanBuilder.build_steps(resp, goal)
        executor.start_pipeline(stages, resp)

        assert len(failed_events) == 1
        assert expected_err_snippet in failed_events[0]
        assert executor.motion_state == MotionExecutionState.IDLE
    finally:
        perception.destroy()
        dispatcher.destroy()
        node.destroy_node()
