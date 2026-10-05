# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for MovePlanBuilder and MoveSequencer."""

from __future__ import annotations

from unittest.mock import Mock

import pytest
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from lekiwi_interfaces.msg import PerceptionContext
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
    MoveStep,
    StepKind,
)
from lekiwi_orchestrator.move_sequencer import MoveSequencer
from lekiwi_orchestrator.obs_navigator import ObsNavigator
from lekiwi_orchestrator.perception_context import PerceptionContextManager
from lekiwi_orchestrator.turn_workflow import MoveWorkflow
from rclpy.node import Node


class DummyPipelineDispatcher(MotionClient):
    """Local test stub for testing move pipeline execution in isolation."""

    def check_feasibility(
        self, goal, timeout_sec=5.0, on_success=None, on_error=None
    ) -> bool:
        if on_success:
            resp = CheckMoveFeasibility.Response()
            resp.feasible = True
            on_success(resp)
        return True

    def send_navigation_goal(
        self, target_pose, timeout_sec=60.0, on_completed=None
    ) -> bool:
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


def test_build_zero_nav_stage(base_goal):
    resp = CheckMoveFeasibility.Response()
    resp.plan_type = CheckMoveFeasibility.Response.PLAN_ZERO_NAV
    resp.pick_point = Point(x=0.1, y=0.2, z=0.03)
    resp.place_point = Point(x=0.3, y=0.4, z=0.03)

    stages = MovePlanBuilder.build_steps(resp, base_goal)
    assert len(stages) == 1
    assert stages[0].name == "MOVE"
    assert stages[0].target_pose is None
    assert stages[0].from_square == "e2"
    assert stages[0].to_square == "e4"
    assert not stages[0].is_capture
    assert stages[0].motion_state == MotionExecutionState.PICKING_PIECE
    assert stages[0].nav_motion_state is None


def test_build_single_base_stage(base_goal):
    resp = CheckMoveFeasibility.Response()
    resp.plan_type = CheckMoveFeasibility.Response.PLAN_SINGLE_BASE
    resp.pick_base_pose.pose.position.x = 1.0

    stages = MovePlanBuilder.build_steps(resp, base_goal)
    assert len(stages) == 1
    assert stages[0].name == "MOVE"
    assert stages[0].target_pose is not None
    assert stages[0].target_pose.pose.position.x == 1.0
    assert stages[0].motion_state == MotionExecutionState.PICKING_PIECE
    assert stages[0].nav_motion_state == MotionExecutionState.NAV_TO_PICK


def test_build_dual_base_stages(base_goal):
    resp = CheckMoveFeasibility.Response()
    resp.plan_type = CheckMoveFeasibility.Response.PLAN_DUAL_BASE
    resp.pick_base_pose.pose.position.x = 1.0
    resp.place_base_pose.pose.position.x = 2.0

    stages = MovePlanBuilder.build_steps(resp, base_goal)
    assert len(stages) == 2
    assert stages[0].name == "PICK"
    assert stages[0].target_pose.pose.position.x == 1.0
    assert stages[0].motion_state == MotionExecutionState.PICKING_PIECE
    assert stages[0].nav_motion_state == MotionExecutionState.NAV_TO_PICK

    assert stages[1].name == "PLACE"
    assert stages[1].target_pose.pose.position.x == 2.0
    assert stages[1].motion_state == MotionExecutionState.PLACING_PIECE
    assert stages[1].nav_motion_state == MotionExecutionState.NAV_TO_PLACE


def test_build_capture_zero_nav_stages(capture_goal):
    resp = CheckMoveFeasibility.Response()
    resp.plan_type = CheckMoveFeasibility.Response.PLAN_CAPTURE_ZERO_NAV
    resp.clear_point = Point(x=0.5, y=0.5, z=0.03)

    stages = MovePlanBuilder.build_steps(resp, capture_goal)
    assert len(stages) == 2
    assert stages[0].name == "CLEAR"
    assert stages[0].is_capture is True
    assert stages[0].motion_state == MotionExecutionState.CLEARING_PIECE
    assert stages[1].name == "MOVE"
    assert stages[1].motion_state == MotionExecutionState.PICKING_PIECE


def test_build_capture_triple_base_stages(capture_goal):
    resp = CheckMoveFeasibility.Response()
    resp.plan_type = CheckMoveFeasibility.Response.PLAN_CAPTURE_TRIPLE_BASE
    resp.clear_base_pose.pose.position.x = 1.0
    resp.pick_base_pose.pose.position.x = 2.0
    resp.place_base_pose.pose.position.x = 3.0

    stages = MovePlanBuilder.build_steps(resp, capture_goal)
    assert len(stages) == 3
    assert stages[0].name == "CLEAR"
    assert stages[0].motion_state == MotionExecutionState.CLEARING_PIECE
    assert stages[1].name == "PICK"
    assert stages[1].motion_state == MotionExecutionState.PICKING_PIECE
    assert stages[2].name == "PLACE"
    assert stages[2].motion_state == MotionExecutionState.PLACING_PIECE


def test_build_stages_with_observation_pose(base_goal):
    resp = CheckMoveFeasibility.Response()
    resp.plan_type = CheckMoveFeasibility.Response.PLAN_SINGLE_BASE
    resp.pick_base_pose.pose.position.x = 1.0

    obs_pose = PoseStamped()
    obs_pose.pose.position.x = -0.65

    stages = MovePlanBuilder.build_steps(resp, base_goal, observation_pose=obs_pose)
    assert len(stages) == 2
    assert stages[0].name == "MOVE"
    assert stages[1].name == "OBSERVATION"
    assert stages[1].target_pose == obs_pose
    assert stages[1].nav_motion_state == MotionExecutionState.NAV_TO_OBS
    assert stages[1].motion_state == MotionExecutionState.IDLE


def test_chess_move_goal_from_uci():
    goal = ChessMoveGoal.from_uci_or_details("e2e4")
    assert goal is not None
    assert goal.uci == "e2e4"
    assert goal.from_square == "e2"
    assert goal.to_square == "e4"

    promo = ChessMoveGoal.from_uci_or_details("e7e8q")
    assert promo is not None
    assert promo.promotion == "q"


def test_move_pipeline_executor_lifecycle(ros_context):
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
        assert executor.motion_state == MotionExecutionState.IDLE
        assert not executor.is_active

        # Build dual base plan with observation pose
        resp = CheckMoveFeasibility.Response()
        resp.feasible = True
        resp.plan_type = CheckMoveFeasibility.Response.PLAN_DUAL_BASE
        resp.pick_base_pose.pose.position.x = 1.0
        resp.place_base_pose.pose.position.x = 2.0

        obs_pose = PoseStamped()
        obs_pose.pose.position.x = -0.65

        goal = ChessMoveGoal(uci="e2e4", from_square="e2", to_square="e4")
        stages = MovePlanBuilder.build_steps(resp, goal, observation_pose=obs_pose)

        # Run pipeline
        executor.start_pipeline(stages, resp)

        # Verified pipeline completion
        assert len(completed_events) == 1
        assert len(failed_events) == 0
        assert executor.motion_state == MotionExecutionState.IDLE
        assert not executor.is_active

        # Check recorded Level 2 transitions
        # Expected sequence: NAV_TO_PICK -> PICKING_PIECE -> NAV_TO_PLACE -> PLACING_PIECE -> NAV_TO_OBS -> IDLE
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
    node = Node("test_pipeline_cancel_node")
    dispatcher = DummyPipelineDispatcher()
    perception = PerceptionContextManager(
        node=node,
        perception_context_topic="/test/cancel/perception_context",
        set_perception_service_name="/test/cancel/set_context",
    )

    executor = MoveSequencer(
        node=node,
        dispatcher=dispatcher,
        perception=perception,
    )
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


def test_stage_pipeline_builder_custom_registry(base_goal):
    """Verify Open/Closed Principle: MovePlanBuilder can be extended with new plan types without modification."""
    custom_plan_type = 999

    def custom_builder(resp, details):
        return [
            MoveStep(
                name="CUSTOM_PROMOTION_STAGE",
                target_pose=None,
                instruction="Custom promotion handler executed",
                from_square=details.from_square,
                to_square=details.to_square,
                is_capture=False,
                pick_point=Point(),
                place_point=Point(),
                motion_state=MotionExecutionState.PICKING_PIECE,
                nav_motion_state=None,
            )
        ]

    MovePlanBuilder.register_builder(custom_plan_type, custom_builder)

    resp = CheckMoveFeasibility.Response()
    resp.plan_type = custom_plan_type

    stages = MovePlanBuilder.build_steps(resp, base_goal)
    assert len(stages) == 1
    assert stages[0].name == "CUSTOM_PROMOTION_STAGE"
    assert stages[0].instruction == "Custom promotion handler executed"


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
        grasp_readiness_provider=lambda: False,  # Grasp permanently not ready!
        pre_grasp_settle_sec=0.1,  # Short settle timeout for test
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


def test_pipeline_cancellation_during_grasp_settle(ros_context):
    """Verify that calling cancel() while waiting for grasp settle aborts without executing manipulation."""
    node = Node("test_cancel_settle_node")
    manip_goals_sent = []

    class SettleCancelDispatcher(DummyPipelineDispatcher):
        def send_manipulation_goal(
            self, goal, timeout_sec=60.0, on_feedback=None, on_completed=None
        ) -> bool:
            manip_goals_sent.append(goal)
            return super().send_manipulation_goal(
                goal, timeout_sec, on_feedback, on_completed
            )

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


def test_active_relocalization_on_grasp_unready_success(ros_context):
    """Verify that when grasp is unready after Nav2, active relocalization repositions and resumes stage."""
    node = Node("test_active_reloc_success_node")
    nav_goals = []

    class RelocTrackingDispatcher(DummyPipelineDispatcher):
        def send_navigation_goal(
            self, target_pose, timeout_sec=60.0, on_completed=None
        ) -> bool:
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
        # [pick_standoff(1.0), radial_retreat(0.65), viewpoint_1(0.65), resume_pick_standoff(1.0)]
        assert len(nav_goals) == 4
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


def test_active_relocalization_nav_action_failure_retries(ros_context):
    """Verify that when relocalization Nav2 goal fails, next viewpoint is retried."""
    node = Node("test_active_reloc_nav_fail_node")
    nav_attempts = {"count": 0}

    class FailOnceNavDispatcher(DummyPipelineDispatcher):
        def send_navigation_goal(
            self, target_pose, timeout_sec=60.0, on_completed=None
        ) -> bool:
            nav_attempts["count"] += 1
            if on_completed:
                if nav_attempts["count"] == 2:
                    on_completed(
                        ActionResult(
                            success=False, message="Nav failed to reach viewpoint 1"
                        )
                    )
                else:
                    on_completed(ActionResult(success=True, message="Nav ok"))
            return True

    dispatcher = FailOnceNavDispatcher()
    perception = PerceptionContextManager(
        node=node,
        perception_context_topic="/test/reloc_nav_fail/perception_context",
        set_perception_service_name="/test/reloc_nav_fail/set_context",
    )
    obs_nav = ObsNavigator(
        node=node,
        dispatcher=dispatcher,
        standoff_distance=0.65,
    )
    completed_events = []
    failed_events = []

    def _readiness_provider():
        return nav_attempts["count"] >= 3

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
        assert executor.motion_state == MotionExecutionState.IDLE
    finally:
        perception.destroy()
        dispatcher.destroy()
        node.destroy_node()


def test_active_relocalization_nav_dispatch_failure(ros_context):
    """Verify that when reposition_to_next_viewpoint fails to dispatch, failure is reported safely."""
    node = Node("test_active_reloc_disp_fail_node")
    nav_attempts = {"count": 0}

    class RejectNavDispatcher(DummyPipelineDispatcher):
        def send_navigation_goal(
            self, target_pose, timeout_sec=60.0, on_completed=None
        ) -> bool:
            nav_attempts["count"] += 1
            if nav_attempts["count"] == 1:
                if on_completed:
                    on_completed(ActionResult(success=True, message="Initial nav ok"))
                return True
            return False

    dispatcher = RejectNavDispatcher()
    perception = PerceptionContextManager(
        node=node,
        perception_context_topic="/test/reloc_disp_fail/perception_context",
        set_perception_service_name="/test/reloc_disp_fail/set_context",
    )
    obs_nav = ObsNavigator(node=node, dispatcher=dispatcher)
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
        assert "Failed to dispatch Nav2 goal for relocalization" in failed_events[0]
        assert executor.motion_state == MotionExecutionState.IDLE
    finally:
        perception.destroy()
        dispatcher.destroy()
        node.destroy_node()


def test_build_capture_dual_base(capture_goal):
    resp = CheckMoveFeasibility.Response()
    resp.plan_type = CheckMoveFeasibility.Response.PLAN_CAPTURE_DUAL_BASE
    resp.clear_base_pose.pose.position.x = 0.2
    resp.pick_base_pose.pose.position.x = 0.5
    resp.place_base_pose.pose.position.x = 0.5
    stages = MovePlanBuilder.build_steps(resp, capture_goal)
    assert len(stages) == 3
    assert stages[0].name == "CLEAR"
    assert stages[1].name == "PICK"
    assert stages[2].name == "PLACE"
    assert stages[1].target_pose is not None

    # Test clear_same_as_pick = True branch
    resp.pick_base_pose.pose.position.x = 0.2
    stages_same = MovePlanBuilder.build_steps(resp, capture_goal)
    assert stages_same[1].target_pose is None


def test_build_fallback_stage(base_goal):
    resp = CheckMoveFeasibility.Response()
    resp.plan_type = 99999
    resp.pick_base_pose.pose.position.x = 0.8
    stages = MovePlanBuilder.build_steps(resp, base_goal)
    assert len(stages) == 1
    assert stages[0].name == "MOVE"
    assert stages[0].target_pose is not None


def test_to_ros_goal_ik_hints():
    stage = MoveStep(
        name="TEST",
        target_pose=None,
        instruction="test",
        from_square="e2",
        to_square="e4",
        is_capture=False,
        pick_point=Point(),
        place_point=Point(),
        motion_state=MotionExecutionState.PICKING_PIECE,
    )
    resp = Mock()
    resp.pick_ik_solution = Mock()
    resp.pick_ik_solution.name = ["joint_1"]
    resp.place_ik_solution = Mock()
    resp.place_ik_solution.name = ["joint_1"]
    ros_goal = stage.to_ros_goal("chessboard_frame", resp)
    assert ros_goal.instruction == "test"
    assert hasattr(ros_goal, "pick_ik_hint")
    assert hasattr(ros_goal, "place_ik_hint")


def test_manipulation_feedback_and_failure(ros_context):
    node = Node("test_manip_fail_node")

    class FailManipDispatcher(DummyPipelineDispatcher):
        def send_manipulation_goal(
            self, goal, timeout_sec=60.0, on_feedback=None, on_completed=None
        ) -> bool:
            if on_feedback:
                fb = Mock()
                fb.feedback = Mock(current_phase="PICK", progress_percent=50.0)
                on_feedback(fb)
            if on_completed:
                on_completed(
                    ActionResult(success=False, message="Arm trajectory abort")
                )
            return True

    dispatcher = FailManipDispatcher()
    perception = PerceptionContextManager(
        node=node,
        perception_context_topic="/test/manip_fail/perception_context",
        set_perception_service_name="/test/manip_fail/set_context",
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
        resp.plan_type = CheckMoveFeasibility.Response.PLAN_ZERO_NAV
        goal = ChessMoveGoal(uci="e2e4", from_square="e2", to_square="e4")
        stages = MovePlanBuilder.build_steps(resp, goal)
        executor.start_pipeline(stages, resp)

        assert len(failed_events) == 1
        assert "Manipulation execution failed: Arm trajectory abort" in failed_events[0]
        assert executor.motion_state == MotionExecutionState.IDLE
    finally:
        perception.destroy()
        dispatcher.destroy()
        node.destroy_node()


def test_manipulation_dispatch_failure(ros_context):
    node = Node("test_manip_disp_fail_node")

    class RejectManipDispatcher(DummyPipelineDispatcher):
        def send_manipulation_goal(
            self, goal, timeout_sec=60.0, on_feedback=None, on_completed=None
        ) -> bool:
            return False

    dispatcher = RejectManipDispatcher()
    perception = PerceptionContextManager(
        node=node,
        perception_context_topic="/test/manip_disp_fail/perception_context",
        set_perception_service_name="/test/manip_disp_fail/set_context",
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
        resp.plan_type = CheckMoveFeasibility.Response.PLAN_ZERO_NAV
        goal = ChessMoveGoal(uci="e2e4", from_square="e2", to_square="e4")
        stages = MovePlanBuilder.build_steps(resp, goal)
        executor.start_pipeline(stages, resp)

        assert len(failed_events) == 1
        assert "Failed to dispatch manipulation goal" in failed_events[0]
        assert executor.motion_state == MotionExecutionState.IDLE
    finally:
        perception.destroy()
        dispatcher.destroy()
        node.destroy_node()


def test_stage_pipeline_builder_capture_single_base(capture_goal):
    resp = CheckMoveFeasibility.Response()
    resp.plan_type = CheckMoveFeasibility.Response.PLAN_CAPTURE_SINGLE_BASE
    resp.clear_base_pose.pose.position.x = 0.5
    stages = MovePlanBuilder.build_steps(resp, capture_goal)
    assert len(stages) == 2
    assert stages[0].name == "CLEAR"
    assert stages[1].name == "MOVE"


def test_nav_stage_dispatch_failure(ros_context):
    node = Node("test_nav_disp_fail_node")

    class RejectNavDisp(DummyPipelineDispatcher):
        def send_navigation_goal(
            self, target_pose, timeout_sec=60.0, on_completed=None
        ) -> bool:
            return False

    dispatcher = RejectNavDisp()
    perception = PerceptionContextManager(
        node=node,
        perception_context_topic="/test/nav_disp_fail/perception_context",
        set_perception_service_name="/test/nav_disp_fail/set_context",
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
        resp.plan_type = CheckMoveFeasibility.Response.PLAN_SINGLE_BASE
        resp.pick_base_pose.pose.position.x = 1.0
        goal = ChessMoveGoal(uci="e2e4", from_square="e2", to_square="e4")
        stages = MovePlanBuilder.build_steps(resp, goal)
        executor.start_pipeline(stages, resp)

        assert len(failed_events) == 1
        assert "Failed to dispatch Nav2 goal for stage 'MOVE'" in failed_events[0]
        assert executor.motion_state == MotionExecutionState.IDLE
        assert executor.current_stage is None
    finally:
        perception.destroy()
        dispatcher.destroy()
        node.destroy_node()


def test_nav_stage_action_failure(ros_context):
    node = Node("test_nav_act_fail_node")

    class FailNavAct(DummyPipelineDispatcher):
        def send_navigation_goal(
            self, target_pose, timeout_sec=60.0, on_completed=None
        ) -> bool:
            if on_completed:
                on_completed(ActionResult(success=False, message="Obstacle blocked"))
            return True

    dispatcher = FailNavAct()
    perception = PerceptionContextManager(
        node=node,
        perception_context_topic="/test/nav_act_fail/perception_context",
        set_perception_service_name="/test/nav_act_fail/set_context",
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
        resp.plan_type = CheckMoveFeasibility.Response.PLAN_SINGLE_BASE
        resp.pick_base_pose.pose.position.x = 1.0
        goal = ChessMoveGoal(uci="e2e4", from_square="e2", to_square="e4")
        stages = MovePlanBuilder.build_steps(resp, goal)
        executor.start_pipeline(stages, resp)

        assert len(failed_events) == 1
        assert "Nav2 navigation failed: Obstacle blocked" in failed_events[0]
        assert executor.motion_state == MotionExecutionState.IDLE
    finally:
        perception.destroy()
        dispatcher.destroy()
        node.destroy_node()


def test_capture_triple_base_observation_pose_anchored_to_place_base(ros_context):
    """
    Verify regression: in CAPTURE_TRIPLE_BASE with place_base_pose at South (y < 0),
    the computed observation pose is anchored radially to South (y < 0), NOT North (y > 0).
    """
    node = Node("test_capture_obs_anchor_node")
    dispatcher = DummyPipelineDispatcher()
    try:
        obs_nav = ObsNavigator(
            node=node,
            dispatcher=dispatcher,
            map_frame="map",
            board_frame="chessboard_frame",
            standoff_distance=0.569,
            radius_tolerance=0.08,
            angle_offsets_map={ObservationIntent.POST_MOVE_VERIFY: [0.0]},
            tf_buffer=None,
        )

        resp = CheckMoveFeasibility.Response()
        resp.feasible = True
        resp.plan_type = CheckMoveFeasibility.Response.PLAN_CAPTURE_TRIPLE_BASE

        # Clear base at North-West
        resp.clear_base_pose.header.frame_id = "chessboard_frame"
        resp.clear_base_pose.pose.position.x = -0.3
        resp.clear_base_pose.pose.position.y = 0.3

        # Pick base at Center
        resp.pick_base_pose.header.frame_id = "chessboard_frame"
        resp.pick_base_pose.pose.position.x = -0.1
        resp.pick_base_pose.pose.position.y = 0.2

        # Place base at South (e.g. e4 capture completed on South side)
        resp.place_base_pose.header.frame_id = "chessboard_frame"
        resp.place_base_pose.pose.position.x = 0.0
        resp.place_base_pose.pose.position.y = -0.4

        # Extract last base pose using MoveWorkflow static method
        last_base_pose = MoveWorkflow.extract_valid_base_pose(resp)
        assert last_base_pose is resp.place_base_pose

        # Compute observation pose
        obs_pose = obs_nav.compute_observation_pose_for_base(last_base_pose)
        assert obs_pose is not None

        # Must be on standoff circle anchored to South (y < 0)
        assert obs_pose.pose.position.y < -0.5  # Should be approx -0.569
        assert abs(obs_pose.pose.position.x) < 0.05  # approx 0.0

        details = ChessMoveGoal(
            uci="d5e4",
            from_square="d5",
            to_square="e4",
            is_capture=True,
            captured_square="e4",
        )
        steps = MovePlanBuilder.build_steps(resp, details, observation_pose=obs_pose)

        # Last step is the observation retreat step
        obs_step = steps[-1]
        assert obs_step.name in (StepKind.OBSERVATION, "OBSERVATION")
        assert obs_step.nav_motion_state == MotionExecutionState.NAV_TO_OBS
        assert obs_step.target_pose is not None
        assert obs_step.target_pose.pose.position.y < -0.5
    finally:
        node.destroy_node()
