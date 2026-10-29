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
from lekiwi_orchestrator.core.fsm import MotionExecutionState
from lekiwi_orchestrator.core.mission_types import (
    ActionResult,
    ChessMoveGoal,
    ObservationIntent,
)
from lekiwi_orchestrator.execution.motion_client import FakeMotionClient, MotionClient
from lekiwi_orchestrator.execution.move_sequencer import MoveSequencer
from lekiwi_orchestrator.execution.turn_workflow import MoveWorkflow
from lekiwi_orchestrator.planning.move_planner import (
    MovePlanBuilder,
    MoveStep,
    StepKind,
)
from lekiwi_orchestrator.planning.obs_navigator import ObsNavigator
from lekiwi_orchestrator.supervision.perception_context import PerceptionContextManager
from rclpy.node import Node


@pytest.fixture(scope="module")
def ros_context():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def create_mock_tf_buffer():
    """Create a mock tf_buffer transforming board to base_footprint."""
    mock_tf = Mock()
    mock_stamped = Mock()
    mock_stamped.transform.translation.x = 0.40
    mock_stamped.transform.translation.y = 0.18
    mock_stamped.transform.translation.z = 0.01
    mock_stamped.transform.rotation.x = 0.0
    mock_stamped.transform.rotation.y = 0.0
    mock_stamped.transform.rotation.z = 0.0
    mock_stamped.transform.rotation.w = 1.0
    mock_tf.lookup_transform.return_value = mock_stamped
    return mock_tf


@pytest.fixture
def base_goal() -> ChessMoveGoal:
    return ChessMoveGoal(
        uci="e2e4",
        promotion_piece="",
        is_capture=False,
    )


@pytest.fixture
def capture_goal() -> ChessMoveGoal:
    return ChessMoveGoal(
        uci="e4d5",
        promotion_piece="",
        is_capture=True,
    )


@pytest.mark.parametrize(
    ("is_capture", "plan_type", "has_obs_pose", "expected_stage_names"),
    [
        (False, CheckMoveFeasibility.Response.PLAN_ZERO_NAV, False, ["MOVE"]),
        (False, CheckMoveFeasibility.Response.PLAN_SINGLE_BASE, False, ["MOVE"]),
        (False, CheckMoveFeasibility.Response.PLAN_DUAL_BASE, False, ["PICK", "PLACE"]),
        (False, CheckMoveFeasibility.Response.PLAN_SINGLE_BASE, True, ["MOVE", "OBSERVATION"]),
        (True, CheckMoveFeasibility.Response.PLAN_CAPTURE_ZERO_NAV, False, ["CLEAR", "MOVE"]),
        (True, CheckMoveFeasibility.Response.PLAN_CAPTURE_SINGLE_BASE, False, ["CLEAR", "MOVE"]),
        (True, CheckMoveFeasibility.Response.PLAN_CAPTURE_DUAL_BASE, False, ["CLEAR", "PICK", "PLACE"]),
        (True, CheckMoveFeasibility.Response.PLAN_CAPTURE_TRIPLE_BASE, False, ["CLEAR", "PICK", "PLACE"]),
    ],
)
def test_build_stages_matrix(base_goal, capture_goal, is_capture, plan_type, has_obs_pose, expected_stage_names):
    """Verify MovePlanBuilder builds proper stage sequences across standard and capture plan types."""
    resp = CheckMoveFeasibility.Response()
    resp.plan_type = plan_type
    resp.clear_base_pose.pose.position.x = 0.5
    resp.pick_base_pose.pose.position.x = 1.0
    resp.place_base_pose.pose.position.x = 2.0
    if is_capture:
        resp.message = "CAPTURE_TEST;bin=clear_left"

    obs_pose = PoseStamped() if has_obs_pose else None
    if obs_pose:
        obs_pose.pose.position.x = -0.65

    goal = capture_goal if is_capture else base_goal
    stages = MovePlanBuilder.build_steps(resp, goal, observation_pose=obs_pose)
    stage_names = [s.name for s in stages]
    assert stage_names == expected_stage_names
    if has_obs_pose:
        assert stages[-1].nav_motion_state == MotionExecutionState.NAV_TO_OBS
    if is_capture:
        assert stages[0].motion_state == MotionExecutionState.CLEARING_PIECE
        assert stages[0].is_capture is True


def test_capture_triple_base_observation_pose_anchored_to_place_base(ros_context):
    """Verify regression: in CAPTURE_TRIPLE_BASE, observation pose is anchored radially to place_base_pose."""
    node = Node("test_capture_obs_anchor_node")
    dispatcher = FakeMotionClient()
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
        resp.message = "CAPTURE_TRIPLE_BASE;bin=clear_right"
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
            is_capture=True,
        )
        steps = MovePlanBuilder.build_steps(resp, details, observation_pose=obs_pose)
        obs_step = steps[-1]
        assert obs_step.name == StepKind.OBSERVATION
        assert obs_step.nav_motion_state == MotionExecutionState.NAV_TO_OBS
        assert obs_step.target_pose.pose.position.y < -0.5
    finally:
        node.destroy_node()


def test_move_pipeline_executor_lifecycle(ros_context):
    """Verify end-to-end execution of a multi-stage move pipeline and Level 2 transitions."""
    node = Node("test_pipeline_executor_node")
    dispatcher = FakeMotionClient()
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
        tf_buffer=create_mock_tf_buffer(),
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

        goal = ChessMoveGoal(uci="e2e4")
        stages = MovePlanBuilder.build_steps(resp, goal, observation_pose=obs_pose)

        executor.start_pipeline(stages)

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


def test_move_pipeline_cancellation_scenarios(ros_context):
    """Verify cancelling pipeline immediately resets motion state (active execution & grasp settle)."""
    node = Node("test_pipeline_cancel_node")
    manip_goals_sent = []

    class SettleCancelDispatcher(FakeMotionClient):
        def send_manipulation_goal(self, goal, timeout_sec=60.0, on_feedback=None, on_completed=None) -> bool:
            manip_goals_sent.append(goal)
            return super().send_manipulation_goal(goal, timeout_sec, on_feedback, on_completed)

    dispatcher = SettleCancelDispatcher()
    perception = PerceptionContextManager(
        node=node,
        perception_context_topic="/test/cancel/perception_context",
        set_perception_service_name="/test/cancel/set_context",
    )

    try:
        # Scenario 1: Immediate explicit cancellation during active stages
        executor = MoveSequencer(node=node, dispatcher=dispatcher, perception=perception)
        resp = CheckMoveFeasibility.Response()
        resp.plan_type = CheckMoveFeasibility.Response.PLAN_SINGLE_BASE
        resp.pick_base_pose.pose.position.x = 1.0
        goal = ChessMoveGoal(uci="e2e4")
        stages = MovePlanBuilder.build_steps(resp, goal)

        executor.start_pipeline(stages)
        executor.cancel()
        assert executor.motion_state == MotionExecutionState.IDLE
        assert not executor.is_active

        # Scenario 2: Cancellation while waiting for grasp settle aborts without executing manipulation
        executor_settle = None

        def _readiness_provider():
            if executor_settle is not None:
                executor_settle.cancel()
            return False

        failed_events = []
        completed_events = []
        executor_settle = MoveSequencer(
            node=node,
            dispatcher=dispatcher,
            perception=perception,
            grasp_readiness_provider=_readiness_provider,
            chassis_settle_timeout_sec=1.0,
            on_pipeline_completed=lambda: completed_events.append(True),
            on_pipeline_failed=lambda err: failed_events.append(err),
        )
        resp_zero = CheckMoveFeasibility.Response(feasible=True, plan_type=CheckMoveFeasibility.Response.PLAN_ZERO_NAV)
        stages_zero = MovePlanBuilder.build_steps(resp_zero, goal)
        executor_settle.start_pipeline(stages_zero)

        assert len(manip_goals_sent) == 0
        assert len(completed_events) == 0
        assert len(failed_events) == 0
        assert executor_settle.motion_state == MotionExecutionState.IDLE
        assert not executor_settle.is_active
    finally:
        perception.destroy()
        dispatcher.destroy()
        node.destroy_node()


def test_move_pipeline_grasp_readiness_scenarios(ros_context):
    """Verify grasp readiness handling: timeout failure vs settle window recovery."""
    node = Node("test_grasp_readiness_node")
    dispatcher = FakeMotionClient()
    perception = PerceptionContextManager(
        node=node,
        perception_context_topic="/test/grasp/perception_context",
        set_perception_service_name="/test/grasp/set_context",
    )

    try:
        resp = CheckMoveFeasibility.Response(feasible=True, plan_type=CheckMoveFeasibility.Response.PLAN_ZERO_NAV)
        goal = ChessMoveGoal(uci="e2e4")
        stages = MovePlanBuilder.build_steps(resp, goal)

        # Scenario 1: Readiness times out safely
        failed_events_to = []
        executor_timeout = MoveSequencer(
            node=node,
            dispatcher=dispatcher,
            perception=perception,
            grasp_readiness_provider=lambda: False,
            chassis_settle_timeout_sec=0.1,
            on_pipeline_failed=lambda err: failed_events_to.append(err),
        )
        executor_timeout.start_pipeline(stages)
        assert len(failed_events_to) == 1
        assert "Grasp readiness check failed" in failed_events_to[0]
        assert executor_timeout.motion_state == MotionExecutionState.IDLE

        # Scenario 2: Settle window success when provider transitions from False to True
        completed_events = []
        failed_events_settle = []
        calls = {"count": 0}

        def _flapping_provider():
            calls["count"] += 1
            return calls["count"] > 1

        executor_settle = MoveSequencer(
            node=node,
            dispatcher=dispatcher,
            perception=perception,
            grasp_readiness_provider=_flapping_provider,
            chassis_settle_timeout_sec=0.2,
            tf_buffer=create_mock_tf_buffer(),
            on_pipeline_completed=lambda: completed_events.append(True),
            on_pipeline_failed=lambda err: failed_events_settle.append(err),
        )
        executor_settle.start_pipeline(stages)
        assert len(failed_events_settle) == 0
        assert len(completed_events) == 1
        assert executor_settle.motion_state == MotionExecutionState.IDLE
    finally:
        perception.destroy()
        dispatcher.destroy()
        node.destroy_node()


def test_active_relocalization_on_grasp_unready_success(ros_context):
    """Verify that when grasp is unready after Nav2, active relocalization repositions and resumes stage."""
    node = Node("test_active_reloc_success_node")
    nav_goals = []

    class RelocTrackingDispatcher(FakeMotionClient):
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
        chassis_settle_timeout_sec=0.01,
        observation_navigator=obs_nav,
        tf_buffer=create_mock_tf_buffer(),
        on_pipeline_completed=lambda: completed_events.append(True),
        on_pipeline_failed=lambda err: failed_events.append(err),
    )

    try:
        resp = CheckMoveFeasibility.Response()
        resp.feasible = True
        resp.plan_type = CheckMoveFeasibility.Response.PLAN_SINGLE_BASE
        resp.pick_base_pose.pose.position.x = 1.0
        resp.pick_base_pose.header.frame_id = "map"
        goal = ChessMoveGoal(uci="e2e4")
        stages = MovePlanBuilder.build_steps(resp, goal)

        executor.start_pipeline(stages)

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
    dispatcher = FakeMotionClient()
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
        chassis_settle_timeout_sec=0.01,
        observation_navigator=obs_nav,
        on_pipeline_failed=lambda err: failed_events.append(err),
    )

    try:
        resp = CheckMoveFeasibility.Response()
        resp.feasible = True
        resp.plan_type = CheckMoveFeasibility.Response.PLAN_SINGLE_BASE
        resp.pick_base_pose.pose.position.x = 1.0
        resp.pick_base_pose.header.frame_id = "map"
        goal = ChessMoveGoal(uci="e2e4")
        stages = MovePlanBuilder.build_steps(resp, goal)

        executor.start_pipeline(stages)

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

    class FaultInjectionDispatcher(FakeMotionClient):
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
        tf_buffer=create_mock_tf_buffer(),
        on_pipeline_failed=lambda err: failed_events.append(err),
    )

    try:
        resp = CheckMoveFeasibility.Response()
        resp.feasible = True
        plan = CheckMoveFeasibility.Response.PLAN_SINGLE_BASE if "NAV" in fail_type else CheckMoveFeasibility.Response.PLAN_ZERO_NAV
        resp.plan_type = plan
        resp.pick_base_pose.pose.position.x = 1.0

        goal = ChessMoveGoal(uci="e2e4")
        stages = MovePlanBuilder.build_steps(resp, goal)
        executor.start_pipeline(stages)

        assert len(failed_events) == 1
        assert expected_err_snippet in failed_events[0]
        assert executor.motion_state == MotionExecutionState.IDLE
    finally:
        perception.destroy()
        dispatcher.destroy()
        node.destroy_node()


def test_observation_step_waits_for_chassis_settle(ros_context):
    """Verify that MoveSequencer waits for chassis settling before advancing to IDLE after observation retreat."""
    node = Node("test_obs_settle_node")
    dispatcher = FakeMotionClient()
    perception = PerceptionContextManager(
        node=node,
        perception_context_topic="/test/obs_settle/perception_context",
        set_perception_service_name="/test/obs_settle/set_context",
    )

    completed_events = []
    settle_polls = {"count": 0}

    def _settle_provider():
        settle_polls["count"] += 1
        return settle_polls["count"] >= 3

    executor = MoveSequencer(
        node=node,
        dispatcher=dispatcher,
        perception=perception,
        chassis_settle_timeout_sec=0.5,
        chassis_settle_provider=_settle_provider,
        tf_buffer=create_mock_tf_buffer(),
        on_pipeline_completed=lambda: completed_events.append(True),
    )

    try:
        resp = CheckMoveFeasibility.Response()
        resp.feasible = True
        resp.plan_type = CheckMoveFeasibility.Response.PLAN_ZERO_NAV
        goal = ChessMoveGoal(uci="e2e4")
        obs_pose = PoseStamped()
        obs_pose.header.frame_id = "map"

        stages = MovePlanBuilder.build_steps(resp, goal, observation_pose=obs_pose)
        assert any(s.name == StepKind.OBSERVATION for s in stages)

        executor.start_pipeline(stages)

        assert len(completed_events) == 1
        assert settle_polls["count"] >= 3
        assert executor.motion_state == MotionExecutionState.IDLE
    finally:
        perception.destroy()
        dispatcher.destroy()
        node.destroy_node()


def test_move_sequencer_transforms_and_bin_side_mapping(ros_context):
    """Verify coordinate transformation to base_footprint and bin side mapping in manipulation goals."""
    from lekiwi_interfaces.action import ExecuteChessMove
    from lekiwi_orchestrator.planning.move_planner import MoveStep, StepKind

    node = Node("test_transform_bin_node")
    dispatcher = FakeMotionClient()
    perception = PerceptionContextManager(node=node)

    mock_tf = Mock()
    mock_tf_stamped = Mock()
    mock_tf_stamped.transform.translation.x = 0.4027
    mock_tf_stamped.transform.translation.y = 0.1862
    mock_tf_stamped.transform.translation.z = 0.0060
    mock_tf_stamped.transform.rotation.x = 0.0
    mock_tf_stamped.transform.rotation.y = 0.0
    mock_tf_stamped.transform.rotation.z = 0.3877
    mock_tf_stamped.transform.rotation.w = 0.9218
    mock_tf.lookup_transform.return_value = mock_tf_stamped

    executor = MoveSequencer(
        node=node,
        dispatcher=dispatcher,
        perception=perception,
        board_frame="chessboard_frame",
        tf_buffer=mock_tf,
    )

    try:
        # 1. Transform accuracy to base_footprint
        stage = MoveStep(
            name=StepKind.MOVE,
            target_pose=None,
            instruction="Pick c7, place c5",
            from_square="c7",
            to_square="c5",
            is_capture=False,
            pick_point=Point(x=-0.07125, y=0.11875, z=0.025),
            place_point=Point(x=-0.07125, y=0.02375, z=0.025),
            motion_state=MotionExecutionState.PICKING_PIECE,
        )
        goal = executor._build_manipulation_goal(stage)
        expected_src = [0.2680, 0.2183, 0.0310]
        expected_tgt = [0.3359, 0.1519, 0.0310]
        assert len(goal.environment_state) == 6
        for i in range(3):
            assert math.isclose(goal.environment_state[i], expected_src[i], abs_tol=1e-3)
            assert math.isclose(goal.environment_state[3 + i], expected_tgt[i], abs_tol=1e-3)

        # 2. Clear bin side mapping: left vs right
        stage_left = MoveStep(
            name=StepKind.CLEAR,
            target_pose=None,
            instruction="Clear white pawn to left bin [d4]",
            from_square="d4",
            to_square="",
            is_capture=True,
            pick_point=Point(),
            place_point=Point(),
            clear_bin_side="left",
        )
        assert executor._build_manipulation_goal(stage_left).operation == ExecuteChessMove.Goal.OP_CLEAR_LEFT

        stage_right = MoveStep(
            name=StepKind.CLEAR,
            target_pose=None,
            instruction="Clear white pawn to right bin [d4]",
            from_square="d4",
            to_square="",
            is_capture=True,
            pick_point=Point(),
            place_point=Point(),
            clear_bin_side="right",
        )
        assert executor._build_manipulation_goal(stage_right).operation == ExecuteChessMove.Goal.OP_CLEAR_RIGHT
    finally:
        perception.destroy()
        dispatcher.destroy()
        node.destroy_node()


def test_move_sequencer_fails_fast_on_missing_or_failed_tf(ros_context):
    """Verify Fail-Fast behavior: MoveSequencer aborts without dispatching goals when TF is missing or lookup fails."""
    node = Node("test_fail_fast_tf_node")
    dispatcher = FakeMotionClient()
    perception = PerceptionContextManager(
        node=node,
        perception_context_topic="/test/fail_fast/perception_context",
        set_perception_service_name="/test/fail_fast/set_context",
    )
    failed_events = []

    from lekiwi_orchestrator.planning.move_planner import MoveStep, StepKind

    stage = MoveStep(
        name=StepKind.MOVE,
        target_pose=None,
        instruction="Pick c7, place c5",
        from_square="c7",
        to_square="c5",
        is_capture=False,
        pick_point=Point(x=-0.07125, y=0.11875, z=0.025),
        place_point=Point(x=-0.07125, y=0.02375, z=0.025),
        motion_state=MotionExecutionState.PICKING_PIECE,
    )

    # Case 1: tf_buffer is None -> must fail fast and not dispatch
    executor_no_tf = MoveSequencer(
        node=node,
        dispatcher=dispatcher,
        perception=perception,
        tf_buffer=None,
        on_pipeline_failed=lambda err: failed_events.append(err),
    )

    executor_no_tf._execute_manip_stage(stage)
    assert len(failed_events) == 1
    assert "tf_buffer was not provided" in failed_events[0]

    # Case 2: lookup_transform raises exception -> must fail fast and not dispatch
    mock_tf_error = Mock()
    mock_tf_error.lookup_transform.side_effect = RuntimeError("TF lookup timeout")

    executor_lookup_fail = MoveSequencer(
        node=node,
        dispatcher=dispatcher,
        perception=perception,
        tf_buffer=mock_tf_error,
        on_pipeline_failed=lambda err: failed_events.append(err),
    )

    executor_lookup_fail._execute_manip_stage(stage)
    assert len(failed_events) == 2
    assert "TF lookup timeout" in failed_events[1]

    perception.destroy()
    dispatcher.destroy()
    node.destroy_node()


def test_smolvla_instructions_and_force_pick_place():
    """Verify SmolVLA instruction formatting across primitives and force_pick_place conversions."""
    # 1. Primitives: MOVE, CLEAR, PICK, PLACE
    goal_move = ChessMoveGoal(uci="c7c5", piece_type="pawn", piece_color="b")
    resp = CheckMoveFeasibility.Response(plan_type=CheckMoveFeasibility.Response.PLAN_ZERO_NAV)
    steps = MovePlanBuilder.build_steps(resp, goal_move)
    assert steps[0].instruction == "Move black pawn to target"

    goal_clear = ChessMoveGoal(
        uci="e4d5",
        piece_type="pawn:knight",
        piece_color="b",
        is_capture=True,
        is_en_passant=True,
        clear_bin_side="left",
    )
    resp_cap = CheckMoveFeasibility.Response(plan_type=CheckMoveFeasibility.Response.PLAN_CAPTURE_ZERO_NAV)
    steps_cap = MovePlanBuilder.build_steps(resp_cap, goal_clear)
    assert steps_cap[0].instruction == "Clear white knight to left bin"
    assert steps_cap[1].instruction == "Move black pawn to target"

    # 2. force_pick_place conversions on zero-nav and single-base
    goal_pp = ChessMoveGoal(uci="e2e4", piece_type="pawn", piece_color="w", force_pick_place=True)
    steps_zero = MovePlanBuilder.build_steps(resp, goal_pp)
    assert [s.name for s in steps_zero] == ["PICK", "PLACE"]
    assert steps_zero[0].instruction == "Pick white pawn"
    assert steps_zero[1].instruction == "Place white pawn"
    assert steps_zero[1].requires_pre_grasp_gate is False

    resp_single = CheckMoveFeasibility.Response(plan_type=CheckMoveFeasibility.Response.PLAN_SINGLE_BASE)
    resp_single.pick_base_pose.pose.position.x = 1.0
    steps_single = MovePlanBuilder.build_steps(resp_single, goal_pp)
    assert [s.name for s in steps_single] == ["PICK", "PLACE"]
    assert steps_single[0].target_pose == resp_single.pick_base_pose
    assert steps_single[1].target_pose is None


def test_move_sequencer_bypass_record_pauses_and_advances(ros_context):
    """Verify that stage with bypass_record=True triggers pause and advances without physical action."""
    node = Node("test_bypass_record_node")
    dispatcher = FakeMotionClient()
    perception = PerceptionContextManager(node)
    mock_tf = create_mock_tf_buffer()

    completed = []
    try:
        sequencer = MoveSequencer(
            node=node,
            dispatcher=dispatcher,
            perception=perception,
            tf_buffer=mock_tf,
            bypass_pause_sec=0.05,  # Short pause for fast test execution
            on_pipeline_completed=lambda: completed.append(True),
        )

        bypassed_step = MoveStep(
            name=StepKind.MOVE,
            target_pose=None,
            instruction="Move white pawn to target [e2 to e4]",
            from_square="e2",
            to_square="e4",
            is_capture=False,
            pick_point=Point(),
            place_point=Point(),
            bypass_record=True,
        )

        sequencer.start_pipeline([bypassed_step])
        # Pipeline should advance and complete after short pause without dispatching any hardware actions
        assert len(completed) == 1
        assert len(dispatcher.manip_goals) == 0
        assert len(dispatcher.nav_goals) == 0
    finally:
        perception.destroy()
        dispatcher.destroy()
        node.destroy_node()


def test_castling_workflow_two_feasibility_checks(ros_context):
    """Verify that castling triggers two sequential feasibility checks and builds King + Rook steps."""
    from unittest.mock import MagicMock
    from lekiwi_orchestrator.core.fsm import MacroMissionState
    from lekiwi_orchestrator.core.parameters import OrchestratorParameters

    node = Node("test_castling_workflow_node")
    dispatcher = FakeMotionClient()
    perception = PerceptionContextManager(node)
    mock_tf = create_mock_tf_buffer()

    host_mock = MagicMock()
    host_mock.is_nav_ready = True
    host_mock.mission_state = MacroMissionState.WAITING_FOR_NEXT_MOVE
    host_mock.transition_to.return_value = True

    obs_nav = ObsNavigator(
        node=node,
        dispatcher=dispatcher,
        map_frame="map",
        board_frame="chessboard_frame",
        standoff_distance=0.569,
        angle_offsets_map={ObservationIntent.POST_MOVE_VERIFY: [0.0]},
        tf_buffer=None,
    )

    sequencer = MoveSequencer(
        node=node,
        dispatcher=dispatcher,
        perception=perception,
        tf_buffer=mock_tf,
    )

    config = OrchestratorParameters(node)

    workflow = MoveWorkflow(
        node=node,
        config=config,
        host=host_mock,
        dispatcher_provider=lambda: dispatcher,
        observation_nav=obs_nav,
        pipeline_executor=sequencer,
        perception=perception,
    )

    castling_goal = ChessMoveGoal(
        uci="e1g1",
        piece_type="king",
        piece_color="w",
        is_castling=True,
    )
    assert castling_goal.is_castling is True

    ok, msg = workflow.dispatch_collection_task(castling_goal)
    assert ok is True

    # 1st feasibility check was dispatched for King
    assert dispatcher.last_feasibility_callback is not None
    cb_king = dispatcher.last_feasibility_callback

    resp_king = CheckMoveFeasibility.Response()
    resp_king.feasible = True
    resp_king.plan_type = CheckMoveFeasibility.Response.PLAN_SINGLE_BASE
    resp_king.pick_base_pose.pose.position.x = 0.5
    resp_king.pick_point.x = 0.02
    resp_king.place_point.x = 0.12

    # Provide King feasibility response -> triggers 2nd feasibility check for Rook
    cb_king(resp_king)

    assert dispatcher.last_feasibility_callback is not None
    cb_rook = dispatcher.last_feasibility_callback

    resp_rook = CheckMoveFeasibility.Response()
    resp_rook.feasible = True
    resp_rook.plan_type = CheckMoveFeasibility.Response.PLAN_ZERO_NAV
    resp_rook.pick_point.x = 0.17
    resp_rook.place_point.x = 0.07

    # Provide Rook feasibility response -> completes assembly and starts pipeline
    cb_rook(resp_rook)

    assert host_mock.transition_to.call_count >= 2
    perception.destroy()
    dispatcher.destroy()
    node.destroy_node()


def test_branching_to_observation_on_failure_and_explicit_call(ros_context):
    """Verify branching to NAV_TO_OBS for active relocalization on Nav2 failure and explicit API call."""
    node = Node("test_branch_obs_node")
    nav_goals = []
    attempt_count = 0
    auto_complete = True

    class DualBranchDispatcher(FakeMotionClient):
        def send_navigation_goal(self, target_pose, timeout_sec=60.0, on_completed=None) -> bool:
            nonlocal attempt_count
            attempt_count += 1
            nav_goals.append(target_pose)
            if auto_complete and on_completed:
                if attempt_count == 1:
                    on_completed(ActionResult(success=False, message="Costmap collision predicted"))
                else:
                    on_completed(ActionResult(success=True, message="Nav ok"))
            return True

    dispatcher = DualBranchDispatcher()
    perception = PerceptionContextManager(node=node)
    obs_nav = ObsNavigator(node=node, dispatcher=dispatcher, standoff_distance=0.65)
    completed_events = []
    failed_events = []
    motion_transitions = []

    executor = MoveSequencer(
        node=node,
        dispatcher=dispatcher,
        perception=perception,
        grasp_readiness_provider=lambda: True,
        chassis_settle_timeout_sec=0.01,
        observation_navigator=obs_nav,
        tf_buffer=create_mock_tf_buffer(),
        on_pipeline_completed=lambda: completed_events.append(True),
        on_pipeline_failed=lambda err: failed_events.append(err),
        on_motion_state_changed=lambda st: motion_transitions.append(st),
    )

    try:
        # Part 1: Explicit branch before pipeline start returns False
        assert executor.branch_to_observation(reason="no_active_stage") is False

        # Part 2: Branching automatically on Nav2 failure
        resp = CheckMoveFeasibility.Response(feasible=True, plan_type=CheckMoveFeasibility.Response.PLAN_SINGLE_BASE)
        resp.pick_base_pose.pose.position.x = 1.0
        resp.pick_base_pose.header.frame_id = "map"
        goal = ChessMoveGoal(uci="e2e4")
        stages = MovePlanBuilder.build_steps(resp, goal)
        executor.start_pipeline(stages)

        assert len(failed_events) == 0
        assert len(completed_events) == 1
        assert MotionExecutionState.NAV_TO_OBS in motion_transitions
        assert executor.motion_state == MotionExecutionState.IDLE

        # Part 3: Explicit branch while active
        auto_complete = False
        stages2 = MovePlanBuilder.build_steps(resp, goal)
        executor.start_pipeline(stages2)
        assert executor.motion_state == MotionExecutionState.NAV_TO_PICK
        assert executor.branch_to_observation(reason="manual_request") is True
        assert executor.motion_state == MotionExecutionState.NAV_TO_OBS
    finally:
        perception.destroy()
        dispatcher.destroy()
        node.destroy_node()


def test_active_relocalization_on_manipulation_failure_scenarios(ros_context):
    """Verify active relocalization on manipulation failure: successful retry vs max recovery exhaustion."""
    node = Node("test_manip_reloc_scenarios_node")
    completed_events = []
    failed_events = []
    feedback_events = []
    node.publish_collection_feedback = lambda stage, pct: feedback_events.append((stage, pct))

    # Scenario 1: Manipulation fails once, recovers on second attempt
    manip_attempts_1 = 0
    nav_goals_1 = []

    class FailOnceDispatcher(FakeMotionClient):
        def send_navigation_goal(self, target_pose, timeout_sec=60.0, on_completed=None) -> bool:
            nav_goals_1.append(target_pose)
            if on_completed:
                on_completed(ActionResult(success=True, message="Nav ok"))
            return True

        def send_manipulation_goal(self, goal, timeout_sec=60.0, on_feedback=None, on_completed=None) -> bool:
            nonlocal manip_attempts_1
            manip_attempts_1 += 1
            if on_completed:
                if manip_attempts_1 == 1:
                    on_completed(ActionResult(success=False, message="IK resolution failed for APPROACH_PICK in RECORD mode"))
                else:
                    on_completed(ActionResult(success=True, message="Episode saved"))
            return True

    dispatcher1 = FailOnceDispatcher()
    perception1 = PerceptionContextManager(node=node)
    obs_nav1 = ObsNavigator(node=node, dispatcher=dispatcher1, standoff_distance=0.65)
    executor1 = MoveSequencer(
        node=node,
        dispatcher=dispatcher1,
        perception=perception1,
        chassis_settle_timeout_sec=0.01,
        observation_navigator=obs_nav1,
        tf_buffer=create_mock_tf_buffer(),
        max_recovery_attempts=2,
        on_pipeline_completed=lambda: completed_events.append(True),
        on_pipeline_failed=lambda err: failed_events.append(err),
    )

    try:
        resp = CheckMoveFeasibility.Response(feasible=True, plan_type=CheckMoveFeasibility.Response.PLAN_SINGLE_BASE)
        resp.pick_base_pose.pose.position.x = 1.0
        resp.pick_base_pose.header.frame_id = "map"
        goal = ChessMoveGoal(uci="e2e4")
        stages = MovePlanBuilder.build_steps(resp, goal)
        executor1.start_pipeline(stages)

        assert len(failed_events) == 0
        assert len(completed_events) == 1
        assert manip_attempts_1 == 2
        assert len(nav_goals_1) == 3
        assert executor1.motion_state == MotionExecutionState.IDLE

        # Scenario 2: Manipulation continuously fails and exhausts max_recovery_attempts
        manip_attempts_2 = 0

        class AlwaysFailDispatcher(FakeMotionClient):
            def send_navigation_goal(self, target_pose, timeout_sec=60.0, on_completed=None) -> bool:
                if on_completed:
                    on_completed(ActionResult(success=True, message="Nav ok"))
                return True

            def send_manipulation_goal(self, goal, timeout_sec=60.0, on_feedback=None, on_completed=None) -> bool:
                nonlocal manip_attempts_2
                manip_attempts_2 += 1
                if on_completed:
                    on_completed(ActionResult(success=False, message="JIT IK unsolvable for target in frame base_footprint"))
                return True

        completed_events.clear()
        failed_events.clear()
        dispatcher2 = AlwaysFailDispatcher()
        perception2 = PerceptionContextManager(node=node)
        obs_nav2 = ObsNavigator(node=node, dispatcher=dispatcher2, standoff_distance=0.65)
        executor2 = MoveSequencer(
            node=node,
            dispatcher=dispatcher2,
            perception=perception2,
            chassis_settle_timeout_sec=0.01,
            observation_navigator=obs_nav2,
            tf_buffer=create_mock_tf_buffer(),
            max_recovery_attempts=2,
            on_pipeline_completed=lambda: completed_events.append(True),
            on_pipeline_failed=lambda err: failed_events.append(err),
        )
        stages2 = MovePlanBuilder.build_steps(resp, goal)
        executor2.start_pipeline(stages2)

        assert len(completed_events) == 0
        assert len(failed_events) == 1
        assert "Manipulation execution failed" in failed_events[0]
        assert manip_attempts_2 == 3
        assert executor2.motion_state == MotionExecutionState.IDLE
    finally:
        perception1.destroy()
        dispatcher1.destroy()
        node.destroy_node()



