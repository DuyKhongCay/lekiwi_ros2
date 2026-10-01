# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for MovePipeline: StagePipelineBuilder and MovePipelineExecutor."""

from __future__ import annotations

import pytest
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from lekiwi_interfaces.msg import PerceptionContext
from lekiwi_interfaces.srv import CheckMoveFeasibility
from lekiwi_orchestrator.fsm import MotionExecutionState
from lekiwi_orchestrator.motion_dispatcher import (
    ActionDispatcherInterface,
    ActionResult,
    ActiveObservationNavigator,
)
from lekiwi_orchestrator.move_pipeline import (
    ChessMoveGoal,
    ExecutionStage,
    MovePipelineExecutor,
    StagePipelineBuilder,
)
from lekiwi_orchestrator.perception_manager import PerceptionContextCoordinator
from rclpy.node import Node


class DummyPipelineDispatcher(ActionDispatcherInterface):
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

    stages = StagePipelineBuilder.build_stages(resp, base_goal)
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

    stages = StagePipelineBuilder.build_stages(resp, base_goal)
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

    stages = StagePipelineBuilder.build_stages(resp, base_goal)
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

    stages = StagePipelineBuilder.build_stages(resp, capture_goal)
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

    stages = StagePipelineBuilder.build_stages(resp, capture_goal)
    assert len(stages) == 3
    assert stages[0].name == "CLEAR"
    assert stages[0].motion_state == MotionExecutionState.CLEARING_PIECE
    assert stages[1].name == "PICK"
    assert stages[1].motion_state == MotionExecutionState.PICKING_PIECE
    assert stages[2].name == "PLACE"
    assert stages[2].motion_state == MotionExecutionState.PLACING_PIECE


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
    perception = PerceptionContextCoordinator(
        node=node,
        perception_context_topic="/test/pipeline/perception_context",
        set_perception_service_name="/test/pipeline/set_context",
    )

    completed_events = []
    failed_events = []
    motion_transitions = []

    executor = MovePipelineExecutor(
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

        # Build dual base plan
        resp = CheckMoveFeasibility.Response()
        resp.feasible = True
        resp.plan_type = CheckMoveFeasibility.Response.PLAN_DUAL_BASE
        resp.pick_base_pose.pose.position.x = 1.0
        resp.place_base_pose.pose.position.x = 2.0

        goal = ChessMoveGoal(uci="e2e4", from_square="e2", to_square="e4")
        stages = StagePipelineBuilder.build_stages(resp, goal)

        # Run pipeline
        executor.start_pipeline(stages, resp)

        # Verified pipeline completion
        assert len(completed_events) == 1
        assert len(failed_events) == 0
        assert executor.motion_state == MotionExecutionState.IDLE
        assert not executor.is_active

        # Check recorded Level 2 transitions
        # Expected sequence: NAV_TO_PICK -> PICKING_PIECE -> NAV_TO_PLACE -> PLACING_PIECE -> POST_MOVE_VERIFYING -> IDLE
        expected_seq = [
            MotionExecutionState.NAV_TO_PICK,
            MotionExecutionState.PICKING_PIECE,
            MotionExecutionState.NAV_TO_PLACE,
            MotionExecutionState.PLACING_PIECE,
            MotionExecutionState.POST_MOVE_VERIFYING,
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
    perception = PerceptionContextCoordinator(
        node=node,
        perception_context_topic="/test/cancel/perception_context",
        set_perception_service_name="/test/cancel/set_context",
    )

    executor = MovePipelineExecutor(
        node=node,
        dispatcher=dispatcher,
        perception=perception,
    )
    try:
        resp = CheckMoveFeasibility.Response()
        resp.plan_type = CheckMoveFeasibility.Response.PLAN_SINGLE_BASE
        resp.pick_base_pose.pose.position.x = 1.0
        goal = ChessMoveGoal(uci="e2e4", from_square="e2", to_square="e4")
        stages = StagePipelineBuilder.build_stages(resp, goal)

        executor.start_pipeline(stages, resp)
        executor.cancel()

        assert executor.motion_state == MotionExecutionState.IDLE
        assert not executor.is_active
    finally:
        perception.destroy()
        dispatcher.destroy()
        node.destroy_node()


def test_stage_pipeline_builder_custom_registry(base_goal):
    """Verify Open/Closed Principle: StagePipelineBuilder can be extended with new plan types without modification."""
    custom_plan_type = 999

    def custom_builder(resp, details):
        return [
            ExecutionStage(
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

    StagePipelineBuilder.register_builder(custom_plan_type, custom_builder)

    resp = CheckMoveFeasibility.Response()
    resp.plan_type = custom_plan_type

    stages = StagePipelineBuilder.build_stages(resp, base_goal)
    assert len(stages) == 1
    assert stages[0].name == "CUSTOM_PROMOTION_STAGE"
    assert stages[0].instruction == "Custom promotion handler executed"


def test_move_pipeline_grasp_readiness_timeout_fails_safely(ros_context):
    """Verify that when grasp readiness times out, failure is safely reported without crashing or hanging."""
    node = Node("test_grasp_readiness_node")
    dispatcher = DummyPipelineDispatcher()
    perception = PerceptionContextCoordinator(
        node=node,
        perception_context_topic="/test/grasp/perception_context",
        set_perception_service_name="/test/grasp/set_context",
    )

    failed_events = []

    executor = MovePipelineExecutor(
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
        stages = StagePipelineBuilder.build_stages(resp, goal)

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
    perception = PerceptionContextCoordinator(
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

    executor = MovePipelineExecutor(
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
        stages = StagePipelineBuilder.build_stages(resp, goal)

        executor.start_pipeline(stages, resp)

        assert len(failed_events) == 0
        assert len(completed_events) == 1
        assert executor.motion_state == MotionExecutionState.IDLE
    finally:
        perception.destroy()
        dispatcher.destroy()
        node.destroy_node()


def test_post_move_verify_retreats_to_observation_pose(ros_context):
    """Verify post-move verification dispatches navigation retreat to observation pose and sets perception to BOARD_STATE_SCAN."""
    node = Node("test_post_move_verify_node")

    dispatched_nav_goals = []

    class VerifyTrackingDispatcher(DummyPipelineDispatcher):
        def send_navigation_goal(
            self, target_pose, timeout_sec=60.0, on_completed=None
        ):
            dispatched_nav_goals.append(target_pose)
            if on_completed:
                on_completed(ActionResult(success=True, message="Reached obs standoff"))
            return True

    dispatcher = VerifyTrackingDispatcher()
    perception = PerceptionContextCoordinator(
        node=node,
        perception_context_topic="/test/post_move/perception_context",
        set_perception_service_name="/test/post_move/set_context",
    )

    obs_pose = PoseStamped()
    obs_pose.header.frame_id = "map"
    obs_pose.pose.position.x = -0.65
    obs_pose.pose.position.y = 0.0

    completed = []
    executor = MovePipelineExecutor(
        node=node,
        dispatcher=dispatcher,
        perception=perception,
        grasp_readiness_provider=lambda: True,
        observation_pose_provider=lambda: obs_pose,
        navigation_enabled=True,
        on_pipeline_completed=lambda: completed.append(True),
    )

    try:
        resp = CheckMoveFeasibility.Response()
        resp.feasible = True
        resp.plan_type = CheckMoveFeasibility.Response.PLAN_ZERO_NAV
        goal = ChessMoveGoal(uci="e2e4", from_square="e2", to_square="e4")
        stages = StagePipelineBuilder.build_stages(resp, goal)

        executor.start_pipeline(stages, resp)

        assert len(completed) == 1
        # Dispatched 1 nav goal: the post-move retreat to obs_pose
        assert len(dispatched_nav_goals) == 1
        assert dispatched_nav_goals[0].pose.position.x == -0.65
        assert perception.context == PerceptionContext.BOARD_STATE_SCAN
        assert executor.motion_state == MotionExecutionState.IDLE
    finally:
        perception.destroy()
        dispatcher.destroy()
        node.destroy_node()


def test_post_move_verify_skips_nav_when_disabled(ros_context):
    """Verify post-move verification skips nav retreat when navigation is disabled and directly sets BOARD_STATE_SCAN."""
    node = Node("test_post_move_skip_node")
    dispatched_nav_goals = []

    class VerifyTrackingDispatcher(DummyPipelineDispatcher):
        def send_navigation_goal(
            self, target_pose, timeout_sec=60.0, on_completed=None
        ):
            dispatched_nav_goals.append(target_pose)
            return True

    dispatcher = VerifyTrackingDispatcher()
    perception = PerceptionContextCoordinator(
        node=node,
        perception_context_topic="/test/post_move_skip/perception_context",
        set_perception_service_name="/test/post_move_skip/set_context",
    )

    completed = []
    executor = MovePipelineExecutor(
        node=node,
        dispatcher=dispatcher,
        perception=perception,
        grasp_readiness_provider=lambda: True,
        observation_pose_provider=None,
        navigation_enabled=False,
        on_pipeline_completed=lambda: completed.append(True),
    )

    try:
        resp = CheckMoveFeasibility.Response()
        resp.feasible = True
        resp.plan_type = CheckMoveFeasibility.Response.PLAN_ZERO_NAV
        goal = ChessMoveGoal(uci="e2e4", from_square="e2", to_square="e4")
        stages = StagePipelineBuilder.build_stages(resp, goal)

        executor.start_pipeline(stages, resp)

        assert len(completed) == 1
        assert len(dispatched_nav_goals) == 0
        assert perception.context == PerceptionContext.BOARD_STATE_SCAN
        assert executor.motion_state == MotionExecutionState.IDLE
    finally:
        perception.destroy()
        dispatcher.destroy()
        node.destroy_node()


def test_post_move_verify_watchdog_waits_for_board_verification(ros_context):
    """Verify that when board is not verified, verification window activates and notify_board_verified completes it."""
    node = Node("test_pmv_watchdog_node")
    dispatcher = DummyPipelineDispatcher()
    perception = PerceptionContextCoordinator(
        node=node,
        perception_context_topic="/test/pmv_w/perception_context",
        set_perception_service_name="/test/pmv_w/set_context",
    )

    obs_pose = PoseStamped()
    obs_pose.header.frame_id = "map"
    obs_pose.pose.position.x = 0.0
    obs_pose.pose.position.y = 0.65

    obs_nav = ActiveObservationNavigator(
        node=node,
        dispatcher=dispatcher,
        map_frame="map",
        standoff_distance=0.65,
    )

    board_verified = [False]
    completed = []

    executor = MovePipelineExecutor(
        node=node,
        dispatcher=dispatcher,
        perception=perception,
        grasp_readiness_provider=lambda: True,
        observation_pose_provider=lambda: obs_pose,
        navigation_enabled=True,
        observation_navigator=obs_nav,
        observation_scan_timeout_sec=5.0,
        board_verified_provider=lambda: board_verified[0],
        on_pipeline_completed=lambda: completed.append(True),
    )

    try:
        resp = CheckMoveFeasibility.Response()
        resp.feasible = True
        resp.plan_type = CheckMoveFeasibility.Response.PLAN_ZERO_NAV
        goal = ChessMoveGoal(uci="e2e4", from_square="e2", to_square="e4")
        stages = StagePipelineBuilder.build_stages(resp, goal)

        executor.start_pipeline(stages, resp)

        # Reached post-move standoff, but board not verified -> remains in POST_MOVE_VERIFYING
        assert len(completed) == 0
        assert executor.motion_state == MotionExecutionState.POST_MOVE_VERIFYING
        assert executor._verification_timer is not None

        # External notification that referee verified board
        board_verified[0] = True
        executor.notify_board_verified()

        # Completes pipeline, cancels timer, returns to IDLE
        assert len(completed) == 1
        assert executor.motion_state == MotionExecutionState.IDLE
        assert executor._verification_timer is None
    finally:
        executor.destroy()
        perception.destroy()
        dispatcher.destroy()
        node.destroy_node()


def test_post_move_verify_timeout_repositions_base_via_navigator(ros_context):
    """Verify that when verification times out, ActiveObservationNavigator repositions base to next vantage point."""
    node = Node("test_pmv_timeout_node")
    repositioned_poses = []

    class RepositionTrackingDispatcher(DummyPipelineDispatcher):
        def send_navigation_goal(
            self, target_pose, timeout_sec=60.0, on_completed=None
        ):
            repositioned_poses.append(target_pose)
            if on_completed:
                on_completed(
                    ActionResult(success=True, message="Reached vantage point")
                )
            return True

    dispatcher = RepositionTrackingDispatcher()
    perception = PerceptionContextCoordinator(
        node=node,
        perception_context_topic="/test/pmv_t/perception_context",
        set_perception_service_name="/test/pmv_t/set_context",
    )

    obs_pose = PoseStamped()
    obs_pose.header.frame_id = "map"
    obs_pose.pose.position.x = 0.0
    obs_pose.pose.position.y = 0.65

    obs_nav = ActiveObservationNavigator(
        node=node,
        dispatcher=dispatcher,
        map_frame="map",
        standoff_distance=0.65,
        robot_color="b",
    )

    board_verified = [False]
    completed = []

    executor = MovePipelineExecutor(
        node=node,
        dispatcher=dispatcher,
        perception=perception,
        grasp_readiness_provider=lambda: True,
        observation_pose_provider=lambda: obs_pose,
        navigation_enabled=True,
        observation_navigator=obs_nav,
        observation_scan_timeout_sec=5.0,
        board_verified_provider=lambda: board_verified[0],
        on_pipeline_completed=lambda: completed.append(True),
    )

    try:
        resp = CheckMoveFeasibility.Response()
        resp.feasible = True
        resp.plan_type = CheckMoveFeasibility.Response.PLAN_ZERO_NAV
        goal = ChessMoveGoal(uci="e2e4", from_square="e2", to_square="e4")
        stages = StagePipelineBuilder.build_stages(resp, goal)

        executor.start_pipeline(stages, resp)

        # 1 nav goal dispatched initially (standoff retreat)
        assert len(repositioned_poses) == 1
        assert executor.motion_state == MotionExecutionState.POST_MOVE_VERIFYING
        assert executor._observation_attempt == 0

        # Simulate timeout firing
        executor._on_verification_timeout()

        # Repositioning goal dispatched by ActiveObservationNavigator!
        assert len(repositioned_poses) == 2
        assert executor._observation_attempt == 1
        assert executor.motion_state == MotionExecutionState.POST_MOVE_VERIFYING

        # Now board is verified at new viewpoint
        board_verified[0] = True
        executor.notify_board_verified()

        assert len(completed) == 1
        assert executor.motion_state == MotionExecutionState.IDLE
    finally:
        executor.destroy()
        perception.destroy()
        dispatcher.destroy()
        node.destroy_node()


def test_post_move_verify_max_attempts_advances_stage(ros_context):
    """Verify that when max observation attempts are reached without verification, stage advances gracefully."""
    node = Node("test_pmv_max_attempts_node")
    dispatcher = DummyPipelineDispatcher()
    perception = PerceptionContextCoordinator(
        node=node,
        perception_context_topic="/test/pmv_max/perception_context",
        set_perception_service_name="/test/pmv_max/set_context",
    )

    obs_pose = PoseStamped()
    obs_pose.header.frame_id = "map"
    obs_pose.pose.position.x = 0.0
    obs_pose.pose.position.y = 0.65

    obs_nav = ActiveObservationNavigator(
        node=node,
        dispatcher=dispatcher,
        map_frame="map",
        standoff_distance=0.65,
    )

    completed = []

    executor = MovePipelineExecutor(
        node=node,
        dispatcher=dispatcher,
        perception=perception,
        grasp_readiness_provider=lambda: True,
        observation_pose_provider=lambda: obs_pose,
        navigation_enabled=True,
        observation_navigator=obs_nav,
        observation_scan_timeout_sec=5.0,
        board_verified_provider=lambda: False,
        max_observation_attempts=2,
        on_pipeline_completed=lambda: completed.append(True),
    )

    try:
        resp = CheckMoveFeasibility.Response()
        resp.feasible = True
        resp.plan_type = CheckMoveFeasibility.Response.PLAN_ZERO_NAV
        goal = ChessMoveGoal(uci="e2e4", from_square="e2", to_square="e4")
        stages = StagePipelineBuilder.build_stages(resp, goal)

        executor.start_pipeline(stages, resp)
        assert len(completed) == 0

        # Attempt 1
        executor._on_verification_timeout()
        assert len(completed) == 0
        assert executor._observation_attempt == 1

        # Attempt 2
        executor._on_verification_timeout()
        assert len(completed) == 0
        assert executor._observation_attempt == 2

        # Attempt 3: exceeds max_observation_attempts (2) -> advances stage
        executor._on_verification_timeout()
        assert len(completed) == 1
        assert executor.motion_state == MotionExecutionState.IDLE
    finally:
        executor.destroy()
        perception.destroy()
        dispatcher.destroy()
        node.destroy_node()
