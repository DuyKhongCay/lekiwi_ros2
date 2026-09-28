# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for MovePipeline: StagePipelineBuilder and MovePipelineExecutor."""

from __future__ import annotations

import pytest
import rclpy
from geometry_msgs.msg import Point
from lekiwi_orchestrator.fsm import MotionExecutionState
from lekiwi_orchestrator.motion_dispatcher import SimulatedActionDispatcher
from lekiwi_orchestrator.move_pipeline import (
    ChessMoveGoal,
    ExecutionStage,
    MovePipelineExecutor,
    StagePipelineBuilder,
)
from lekiwi_orchestrator.perception_manager import PerceptionContextCoordinator
from rclpy.node import Node

from lekiwi_interfaces.srv import CheckMoveFeasibility


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
    dispatcher = SimulatedActionDispatcher(node)
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
    dispatcher = SimulatedActionDispatcher(node)
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


def test_move_pipeline_grasp_readiness_recovery(ros_context):
    """Verify that when grasp readiness is False, observation recovery is invoked without crashing."""
    node = Node("test_grasp_readiness_node")
    dispatcher = SimulatedActionDispatcher(node)
    perception = PerceptionContextCoordinator(
        node=node,
        perception_context_topic="/test/grasp/perception_context",
        set_perception_service_name="/test/grasp/set_context",
    )

    recovery_called = []
    failed_events = []

    executor = MovePipelineExecutor(
        node=node,
        dispatcher=dispatcher,
        perception=perception,
        grasp_readiness_provider=lambda: False,  # Grasp not ready!
        on_observation_recovery_requested=lambda: recovery_called.append(True) or True,
        on_pipeline_failed=lambda err: failed_events.append(err),
    )

    try:
        resp = CheckMoveFeasibility.Response()
        resp.feasible = True
        resp.plan_type = CheckMoveFeasibility.Response.PLAN_ZERO_NAV
        goal = ChessMoveGoal(uci="e2e4", from_square="e2", to_square="e4")
        stages = StagePipelineBuilder.build_stages(resp, goal)

        executor.start_pipeline(stages, resp)

        # Active observation recovery should be invoked
        assert len(recovery_called) == 1
        # Should NOT report pipeline failure / crash
        assert len(failed_events) == 0
        assert executor.motion_state == MotionExecutionState.IDLE
        assert not executor.is_active
    finally:
        perception.destroy()
        dispatcher.destroy()
        node.destroy_node()


def test_move_pipeline_grasp_unready_without_recovery_fails(ros_context):
    """Verify that when grasp is unready and no recovery callback exists, failure is reported."""
    node = Node("test_grasp_fail_node")
    dispatcher = SimulatedActionDispatcher(node)
    perception = PerceptionContextCoordinator(
        node=node,
        perception_context_topic="/test/grasp_fail/perception_context",
        set_perception_service_name="/test/grasp_fail/set_context",
    )

    failed_events = []

    executor = MovePipelineExecutor(
        node=node,
        dispatcher=dispatcher,
        perception=perception,
        grasp_readiness_provider=lambda: False,
        on_observation_recovery_requested=None,
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
    finally:
        perception.destroy()
        dispatcher.destroy()
        node.destroy_node()

