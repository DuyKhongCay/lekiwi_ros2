# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for StagePipelineBuilder pure domain logic across all plan types."""

from __future__ import annotations

import pytest
from geometry_msgs.msg import Point
from lekiwi_interfaces.srv import CheckMoveFeasibility

from lekiwi_orchestrator.stage_pipeline_builder import (
    ChessMoveGoal,
    StagePipelineBuilder,
)


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
    assert stages[0].pick_point.x == 0.1
    assert stages[0].place_point.x == 0.3


def test_build_single_base_stage(base_goal):
    resp = CheckMoveFeasibility.Response()
    resp.plan_type = CheckMoveFeasibility.Response.PLAN_SINGLE_BASE
    resp.pick_base_pose.pose.position.x = 1.0

    stages = StagePipelineBuilder.build_stages(resp, base_goal)
    assert len(stages) == 1
    assert stages[0].name == "MOVE"
    assert stages[0].target_pose is not None
    assert stages[0].target_pose.pose.position.x == 1.0


def test_build_dual_base_stages(base_goal):
    resp = CheckMoveFeasibility.Response()
    resp.plan_type = CheckMoveFeasibility.Response.PLAN_DUAL_BASE
    resp.pick_base_pose.pose.position.x = 1.0
    resp.place_base_pose.pose.position.x = 2.0

    stages = StagePipelineBuilder.build_stages(resp, base_goal)
    assert len(stages) == 2
    assert stages[0].name == "PICK"
    assert stages[0].target_pose.pose.position.x == 1.0
    assert stages[0].from_square == "e2"
    assert stages[1].name == "PLACE"
    assert stages[1].target_pose.pose.position.x == 2.0
    assert stages[1].to_square == "e4"


def test_build_capture_zero_nav_stages(capture_goal):
    resp = CheckMoveFeasibility.Response()
    resp.plan_type = CheckMoveFeasibility.Response.PLAN_CAPTURE_ZERO_NAV
    resp.clear_point = Point(x=0.5, y=0.5, z=0.03)

    stages = StagePipelineBuilder.build_stages(resp, capture_goal)
    assert len(stages) == 2
    assert stages[0].name == "CLEAR"
    assert stages[0].target_pose is None
    assert stages[0].is_capture is True
    assert stages[0].from_square == "d5"
    assert stages[1].name == "MOVE"
    assert stages[1].target_pose is None
    assert stages[1].is_capture is False


def test_build_capture_single_base_stages(capture_goal):
    resp = CheckMoveFeasibility.Response()
    resp.plan_type = CheckMoveFeasibility.Response.PLAN_CAPTURE_SINGLE_BASE
    resp.clear_base_pose.pose.position.y = 0.5

    stages = StagePipelineBuilder.build_stages(resp, capture_goal)
    assert len(stages) == 2
    assert stages[0].name == "CLEAR"
    assert stages[0].target_pose is not None
    assert stages[0].target_pose.pose.position.y == 0.5
    assert stages[1].name == "MOVE"
    assert stages[1].target_pose is None


def test_build_capture_triple_base_stages(capture_goal):
    resp = CheckMoveFeasibility.Response()
    resp.plan_type = CheckMoveFeasibility.Response.PLAN_CAPTURE_TRIPLE_BASE
    resp.clear_base_pose.pose.position.x = 1.0
    resp.pick_base_pose.pose.position.x = 2.0
    resp.place_base_pose.pose.position.x = 3.0

    stages = StagePipelineBuilder.build_stages(resp, capture_goal)
    assert len(stages) == 3
    assert stages[0].name == "CLEAR"
    assert stages[0].target_pose.pose.position.x == 1.0
    assert stages[1].name == "PICK"
    assert stages[1].target_pose.pose.position.x == 2.0
    assert stages[2].name == "PLACE"
    assert stages[2].target_pose.pose.position.x == 3.0


def test_build_capture_dual_base_stages_same_clear_and_pick(capture_goal):
    resp = CheckMoveFeasibility.Response()
    resp.plan_type = CheckMoveFeasibility.Response.PLAN_CAPTURE_DUAL_BASE
    # clear and pick base are identical (<5cm)
    resp.clear_base_pose.pose.position.x = 1.0
    resp.pick_base_pose.pose.position.x = 1.01
    resp.place_base_pose.pose.position.x = 3.0

    stages = StagePipelineBuilder.build_stages(resp, capture_goal)
    assert len(stages) == 3
    assert stages[0].name == "CLEAR"
    assert stages[0].target_pose is not None
    # PICK target_pose is None because robot is already at clear base!
    assert stages[1].name == "PICK"
    assert stages[1].target_pose is None
    assert stages[2].name == "PLACE"
    assert stages[2].target_pose is not None
