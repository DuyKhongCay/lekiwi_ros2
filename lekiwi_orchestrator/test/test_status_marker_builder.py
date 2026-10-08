# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for StatusMarkerBuilder and 3D Robot HUD Visualization."""

import pytest
from builtin_interfaces.msg import Time
from lekiwi_orchestrator.fsm import MacroMissionState
from lekiwi_orchestrator.health_supervisor import (
    OrchestratorStateSnapshot,
    StatusMarkerBuilder,
    VisualizerConfig,
)
from visualization_msgs.msg import Marker


@pytest.fixture
def dummy_stamp() -> Time:
    t = Time()
    t.sec = 1234
    t.nanosec = 5678
    return t


def test_builder_deleteall_hygiene(dummy_stamp):
    """Verify that builder always prepends a DELETEALL marker for RViz telemetry hygiene."""
    builder = StatusMarkerBuilder(VisualizerConfig(robot_frame="test_base"))
    snapshot = OrchestratorStateSnapshot(
        mission_state=MacroMissionState.WAITING_FOR_PLAYER_MOVE,
        perception_context=0,
    )
    markers = builder.build(
        snapshot=snapshot,
        nav_ready=True,
        robot_color="w",
        stamp=dummy_stamp,
    )

    assert len(markers.markers) == 2
    clear_marker = markers.markers[0]
    assert clear_marker.action == Marker.DELETEALL
    assert clear_marker.header.frame_id == "test_base"
    assert clear_marker.header.stamp == dummy_stamp


@pytest.mark.parametrize(
    ("state", "nav_ready", "robot_color", "last_move", "stage", "expected_text_snippets"),
    [
        (
            MacroMissionState.WAITING_FOR_TF_READY,
            False,
            "w",
            "",
            "",
            ["Waiting for Nav Ready", "White"],
        ),
        (
            MacroMissionState.WAITING_FOR_PLAYER_MOVE,
            True,
            "b",
            "",
            "",
            ["Waiting for Opponent", "Black", "Nav: Ready"],
        ),
        (
            MacroMissionState.EXECUTING_MOVE_PIPELINE,
            True,
            "w",
            "e2e4",
            "NAVIGATING_TO_PICK",
            ["Move: e2e4 (White)", "Stage: NAVIGATING_TO_PICK"],
        ),
        (
            MacroMissionState.ERROR_FALLBACK,
            False,
            "b",
            "",
            "",
            ["[ERROR]", "Nav: Lost"],
        ),
        (
            MacroMissionState.EVALUATING_BEST_MOVE,
            True,
            "w",
            "e2e4",
            "",
            ["e2e4"],
        ),
        (
            MacroMissionState.TURN_COMPLETED,
            True,
            "w",
            "",
            "",
            ["[COMPLETED]"],
        ),
    ],
)
def test_builder_hud_marker_attributes_parameterized(
    dummy_stamp, state, nav_ready, robot_color, last_move, stage, expected_text_snippets
):
    """Verify HUD marker attributes, coordinate frames, and text annotations across all states."""
    cfg = VisualizerConfig(robot_frame="base_link", hud_z_offset=0.5, font_scale=0.03)
    builder = StatusMarkerBuilder(cfg)
    snapshot = OrchestratorStateSnapshot(
        mission_state=state,
        perception_context=0,
        last_goal_move=last_move,
        execution_stage=stage,
    )
    markers = builder.build(
        snapshot=snapshot,
        nav_ready=nav_ready,
        robot_color=robot_color,
        stamp=dummy_stamp,
    )

    hud = markers.markers[1]
    assert hud.header.frame_id == "base_link"
    assert hud.header.stamp == dummy_stamp
    assert hud.ns == "mission/robot_hud"
    assert hud.id == 0
    assert hud.type == Marker.TEXT_VIEW_FACING
    assert hud.action == Marker.ADD
    assert hud.pose.position.z == 0.5
    assert hud.scale.z == 0.03

    for snippet in expected_text_snippets:
        assert snippet in hud.text
