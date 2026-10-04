# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for StatusMarkerBuilder and 3D Robot HUD Visualization."""

import threading
from unittest.mock import MagicMock

import pytest
from builtin_interfaces.msg import Time
from lekiwi_orchestrator.fsm import MacroMissionState
from lekiwi_orchestrator.health_supervisor import (
    HealthSupervisor,
    HealthSupervisorConfig,
    OrchestratorStateSnapshot,
    StatusMarkerBuilder,
    VisualizerConfig,
)
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from visualization_msgs.msg import Marker


@pytest.fixture
def dummy_stamp() -> Time:
    t = Time()
    t.sec = 1234
    t.nanosec = 5678
    return t


def test_builder_deleteall_hygiene(dummy_stamp):
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


def test_builder_hud_marker_attributes(dummy_stamp):
    cfg = VisualizerConfig(robot_frame="base_link", hud_z_offset=0.5, font_scale=0.03)
    builder = StatusMarkerBuilder(cfg)
    snapshot = OrchestratorStateSnapshot(
        mission_state=MacroMissionState.WAITING_FOR_PLAYER_MOVE,
        perception_context=0,
    )
    markers = builder.build(
        snapshot=snapshot,
        nav_ready=True,
        robot_color="b",
        stamp=dummy_stamp,
    )

    hud = markers.markers[1]
    assert hud.header.frame_id == "base_link"
    assert hud.header.stamp == dummy_stamp
    assert hud.ns == "mission/robot_hud"
    assert hud.id == 0
    assert hud.type == Marker.TEXT_VIEW_FACING
    assert hud.action == Marker.ADD
    assert hud.pose.position.x == 0.0
    assert hud.pose.position.y == 0.0
    assert hud.pose.position.z == 0.5
    assert hud.scale.z == 0.03


def test_builder_boot_initializing_state(dummy_stamp):
    builder = StatusMarkerBuilder()
    snapshot = OrchestratorStateSnapshot(
        mission_state=MacroMissionState.WAITING_FOR_TF_READY,
        perception_context=0,
    )
    markers = builder.build(
        snapshot, nav_ready=False, robot_color="w", stamp=dummy_stamp
    )
    hud = markers.markers[1]

    assert hud.color.r == 1.0
    assert hud.color.g == 0.8
    assert hud.color.b == 0.0
    assert "Waiting for Nav Ready" in hud.text
    assert "White" in hud.text


def test_builder_idle_waiting_for_opponent(dummy_stamp):
    builder = StatusMarkerBuilder()
    snapshot = OrchestratorStateSnapshot(
        mission_state=MacroMissionState.WAITING_FOR_PLAYER_MOVE,
        perception_context=0,
    )
    markers = builder.build(
        snapshot, nav_ready=True, robot_color="b", stamp=dummy_stamp
    )
    hud = markers.markers[1]

    assert hud.color.r == 0.2
    assert hud.color.g == 0.8
    assert hud.color.b == 1.0
    assert "Waiting for Opponent" in hud.text
    assert "Black" in hud.text
    assert "Nav: Ready" in hud.text


def test_builder_executing_move_pipeline(dummy_stamp):
    builder = StatusMarkerBuilder()
    snapshot = OrchestratorStateSnapshot(
        mission_state=MacroMissionState.EXECUTING_MOVE_PIPELINE,
        perception_context=0,
        last_goal_move="e2e4",
        execution_stage="NAVIGATING_TO_PICK",
    )
    markers = builder.build(
        snapshot, nav_ready=True, robot_color="w", stamp=dummy_stamp
    )
    hud = markers.markers[1]

    assert hud.color.r == 0.1
    assert hud.color.g == 1.0
    assert hud.color.b == 0.3
    assert "Move: e2e4 (White)" in hud.text
    assert "Stage: NAVIGATING_TO_PICK" in hud.text


def test_builder_error_fallback(dummy_stamp):
    builder = StatusMarkerBuilder()
    snapshot = OrchestratorStateSnapshot(
        mission_state=MacroMissionState.ERROR_FALLBACK,
        perception_context=0,
    )
    markers = builder.build(
        snapshot, nav_ready=False, robot_color="b", stamp=dummy_stamp
    )
    hud = markers.markers[1]

    assert hud.color.r == 1.0
    assert hud.color.g == 0.1
    assert hud.color.b == 0.1
    assert "[ERROR]" in hud.text
    assert "Nav: Lost" in hud.text


def test_builder_additional_states(dummy_stamp):
    builder = StatusMarkerBuilder(VisualizerConfig(robot_frame="base_link"))
    snap_eval = OrchestratorStateSnapshot(
        mission_state=MacroMissionState.EVALUATING_BEST_MOVE,
        perception_context=1,
        last_goal_move="e2e4",
    )
    hud_eval = builder.build(
        snap_eval, nav_ready=True, robot_color="w", stamp=dummy_stamp
    ).markers[1]
    assert "e2e4" in hud_eval.text

    snap_done = OrchestratorStateSnapshot(
        mission_state=MacroMissionState.TURN_COMPLETED,
        perception_context=0,
    )
    hud_done = builder.build(
        snap_done, nav_ready=True, robot_color="w", stamp=dummy_stamp
    ).markers[1]
    assert "[COMPLETED]" in hud_done.text

    snap_fallback = OrchestratorStateSnapshot(
        mission_state=MacroMissionState.BOOT_INITIALIZING,
        perception_context=0,
    )
    hud_fallback = builder.build(
        snap_fallback, nav_ready=True, robot_color="w", stamp=dummy_stamp
    ).markers[1]
    assert hud_fallback.text



def test_health_supervisor_visualizer_integration():
    mock_node = MagicMock()
    mock_clock = MagicMock()
    now_msg = Time()
    now_msg.sec = 100
    mock_clock.now.return_value.to_msg.return_value = now_msg
    mock_node.get_clock.return_value = mock_clock

    mock_diag_pub = MagicMock()
    mock_marker_pub = MagicMock()
    mock_node.create_publisher.side_effect = [mock_diag_pub, mock_marker_pub]

    snapshot = OrchestratorStateSnapshot(
        mission_state=MacroMissionState.EXECUTING_MOVE_PIPELINE,
        perception_context=1,
        last_goal_move="d2d4",
        execution_stage="PICKING_PIECE",
    )

    config = HealthSupervisorConfig(
        robot_color="w",
        auto_recovery_enabled=False,
        enable_visualizer=True,
    )
    monitor = HealthSupervisor(
        node=mock_node,
        config=config,
        state_lock=threading.RLock(),
        cb_group_sub=MutuallyExclusiveCallbackGroup(),
        state_provider=lambda: snapshot,
    )

    monitor.publish_diagnostics(snapshot)

    assert mock_diag_pub.publish.called
    assert mock_marker_pub.publish.called
    published_markers = mock_marker_pub.publish.call_args[0][0]
    assert len(published_markers.markers) == 2
    assert published_markers.markers[0].action == Marker.DELETEALL
    assert "d2d4" in published_markers.markers[1].text
    assert "PICKING_PIECE" in published_markers.markers[1].text

    monitor.destroy()
    assert mock_node.destroy_publisher.called
