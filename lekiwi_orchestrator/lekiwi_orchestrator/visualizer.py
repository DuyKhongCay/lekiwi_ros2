# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""
3D Status HUD Visualizer for LeKiwi Mission Orchestrator.
Builds RViz2 MarkerArray representing high-level mission state, execution stage,
and system health floating above base_footprint.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker, MarkerArray

from lekiwi_orchestrator.fsm import MISSION_STATE_NAMES, MissionState

if TYPE_CHECKING:
    from lekiwi_orchestrator.health_monitor import OrchestratorStateSnapshot


@dataclass(frozen=True)
class VisualizerConfig:
    """Configuration for 3D Robot Status HUD Marker."""

    robot_frame: str = "base_footprint"
    hud_z_offset: float = 0.35
    font_scale: float = 0.025
    ns: str = "mission/robot_hud"


class MissionStatusMarkerBuilder:
    """
    Pure builder producing standardized ROS 2 MarkerArray for Robot 3D Status HUD.
    Decoupled from ROS node context for deterministic testing.
    """

    def __init__(self, config: VisualizerConfig | None = None) -> None:
        self._config = config or VisualizerConfig()

    @property
    def config(self) -> VisualizerConfig:
        return self._config

    def build(
        self,
        snapshot: OrchestratorStateSnapshot,
        nav_ready: bool,
        robot_color: str,
        stamp: Time,
    ) -> MarkerArray:
        """
        Assemble MarkerArray with DELETEALL hygiene followed by a 3D text billboard.
        """
        markers = MarkerArray()

        # 0. Always start with DELETEALL to prevent stale remnants across state changes
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        clear_marker.header.frame_id = self._config.robot_frame
        clear_marker.header.stamp = stamp
        markers.markers.append(clear_marker)

        # 1. Layer 1: Robot 3D Status HUD Billboard
        hud = Marker()
        hud.header.frame_id = self._config.robot_frame
        hud.header.stamp = stamp
        hud.ns = self._config.ns
        hud.id = 0
        hud.type = Marker.TEXT_VIEW_FACING
        hud.action = Marker.ADD

        hud.pose.position.x = 0.0
        hud.pose.position.y = 0.0
        hud.pose.position.z = self._config.hud_z_offset
        hud.pose.orientation.w = 1.0

        hud.scale.z = self._config.font_scale

        state = snapshot.mission_state
        state_str = MISSION_STATE_NAMES.get(state, "UNKNOWN")
        color_name = "White" if robot_color.lower().startswith("w") else "Black"

        if state in (MissionState.BOOT_INITIALIZING, MissionState.WAITING_FOR_TF_READY):
            # Warning amber/yellow: Waiting on Nav lease
            hud.color.r = 1.0
            hud.color.g = 0.8
            hud.color.b = 0.0
            hud.color.a = 1.0
            hud.text = f"[{state_str}]\nWaiting for Nav Ready | Robot: {color_name}"

        elif state == MissionState.WAITING_FOR_PLAYER_MOVE:
            # Soft blue/cyan: Idle waiting for opponent
            hud.color.r = 0.2
            hud.color.g = 0.8
            hud.color.b = 1.0
            hud.color.a = 1.0
            nav_status = "Ready" if nav_ready else "Unready"
            hud.text = (
                f"[IDLE] Waiting for Opponent\nRobot: {color_name} | Nav: {nav_status}"
            )

        elif state in (
            MissionState.EVALUATING_BEST_MOVE,
            MissionState.CHECKING_REACHABILITY,
        ):
            # Warm yellow: Decision / reachability computation in progress
            hud.color.r = 0.95
            hud.color.g = 0.9
            hud.color.b = 0.2
            hud.color.a = 1.0
            target_str = snapshot.last_goal_move or "Evaluating"
            hud.text = f"[{state_str}]\nTarget: {target_str} ({color_name})"

        elif state == MissionState.EXECUTING_MOVE_PIPELINE:
            # Neon green: Active execution
            hud.color.r = 0.1
            hud.color.g = 1.0
            hud.color.b = 0.3
            hud.color.a = 1.0
            stage_str = snapshot.execution_stage or "Executing"
            move_str = snapshot.last_goal_move or "Unknown"
            hud.text = (
                f"[EXECUTING] Move: {move_str} ({color_name})\nStage: {stage_str}"
            )

        elif state == MissionState.TURN_COMPLETED:
            # Bright cyan: Concluded turn
            hud.color.r = 0.0
            hud.color.g = 1.0
            hud.color.b = 0.8
            hud.color.a = 1.0
            hud.text = f"[COMPLETED] Turn Finalized\nRobot: {color_name}"

        elif state == MissionState.ERROR_FALLBACK:
            # Alert vivid red: Error state
            hud.color.r = 1.0
            hud.color.g = 0.1
            hud.color.b = 0.1
            hud.color.a = 1.0
            nav_status = "Ready" if nav_ready else "Lost"
            hud.text = f"[ERROR] System in Error Fallback\nNav: {nav_status}"

        else:
            # Neutral light gray for any other state
            hud.color.r = 0.85
            hud.color.g = 0.85
            hud.color.b = 0.85
            hud.color.a = 1.0
            hud.text = f"[{state_str}]\nRobot: {color_name}"

        markers.markers.append(hud)
        return markers
