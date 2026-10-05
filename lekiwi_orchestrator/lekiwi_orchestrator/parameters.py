# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Type-safe configuration container parsed from ROS 2 node parameters.

Encapsulates parameter declarations, default values, and runtime type coercion.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from rclpy.node import Node


class OrchestratorParameters:
    """Type-safe configuration container parsed from ROS 2 node parameters.

    Provides early validation and cached immutable attribute access for mission settings.
    """

    def __init__(self, node: Node) -> None:
        """Declare and parse all orchestrator ROS 2 parameters from node handle."""
        node.declare_parameter("robot_color", "b")
        node.declare_parameter("board_frame", "chessboard_frame")
        node.declare_parameter("map_frame", "map")
        node.declare_parameter("feasibility_timeout_sec", 5.0)
        node.declare_parameter("action_timeout_sec", 60.0)
        node.declare_parameter("readiness_timeout_sec", 1.0)
        node.declare_parameter("pre_grasp_settle_sec", 2.0)
        node.declare_parameter("navigation", True)

        node.declare_parameter("recovery.auto_recovery_enabled", True)
        node.declare_parameter("recovery.auto_recovery_timeout_sec", 5.0)
        node.declare_parameter("recovery.max_recovery_attempts", 3)

        node.declare_parameter("topics.nav_ready", "/system/nav_ready")
        node.declare_parameter("topics.grasp_ready", "/system/grasp_ready")
        node.declare_parameter("topics.game_status", "/chess/game_status")
        node.declare_parameter("topics.perception_context", "/perception_context")
        node.declare_parameter("topics.diagnostics", "/diagnostics")

        node.declare_parameter(
            "services.check_feasibility", "/workspace/check_move_feasibility"
        )
        node.declare_parameter("services.recover", "/orchestrator/recover")
        node.declare_parameter(
            "services.set_perception_context",
            "/orchestrator/set_perception_context",
        )

        node.declare_parameter("actions.navigate_to_pose", "/navigate_to_pose")
        node.declare_parameter(
            "actions.execute_chess_move", "/manipulation/execute_chess_move"
        )

        node.declare_parameter("observation.standoff_distance", 0.65)
        node.declare_parameter("observation.radius_tolerance", 0.04)
        node.declare_parameter("observation.scan_timeout_sec", 6.0)
        node.declare_parameter(
            "observation.angle_offsets.relocalize",
            [0.0, -0.314, 0.314],
        )
        node.declare_parameter(
            "observation.angle_offsets.post_move_verify",
            [0.0, -0.314, 0.314, -0.558, 0.558],
        )

        node.declare_parameter("visualization.enabled", True)
        node.declare_parameter("visualization.topic", "~/status_markers")
        node.declare_parameter("visualization.hud_z_offset", 0.35)
        node.declare_parameter("visualization.robot_frame", "base_footprint")

        readiness_to = float(node.get_parameter("readiness_timeout_sec").value)
        if not math.isfinite(readiness_to) or readiness_to <= 0:
            raise ValueError("readiness_timeout_sec must be finite and positive")

        self.robot_color = str(node.get_parameter("robot_color").value).lower()
        self.board_frame = str(node.get_parameter("board_frame").value)
        self.map_frame = str(node.get_parameter("map_frame").value)
        self.feasibility_timeout_sec = float(
            node.get_parameter("feasibility_timeout_sec").value
        )
        self.action_timeout_sec = float(node.get_parameter("action_timeout_sec").value)
        self.readiness_timeout_sec = readiness_to
        self.pre_grasp_settle_sec = float(
            node.get_parameter("pre_grasp_settle_sec").value
        )
        self.navigation = bool(node.get_parameter("navigation").value)

        self.auto_recovery_enabled = bool(
            node.get_parameter("recovery.auto_recovery_enabled").value
        )
        self.auto_recovery_timeout_sec = float(
            node.get_parameter("recovery.auto_recovery_timeout_sec").value
        )
        self.max_recovery_attempts = int(
            node.get_parameter("recovery.max_recovery_attempts").value
        )

        self.nav_ready_topic = str(node.get_parameter("topics.nav_ready").value)
        self.grasp_ready_topic = str(node.get_parameter("topics.grasp_ready").value)
        self.game_status_topic = str(node.get_parameter("topics.game_status").value)
        self.perception_context_topic = str(
            node.get_parameter("topics.perception_context").value
        )
        self.diagnostics_topic = str(node.get_parameter("topics.diagnostics").value)

        self.check_feasibility_srv = str(
            node.get_parameter("services.check_feasibility").value
        )
        self.recover_srv = str(node.get_parameter("services.recover").value)
        self.set_perception_srv = str(
            node.get_parameter("services.set_perception_context").value
        )

        self.navigate_to_pose_action = str(
            node.get_parameter("actions.navigate_to_pose").value
        )
        self.execute_chess_move_action = str(
            node.get_parameter("actions.execute_chess_move").value
        )

        self.observation_standoff_distance = float(
            node.get_parameter("observation.standoff_distance").value
        )
        self.observation_radius_tolerance = float(
            node.get_parameter("observation.radius_tolerance").value
        )
        self.observation_scan_timeout_sec = float(
            node.get_parameter("observation.scan_timeout_sec").value
        )
        raw_reloc = node.get_parameter("observation.angle_offsets.relocalize").value
        self.observation_angle_offsets_relocalize = (
            [float(v) for v in raw_reloc] if raw_reloc else [0.0, -0.314, 0.314]
        )
        raw_pmv = node.get_parameter("observation.angle_offsets.post_move_verify").value
        self.observation_angle_offsets_post_move_verify = (
            [float(v) for v in raw_pmv]
            if raw_pmv
            else [0.0, -0.314, 0.314, -0.558, 0.558]
        )

        self.visualization_enabled = bool(
            node.get_parameter("visualization.enabled").value
        )
        self.visualization_topic = str(node.get_parameter("visualization.topic").value)
        self.visualization_hud_z_offset = float(
            node.get_parameter("visualization.hud_z_offset").value
        )
        self.visualization_robot_frame = str(
            node.get_parameter("visualization.robot_frame").value
        )


