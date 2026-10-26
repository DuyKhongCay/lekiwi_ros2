# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Type-safe configuration container parsed from ROS 2 node parameters.

Encapsulates parameter declarations, default values, and runtime type coercion.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from rcl_interfaces.msg import ParameterDescriptor

if TYPE_CHECKING:
    from rclpy.node import Node


class OrchestratorParameters:
    """Type-safe configuration container parsed from ROS 2 node parameters.

    Provides early validation and cached immutable attribute access for mission settings.
    """

    def __init__(self, node: Node) -> None:
        """Declare and parse all orchestrator ROS 2 parameters from node handle."""
        node.declare_parameter("board_frame", "chessboard_frame")
        node.declare_parameter("map_frame", "map")
        node.declare_parameter("service_timeout_sec", 5.0)
        node.declare_parameter("action_timeout_sec", 60.0)
        node.declare_parameter("readiness_timeout_sec", 1.0)
        node.declare_parameter("chassis_settle_timeout_sec", 2.0)
        node.declare_parameter(
            "navigation", True, ParameterDescriptor(dynamic_typing=True)
        )
        node.declare_parameter(
            "record", False, ParameterDescriptor(dynamic_typing=True)
        )
        node.declare_parameter("robot_color", "white")
        node.declare_parameter("think_time_ms", 1000)

        node.declare_parameter("recovery.auto_recovery_enabled", True)
        node.declare_parameter("recovery.auto_recovery_timeout_sec", 5.0)
        node.declare_parameter("recovery.max_recovery_attempts", 3)

        node.declare_parameter("topics.nav_ready", "/system/nav_ready")
        node.declare_parameter("topics.grasp_ready", "/system/grasp_ready")
        node.declare_parameter("topics.game_status", "/chess/game_status")
        node.declare_parameter("topics.candidate_move", "/chess/candidate_move")
        node.declare_parameter("topics.perception_context", "/perception_context")
        node.declare_parameter("topics.diagnostics", "/diagnostics")

        node.declare_parameter(
            "services.check_feasibility", "/workspace/check_move_feasibility"
        )
        node.declare_parameter("services.recover", "/orchestrator/recover")
        node.declare_parameter("services.import_fen", "/orchestrator/import_fen")
        node.declare_parameter("services.export_fen", "/orchestrator/export_fen")
        node.declare_parameter(
            "actions.execute_collection_task",
            "/orchestrator/execute_collection_task",
        )
        node.declare_parameter("services.chess_set_fen", "/chess/set_fen")
        node.declare_parameter(
            "actions.chess_compute_best_move", "/chess/compute_best_move"
        )
        node.declare_parameter(
            "services.set_perception_context",
            "/orchestrator/set_perception_context",
        )
        node.declare_parameter(
            "services.system_reset_readiness",
            "/system/reset_readiness",
        )
        node.declare_parameter(
            "services.ekf_reset",
            "/ekf_filter_node_map/reset",
        )

        node.declare_parameter("disturbance.local_odom_topic", "/odometry/local")
        node.declare_parameter("disturbance.global_odom_topic", "/odometry/global")
        node.declare_parameter("disturbance.uncommanded_linear_thresh", 0.03)
        node.declare_parameter("disturbance.uncommanded_angular_thresh", 0.08)
        node.declare_parameter("disturbance.uncommanded_confirm_duration_sec", 0.20)

        node.declare_parameter("actions.navigate_to_pose", "/navigate_to_pose")
        node.declare_parameter(
            "actions.execute_chess_move", "/manipulation/execute_chess_move"
        )

        node.declare_parameter("observation.standoff_distance", 0.65)
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

        self.board_frame = str(node.get_parameter("board_frame").value)
        self.map_frame = str(node.get_parameter("map_frame").value)
        self.service_timeout_sec = float(
            node.get_parameter("service_timeout_sec").value
        )
        self.action_timeout_sec = float(node.get_parameter("action_timeout_sec").value)
        self.readiness_timeout_sec = readiness_to
        self.chassis_settle_timeout_sec = float(
            node.get_parameter("chassis_settle_timeout_sec").value
        )
        raw_nav = node.get_parameter("navigation").value
        self.navigation = (
            str(raw_nav).strip().lower() in ("true", "1", "yes")
            if isinstance(raw_nav, str)
            else bool(raw_nav)
        )
        raw_record = node.get_parameter("record").value
        self.record = (
            str(raw_record).strip().lower() in ("true", "1", "yes")
            if isinstance(raw_record, str)
            else bool(raw_record)
        )
        raw_color = str(node.get_parameter("robot_color").value).strip().lower()
        self.robot_color = "b" if raw_color in ("b", "black") else "w"
        self.think_time_ms = int(node.get_parameter("think_time_ms").value)

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
        self.candidate_move_topic = str(node.get_parameter("topics.candidate_move").value)
        self.perception_context_topic = str(
            node.get_parameter("topics.perception_context").value
        )
        self.diagnostics_topic = str(node.get_parameter("topics.diagnostics").value)

        self.check_feasibility_srv = str(
            node.get_parameter("services.check_feasibility").value
        )
        self.recover_srv = str(node.get_parameter("services.recover").value)
        self.import_fen_srv = str(node.get_parameter("services.import_fen").value)
        self.export_fen_srv = str(node.get_parameter("services.export_fen").value)
        self.execute_collection_task_action = str(
            node.get_parameter("actions.execute_collection_task").value
        )
        self.chess_set_fen_srv = str(node.get_parameter("services.chess_set_fen").value)
        self.chess_compute_best_move_action = str(
            node.get_parameter("actions.chess_compute_best_move").value
        )
        self.set_perception_srv = str(
            node.get_parameter("services.set_perception_context").value
        )
        self.system_reset_readiness_srv = str(
            node.get_parameter("services.system_reset_readiness").value
        )
        self.ekf_reset_srv = str(node.get_parameter("services.ekf_reset").value)

        self.disturbance_local_odom_topic = str(
            node.get_parameter("disturbance.local_odom_topic").value
        )
        self.disturbance_global_odom_topic = str(
            node.get_parameter("disturbance.global_odom_topic").value
        )
        self.disturbance_uncommanded_linear_thresh = float(
            node.get_parameter("disturbance.uncommanded_linear_thresh").value
        )
        self.disturbance_uncommanded_angular_thresh = float(
            node.get_parameter("disturbance.uncommanded_angular_thresh").value
        )
        self.disturbance_uncommanded_confirm_duration_sec = float(
            node.get_parameter("disturbance.uncommanded_confirm_duration_sec").value
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


