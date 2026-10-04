# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""
Active Observation Navigator with Two-Phase Standoff Recovery.

Provides:
- Radial Standoff Guard ensuring robot is on standoff circle S^1 around chessboard.
- Azimuth Cost Viewpoint Navigation ranking candidates on S^1 by geodesic distance.
"""

from __future__ import annotations

import math
import threading
from collections.abc import Callable, Sequence
from typing import TYPE_CHECKING

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from lekiwi_orchestrator.board_geometry import (
    compute_radial_entry_pose,
    get_default_approach_yaw,
    is_on_standoff_circle,
    rank_by_azimuth,
    transform_board_pose_to_map,
)
from lekiwi_orchestrator.mission_types import ActionResult, ObservationIntent

if TYPE_CHECKING:
    from lekiwi_orchestrator.motion_client import NavigationClient


class ObsNavigator:
    """
    Coordinates robot base relocation to alternative observation viewpoints
    when chessboard perception is occluded (POST_MOVE_VERIFY) or EKF drifts (RELOCALIZE).

    Enforces 2-Phase Geometric Strategy:
    1. Radial Standoff Guard: Validates that the robot lies on the standoff circle
       of radius R around the chessboard center. If inside (e.g. grasping r < R) or
       outside, a radial retreat goal is dispatched first.
    2. Azimuth Cost Viewpoint Navigation: Ranks candidate viewpoints by shortest
       arc distance on S^1 from the robot's current polar angle, choosing the
       closest vantage point first.
    """

    def __init__(
        self,
        node: Node,
        dispatcher: NavigationClient,
        map_frame: str = "map",
        board_frame: str = "chessboard_frame",
        standoff_distance: float = 0.65,
        radius_tolerance: float = 0.04,
        robot_color: str = "b",
        angle_offsets_map: dict[ObservationIntent, Sequence[float]] | None = None,
        tf_buffer: tf2_ros.Buffer | None = None,
    ) -> None:
        self._node = node
        self._dispatcher = dispatcher
        self._map_frame = map_frame
        self._board_frame = board_frame
        self._standoff_distance = standoff_distance
        self._radius_tolerance = radius_tolerance
        self._robot_color = robot_color
        self._tf_buffer = tf_buffer
        self._lock = threading.RLock()

        default_verify = [0.0, -0.314, 0.314, -0.558, 0.558]
        default_reloc = [0.0, -0.314, 0.314]

        self._angle_offsets_map = {
            ObservationIntent.POST_MOVE_VERIFY: list(
                angle_offsets_map.get(
                    ObservationIntent.POST_MOVE_VERIFY, default_verify
                )
                if angle_offsets_map
                else default_verify
            ),
            ObservationIntent.RELOCALIZE: list(
                angle_offsets_map.get(
                    ObservationIntent.RELOCALIZE, default_reloc
                )
                if angle_offsets_map
                else default_reloc
            ),
        }

        self._viewpoint_indices: dict[ObservationIntent, int] = {
            ObservationIntent.POST_MOVE_VERIFY: 0,
            ObservationIntent.RELOCALIZE: 0,
        }
        self._last_reference_pose: PoseStamped | None = None

    @property
    def standoff_distance(self) -> float:
        return self._standoff_distance

    @property
    def viewpoint_index(self) -> int:
        with self._lock:
            return self._viewpoint_indices[ObservationIntent.POST_MOVE_VERIFY]

    def get_viewpoint_index(self, intent: ObservationIntent) -> int:
        with self._lock:
            return self._viewpoint_indices.get(intent, 0)

    @property
    def robot_color(self) -> str:
        return self._robot_color

    @robot_color.setter
    def robot_color(self, color: str) -> None:
        self._robot_color = color

    def get_max_attempts(self, intent: ObservationIntent) -> int:
        return len(self._angle_offsets_map.get(intent, []))

    def has_exhausted_viewpoints(self, intent: ObservationIntent) -> bool:
        with self._lock:
            return self._viewpoint_indices.get(intent, 0) >= self.get_max_attempts(intent)

    def reset_viewpoint_index(self, intent: ObservationIntent | None = None) -> None:
        with self._lock:
            if intent is not None:
                self._viewpoint_indices[intent] = 0
            else:
                for k in self._viewpoint_indices:
                    self._viewpoint_indices[k] = 0

    def cancel(self) -> None:
        self._dispatcher.cancel_active_goal()

    def get_current_board_position(
        self,
        reference_pose: PoseStamped | None = None,
        board_x: float = 0.0,
        board_y: float = 0.0,
        board_yaw: float = 0.0,
    ) -> tuple[float, float]:
        """Resolve current robot base position (x, y) relative to chessboard_frame."""
        if self._tf_buffer is not None:
            try:
                t = self._tf_buffer.lookup_transform(
                    self._board_frame,
                    "base_footprint",
                    rclpy.time.Time(),
                )
                return float(t.transform.translation.x), float(t.transform.translation.y)
            except Exception as e:
                self._node.get_logger().warn(
                    f"TF lookup failed between '{self._board_frame}' and 'base_footprint': {e}. Falling back to reference pose.",
                    throttle_duration_sec=5.0,
                )

        if reference_pose is not None:
            frame_id = getattr(getattr(reference_pose, "header", None), "frame_id", "")
            px = float(reference_pose.pose.position.x)
            py = float(reference_pose.pose.position.y)
            if frame_id == self._board_frame:
                return px, py
            dx = px - board_x
            dy = py - board_y
            cos_b = math.cos(-board_yaw)
            sin_b = math.sin(-board_yaw)
            return (dx * cos_b - dy * sin_b, dx * sin_b + dy * cos_b)

        base_angle = get_default_approach_yaw(self._robot_color)
        return (
            self._standoff_distance * math.cos(base_angle),
            self._standoff_distance * math.sin(base_angle),
        )

    def transform_board_pose_to_map(
        self,
        pose_board: PoseStamped,
        board_x: float = 0.0,
        board_y: float = 0.0,
        board_yaw: float = 0.0,
    ) -> PoseStamped:
        """Transform a PoseStamped from chessboard_frame to map frame."""
        logger = self._node.get_logger() if hasattr(self._node, "get_logger") else None
        return transform_board_pose_to_map(
            pose_board=pose_board,
            map_frame=self._map_frame,
            board_x=board_x,
            board_y=board_y,
            board_yaw=board_yaw,
            tf_buffer=self._tf_buffer,
            logger=logger,
        )

    def _dispatch_azimuth_viewpoint(
        self,
        intent: ObservationIntent,
        curr_bx: float,
        curr_by: float,
        board_x: float = 0.0,
        board_y: float = 0.0,
        board_yaw: float = 0.0,
        timeout_sec: float = 60.0,
        on_completed: Callable[[ActionResult], None] | None = None,
    ) -> bool:
        """Phase 2: Rank candidate viewpoints by Azimuth cost and dispatch optimal goal."""
        offsets = self._angle_offsets_map.get(intent, [0.0])
        if not offsets:
            return False

        fallback_yaw = get_default_approach_yaw(self._robot_color)
        ranked = rank_by_azimuth(
            curr_x=curr_bx,
            curr_y=curr_by,
            candidate_offsets=offsets,
            standoff_radius=self._standoff_distance,
            board_frame=self._board_frame,
            fallback_yaw=fallback_yaw,
        )
        if not ranked:
            return False

        with self._lock:
            idx = self._viewpoint_indices[intent]
            self._viewpoint_indices[intent] += 1

        selected_vp = ranked[idx % len(ranked)]
        target_pose_map = self.transform_board_pose_to_map(
            selected_vp.pose_in_board,
            board_x=board_x,
            board_y=board_y,
            board_yaw=board_yaw,
        )

        intent_name = getattr(intent, "value", str(intent))
        self._node.get_logger().info(
            f"[ACTIVE PERCEPTION] Repositioning base for {intent_name} "
            f"viewpoint #{idx + 1}/{len(offsets)} (azimuth={math.degrees(selected_vp.angle_offset):.1f} deg, "
            f"arc_dist={selected_vp.arc_cost:.3f}m) to ({target_pose_map.pose.position.x:.3f}, {target_pose_map.pose.position.y:.3f})..."
        )

        return self._dispatcher.send_navigation_goal(
            target_pose=target_pose_map,
            timeout_sec=timeout_sec,
            on_completed=on_completed,
        )

    def reposition_to_next_viewpoint(
        self,
        intent: ObservationIntent = ObservationIntent.POST_MOVE_VERIFY,
        reference_pose: PoseStamped | None = None,
        board_x: float = 0.0,
        board_y: float = 0.0,
        board_yaw: float = 0.0,
        timeout_sec: float = 60.0,
        on_completed: Callable[[ActionResult], None] | None = None,
    ) -> bool:
        """
        Reposition base executing 2-Phase Strategy:
        1. Radial Standoff Guard: Ensure robot is on standoff circle of radius R.
        2. Azimuth Cost Navigation: Navigate to lowest arc cost viewpoint.
        """
        offsets = self._angle_offsets_map.get(intent, [0.0])
        if not offsets:
            return False

        ref = reference_pose or self._last_reference_pose
        curr_bx, curr_by = self.get_current_board_position(
            reference_pose=ref,
            board_x=board_x,
            board_y=board_y,
            board_yaw=board_yaw,
        )
        on_circle = is_on_standoff_circle(
            curr_bx,
            curr_by,
            standoff_radius=self._standoff_distance,
            tolerance=self._radius_tolerance,
        )

        if not on_circle:
            fallback_yaw = get_default_approach_yaw(self._robot_color)
            radial_pose_board = compute_radial_entry_pose(
                curr_x=curr_bx,
                curr_y=curr_by,
                standoff_radius=self._standoff_distance,
                board_frame=self._board_frame,
                fallback_yaw=fallback_yaw,
            )
            radial_pose_map = self.transform_board_pose_to_map(
                radial_pose_board,
                board_x=board_x,
                board_y=board_y,
                board_yaw=board_yaw,
            )
            r_curr = math.hypot(curr_bx, curr_by)
            self._node.get_logger().info(
                f"[RADIAL STANDOFF GUARD] Robot at radius {r_curr:.3f}m is not on standoff circle "
                f"({self._standoff_distance:.3f}m +/- {self._radius_tolerance:.3f}m). "
                f"Executing radial retreat to ({radial_pose_map.pose.position.x:.3f}, {radial_pose_map.pose.position.y:.3f}) first..."
            )

            def _on_radial_completed(res: ActionResult) -> None:
                if not res.success:
                    self._node.get_logger().error(
                        f"[RADIAL STANDOFF GUARD] Failed to reach standoff circle: {res.message}"
                    )
                    if on_completed:
                        on_completed(res)
                    return
                self._dispatch_azimuth_viewpoint(
                    intent=intent,
                    curr_bx=radial_pose_board.pose.position.x,
                    curr_by=radial_pose_board.pose.position.y,
                    board_x=board_x,
                    board_y=board_y,
                    board_yaw=board_yaw,
                    timeout_sec=timeout_sec,
                    on_completed=on_completed,
                )

            return self._dispatcher.send_navigation_goal(
                target_pose=radial_pose_map,
                timeout_sec=timeout_sec,
                on_completed=_on_radial_completed,
            )

        return self._dispatch_azimuth_viewpoint(
            intent=intent,
            curr_bx=curr_bx,
            curr_by=curr_by,
            board_x=board_x,
            board_y=board_y,
            board_yaw=board_yaw,
            timeout_sec=timeout_sec,
            on_completed=on_completed,
        )


__all__ = [
    "ObsNavigator",
]
