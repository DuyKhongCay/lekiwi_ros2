# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""
Active Observation Navigator with Geodesic Viewpoint Scheduling.

Provides:
- Geodesic Viewpoint Navigation ranking calibrated candidates on S^1 by arc distance.
- Direct-to-viewpoint positioning bypassing uncalibrated intermediate radial waypoints.
"""

from __future__ import annotations

import math
import threading
from collections.abc import Callable, Sequence
from typing import TYPE_CHECKING

import rclpy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from lekiwi_orchestrator.board_geometry import rank_by_azimuth
from lekiwi_orchestrator.mission_types import ActionResult, ObservationIntent

if TYPE_CHECKING:
    from lekiwi_orchestrator.motion_client import NavigationClient


class ObsNavigator:
    """
    Coordinates robot base relocation to calibrated observation viewpoints
    when chessboard perception is occluded (POST_MOVE_VERIFY) or EKF drifts (RELOCALIZE).

    Enforces direct geodesic viewpoint positioning:
    - Ranks whitelisted candidate viewpoints in `angle_offsets` by shortest geodesic
      arc distance along the standoff circle from the robot's current polar coordinates.
    - Eliminates uncalibrated radial waypoints, ensuring every observation pose corresponds
      to a tested, occlusion-free sensor viewing angle.
    """

    def __init__(
        self,
        node: Node,
        dispatcher: NavigationClient,
        map_frame: str = "map",
        board_frame: str = "chessboard_frame",
        standoff_distance: float = 0.65,
        angle_offsets_map: dict[ObservationIntent, Sequence[float]] | None = None,
        tf_buffer: tf2_ros.Buffer | None = None,
    ) -> None:
        self._node = node
        self._dispatcher = dispatcher
        self._map_frame = map_frame
        self._board_frame = board_frame
        self._standoff_distance = standoff_distance
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
                angle_offsets_map.get(ObservationIntent.RELOCALIZE, default_reloc)
                if angle_offsets_map
                else default_reloc
            ),
        }

        self._viewpoint_indices: dict[ObservationIntent, int] = {
            ObservationIntent.POST_MOVE_VERIFY: 0,
            ObservationIntent.RELOCALIZE: 0,
        }

    @property
    def standoff_distance(self) -> float:
        """Return the configured radial standoff viewing distance from board center (meters)."""
        return self._standoff_distance

    @property
    def viewpoint_index(self) -> int:
        """Return the current candidate viewpoint index for post-move verification under lock."""
        with self._lock:
            return self._viewpoint_indices[ObservationIntent.POST_MOVE_VERIFY]

    def get_viewpoint_index(self, intent: ObservationIntent) -> int:
        """Return the active viewpoint index for the specified observation intent."""
        with self._lock:
            return self._viewpoint_indices.get(intent, 0)

    def get_max_attempts(self, intent: ObservationIntent) -> int:
        """Return the total number of candidate angular offsets for the specified intent."""
        return len(self._angle_offsets_map.get(intent, []))

    def has_exhausted_viewpoints(self, intent: ObservationIntent) -> bool:
        """Return True if all configured angular offsets for the intent have been attempted."""
        with self._lock:
            return self._viewpoint_indices.get(intent, 0) >= self.get_max_attempts(
                intent
            )

    def reset_viewpoint_index(self, intent: ObservationIntent | None = None) -> None:
        """Reset observation viewpoint index for a specific intent or all intents."""
        with self._lock:
            if intent is not None:
                self._viewpoint_indices[intent] = 0
            else:
                for k in self._viewpoint_indices:
                    self._viewpoint_indices[k] = 0

    def cancel(self) -> None:
        """Cancel any ongoing navigation action dispatched by this observer."""
        self._dispatcher.cancel_active_goal()

    def pose_to_board_xy(self, pose: PoseStamped) -> tuple[float, float]:
        """Convert a PoseStamped in any frame to planar (x, y) coordinates in `chessboard_frame`."""
        frame_id = getattr(getattr(pose, "header", None), "frame_id", "")
        if frame_id == self._board_frame:
            return float(pose.pose.position.x), float(pose.pose.position.y)

        if self._tf_buffer is not None:
            try:
                transformed = self._tf_buffer.transform(pose, self._board_frame)
                return float(transformed.pose.position.x), float(
                    transformed.pose.position.y
                )
            except Exception as e:
                self._node.get_logger().warn(
                    f"TF transform pose to '{self._board_frame}' failed: {e}. Using raw coordinates.",
                    throttle_duration_sec=5.0,
                )
        return float(pose.pose.position.x), float(pose.pose.position.y)

    def get_current_board_position(
        self,
        reference_pose: PoseStamped | None = None,
    ) -> tuple[float, float]:
        """Resolve robot base planar position (x, y) relative to `chessboard_frame`."""
        if reference_pose is not None:
            return self.pose_to_board_xy(reference_pose)

        if self._tf_buffer is not None:
            try:
                t = self._tf_buffer.lookup_transform(
                    self._board_frame,
                    "base_footprint",
                    rclpy.time.Time(),
                )
                return float(t.transform.translation.x), float(
                    t.transform.translation.y
                )
            except Exception as e:
                self._node.get_logger().warn(
                    f"TF lookup failed between '{self._board_frame}' and 'base_footprint': {e}. Using fallback coordinates.",
                    throttle_duration_sec=5.0,
                )

        return (self._standoff_distance, 0.0)

    def compute_observation_pose_for_base(
        self,
        base_pose: PoseStamped | None,
        intent: ObservationIntent = ObservationIntent.POST_MOVE_VERIFY,
    ) -> PoseStamped | None:
        """Compute the closest calibrated observation viewpoint from angle_offsets for a base pose.

        Projects the base interaction coordinates onto the standoff circle S^1, ranks candidate
        viewpoints by geodesic distance, and selects the closest whitelisted viewpoint (index 0).
        Advances `viewpoint_indices[intent]` to 1 so that subsequent timeout repositioning picks
        the next distinct vantage point.
        """
        if base_pose is not None:
            bx, by = self.pose_to_board_xy(base_pose)
        else:
            bx, by = self.get_current_board_position()

        offsets = self._angle_offsets_map.get(intent, [0.0])
        if not offsets:
            return None

        ranked = rank_by_azimuth(
            curr_x=bx,
            curr_y=by,
            candidate_offsets=offsets,
            standoff_radius=self._standoff_distance,
            board_frame=self._board_frame,
            fallback_yaw=0.0,
        )
        if not ranked:
            return None

        primary_vp = ranked[0]
        with self._lock:
            self._viewpoint_indices[intent] = 1

        return self.transform_board_pose_to_map(primary_vp.pose_in_board)

    def transform_board_pose_to_map(
        self,
        pose_board: PoseStamped,
    ) -> PoseStamped | None:
        """Transform a PoseStamped from `chessboard_frame` to `map_frame` via TF2 buffer."""
        if self._tf_buffer is not None:
            try:
                return self._tf_buffer.transform(
                    pose_board,
                    self._map_frame,
                )
            except Exception as ex:
                self._node.get_logger().error(
                    f"[ACTIVE PERCEPTION] TF transform from '{pose_board.header.frame_id}' "
                    f"to '{self._map_frame}' failed: {ex}"
                )
                return None

        pose_map = PoseStamped()
        pose_map.header.frame_id = self._map_frame
        pose_map.header.stamp = pose_board.header.stamp
        pose_map.pose = pose_board.pose
        return pose_map

    def _dispatch_azimuth_viewpoint(
        self,
        intent: ObservationIntent,
        curr_bx: float,
        curr_by: float,
        timeout_sec: float = 60.0,
        on_completed: Callable[[ActionResult], None] | None = None,
    ) -> bool:
        """Rank candidate observation viewpoints by geodesic distance and dispatch navigation."""
        offsets = self._angle_offsets_map.get(intent, [0.0])
        if not offsets:
            return False

        ranked = rank_by_azimuth(
            curr_x=curr_bx,
            curr_y=curr_by,
            candidate_offsets=offsets,
            standoff_radius=self._standoff_distance,
            board_frame=self._board_frame,
            fallback_yaw=0.0,
        )
        if not ranked:
            return False

        with self._lock:
            idx = self._viewpoint_indices[intent]
            self._viewpoint_indices[intent] += 1

        selected_vp = ranked[idx % len(ranked)]
        target_pose_map = self.transform_board_pose_to_map(selected_vp.pose_in_board)
        if target_pose_map is None:
            self._node.get_logger().error(
                "[ACTIVE PERCEPTION] Failed to transform viewpoint to map frame. Aborting goal dispatch."
            )
            return False

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
        timeout_sec: float = 60.0,
        on_completed: Callable[[ActionResult], None] | None = None,
    ) -> bool:
        """Dispatch navigation directly to the next calibrated viewpoint on the circle.

        Evaluates geodesic distance from current polar coordinates to whitelisted
        candidate offsets, choosing the next sequential vantage point in ranked order.
        """
        curr_bx, curr_by = self.get_current_board_position(reference_pose=reference_pose)
        return self._dispatch_azimuth_viewpoint(
            intent=intent,
            curr_bx=curr_bx,
            curr_by=curr_by,
            timeout_sec=timeout_sec,
            on_completed=on_completed,
        )


__all__ = [
    "ObsNavigator",
]
