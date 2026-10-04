# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""
Board geometry domain: SE(2) planar mathematics, radial standoff guards,
and azimuth viewpoint candidate ranking for autonomous chess robot.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion


# ==============================================================================
# SE(2) Primitives
# ==============================================================================


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert planar yaw angle (radians) to geometry_msgs Quaternion."""
    half_yaw = yaw * 0.5
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(half_yaw),
        w=math.cos(half_yaw),
    )


def normalize_angle(angle: float) -> float:
    """Normalize any planar angle (radians) to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def get_default_approach_yaw(robot_color: str) -> float:
    """Return default observation approach yaw based on robot color (-pi/2 for White, pi/2 for Black)."""
    return -math.pi / 2.0 if str(robot_color).lower().startswith("w") else math.pi / 2.0


default_approach_yaw = get_default_approach_yaw


def _create_standoff_pose(
    target_x: float,
    target_y: float,
    facing_yaw: float,
    frame_id: str,
) -> PoseStamped:
    """Internal helper to construct a stamped planar pose facing the target."""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose = Pose(
        position=Point(x=target_x, y=target_y, z=0.0),
        orientation=yaw_to_quaternion(facing_yaw),
    )
    return pose


# ==============================================================================
# Coordinate Frame Transforms
# ==============================================================================


def transform_board_pose_to_map(
    pose_board: PoseStamped,
    map_frame: str = "map",
    board_x: float = 0.0,
    board_y: float = 0.0,
    board_yaw: float = 0.0,
    tf_buffer: Any | None = None,
    logger: Any | None = None,
) -> PoseStamped:
    """
    Transform a PoseStamped from chessboard_frame to map frame.

    Uses tf2_ros.Buffer if available and functional. If TF transform fails,
    logs an explicit warning and falls back to SE(2) planar transform with yaw facing the chessboard center.
    """
    if tf_buffer is not None:
        try:
            return tf_buffer.transform(pose_board, map_frame)
        except Exception as ex:
            if logger is not None:
                src_frame = getattr(pose_board.header, "frame_id", "chessboard_frame")
                logger.warn(
                    f"[TF TRANSFORM FALLBACK] tf_buffer.transform from '{src_frame}' to '{map_frame}' "
                    f"failed: {ex}. Falling back to planar SE(2) transform with board pose "
                    f"({board_x:.3f}, {board_y:.3f}, yaw={board_yaw:.3f})."
                )
                if abs(board_x) < 1e-4 and abs(board_y) < 1e-4 and abs(board_yaw) < 1e-4:
                    logger.warn(
                        "[TF TRANSFORM FALLBACK] Board pose reference is at origin (0, 0, 0). "
                        "Planar map coordinates may deviate from real workspace."
                    )

    cos_b = math.cos(board_yaw)
    sin_b = math.sin(board_yaw)
    bx = float(pose_board.pose.position.x)
    by = float(pose_board.pose.position.y)

    map_x = board_x + (bx * cos_b - by * sin_b)
    map_y = board_y + (bx * sin_b + by * cos_b)

    facing_board_yaw = math.atan2(-by, -bx)
    facing_map_yaw = facing_board_yaw + board_yaw

    return _create_standoff_pose(
        target_x=map_x,
        target_y=map_y,
        facing_yaw=facing_map_yaw,
        frame_id=map_frame,
    )


# ==============================================================================
# Radial Standoff Guard
# ==============================================================================


def is_on_standoff_circle(
    x: float,
    y: float,
    standoff_radius: float,
    tolerance: float = 0.04,
) -> bool:
    """
    Evaluate if planar position (x, y) lies on the observation circle of radius standoff_radius.

    :param x: X position in chessboard_frame (meters).
    :param y: Y position in chessboard_frame (meters).
    :param standoff_radius: Target observation circle radius R (meters).
    :param tolerance: Radial tolerance band delta_R (meters, default 4cm).
    :return: True if |sqrt(x^2 + y^2) - standoff_radius| <= tolerance.
    """
    r = math.hypot(x, y)
    return abs(r - standoff_radius) <= tolerance


def compute_radial_entry_pose(
    curr_x: float,
    curr_y: float,
    standoff_radius: float,
    board_frame: str = "chessboard_frame",
    fallback_yaw: float = 0.0,
) -> PoseStamped:
    """
    Generate target entry pose on standoff circle along radial vector from origin through (curr_x, curr_y).

    Robot heading is oriented inward facing the chessboard center.
    If robot is exactly at origin, uses fallback_yaw as the radial direction.
    """
    dist = math.hypot(curr_x, curr_y)
    azimuth = math.atan2(curr_y, curr_x) if dist > 1e-4 else fallback_yaw

    target_x = standoff_radius * math.cos(azimuth)
    target_y = standoff_radius * math.sin(azimuth)
    facing_yaw = math.atan2(-target_y, -target_x)

    return _create_standoff_pose(
        target_x=target_x,
        target_y=target_y,
        facing_yaw=facing_yaw,
        frame_id=board_frame,
    )


# ==============================================================================
# Azimuth Viewpoint Scheduler
# ==============================================================================


@dataclass(frozen=True)
class RankedViewpoint:
    """Evaluated observation viewpoint ranked by azimuthal travel cost."""

    angle_offset: float
    pose_in_board: PoseStamped
    angular_distance: float
    arc_cost: float


def rank_by_azimuth(
    curr_x: float,
    curr_y: float,
    candidate_offsets: list[float],
    standoff_radius: float,
    board_frame: str = "chessboard_frame",
    fallback_yaw: float = 0.0,
) -> list[RankedViewpoint]:
    """
    Rank candidate viewpoints by shortest azimuthal distance from current polar angle.

    :param curr_x: Current X position in chessboard_frame.
    :param curr_y: Current Y position in chessboard_frame.
    :param candidate_offsets: List of candidate azimuth angles (radians) in chessboard_frame.
    :param standoff_radius: Observation radius R (meters).
    :param board_frame: Frame ID of the chessboard.
    :param fallback_yaw: Fallback angle if current position is at origin.
    :return: List of RankedViewpoint sorted ascending by arc_cost (closest first).
    """
    dist = math.hypot(curr_x, curr_y)
    curr_azimuth = math.atan2(curr_y, curr_x) if dist > 1e-4 else fallback_yaw

    ranked: list[RankedViewpoint] = []
    for offset in candidate_offsets:
        delta_theta = abs(normalize_angle(offset - curr_azimuth))
        arc_cost = standoff_radius * delta_theta

        target_x = standoff_radius * math.cos(offset)
        target_y = standoff_radius * math.sin(offset)
        facing_yaw = math.atan2(-target_y, -target_x)

        pose = _create_standoff_pose(
            target_x=target_x,
            target_y=target_y,
            facing_yaw=facing_yaw,
            frame_id=board_frame,
        )

        ranked.append(
            RankedViewpoint(
                angle_offset=offset,
                pose_in_board=pose,
                angular_distance=delta_theta,
                arc_cost=arc_cost,
            )
        )

    ranked.sort(key=lambda vp: vp.arc_cost)
    return ranked
