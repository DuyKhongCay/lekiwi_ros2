# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Board geometry domain: SE(2) planar mathematics and observation standoff.

Provides planar quaternion conversions, radial circle boundaries, and azimuth
candidate ranking for active perception vantage point planning.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion

# ==============================================================================
# SE(2) Primitives
# ==============================================================================


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert planar yaw angle in radians to a geometry_msgs Quaternion."""
    half_yaw = yaw * 0.5
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(half_yaw),
        w=math.cos(half_yaw),
    )


def normalize_angle(angle: float) -> float:
    """Normalize any planar angle in radians to the principal interval [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _create_standoff_pose(
    target_x: float,
    target_y: float,
    facing_yaw: float,
    frame_id: str,
) -> PoseStamped:
    """Construct a PoseStamped with position (target_x, target_y) and heading facing_yaw."""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose = Pose(
        position=Point(x=target_x, y=target_y, z=0.0),
        orientation=yaw_to_quaternion(facing_yaw),
    )
    return pose


# ==============================================================================
# Radial Standoff Guard
# ==============================================================================


def is_on_standoff_circle(
    x: float,
    y: float,
    standoff_radius: float,
    tolerance: float = 0.04,
) -> bool:
    """Evaluate whether planar coordinates (x, y) lie on the standoff circle.

    Prevents redundant base navigation if the robot is already within viewing tolerance.
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
    """Generate target entry pose on standoff circle along the radial vector.

    Orients robot heading inward facing the board center to re-acquire visual markers.

    Args:
        curr_x: Current X position in `board_frame` (meters).
        curr_y: Current Y position in `board_frame` (meters).
        standoff_radius: Target observation circle radius R from board origin (meters).
        board_frame: Coordinate frame ID of the chessboard.
        fallback_yaw: Fallback radial direction if current position is exactly at origin.

    Returns:
        PoseStamped representing the radial entry waypoint facing board center.
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
    """Rank candidate viewpoints by shortest geodesic distance along the standoff circle.

    Minimizes mobile base transit time by selecting the closest angular vantage point first.

    Args:
        curr_x: Current X position in `board_frame` (meters).
        curr_y: Current Y position in `board_frame` (meters).
        candidate_offsets: Candidate azimuth angles (radians) in `board_frame`.
        standoff_radius: Observation radius R (meters).
        board_frame: Coordinate frame ID of the chessboard.
        fallback_yaw: Fallback angle if current robot position is at board origin.

    Returns:
        List of RankedViewpoint instances sorted ascending by arc_cost (closest first).
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
