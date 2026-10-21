# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Board geometry domain: Observation standoff and azimuth viewpoint scheduling."""

from __future__ import annotations

import math
from dataclasses import dataclass

from geometry_msgs.msg import PoseStamped

from lekiwi_orchestrator.core.utils import create_pose2d_stamped, normalize_angle


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
    cumulative_cost: float = 0.0


def _resolve_azimuth(curr_x: float, curr_y: float, fallback_yaw: float) -> float:
    """Resolve current azimuth angle, falling back if at origin."""
    return math.atan2(curr_y, curr_x) if math.hypot(curr_x, curr_y) > 1e-4 else fallback_yaw


def _build_viewpoint(
    target_offset: float,
    ref_azimuth: float,
    standoff_radius: float,
    board_frame: str,
    cum_cost: float,
) -> RankedViewpoint:
    """Construct a RankedViewpoint computed relative to the preceding reference azimuth."""
    delta_theta = abs(normalize_angle(target_offset - ref_azimuth))
    step_cost = standoff_radius * delta_theta

    target_x = standoff_radius * math.cos(target_offset)
    target_y = standoff_radius * math.sin(target_offset)
    facing_yaw = math.atan2(-target_y, -target_x)

    pose = create_pose2d_stamped(
        x=target_x,
        y=target_y,
        yaw=facing_yaw,
        frame_id=board_frame,
    )

    return RankedViewpoint(
        angle_offset=target_offset,
        pose_in_board=pose,
        angular_distance=delta_theta,
        arc_cost=step_cost,
        cumulative_cost=cum_cost + step_cost,
    )


def _generate_candidate_paths(pts: list[float]):
    """Generate cyclic CCW, CW, and single-turnaround candidate sweep tours."""
    n = len(pts)
    doubled = pts + pts
    for cut in range(n):
        arc = doubled[cut + 1 : cut + 1 + n]
        yield arc
        yield arc[::-1]
        for k in range(n - 1):
            yield arc[k::-1] + arc[k + 1 :]
            yield arc[k + 1 :] + arc[k::-1]


def _compute_path_cost(path: list[float], curr_azimuth: float) -> tuple[float, float]:
    """Compute total angular arc cost and initial displacement for an azimuth tour."""
    first_step = abs(normalize_angle(path[0] - curr_azimuth))
    cost = first_step + sum(
        abs(normalize_angle(path[i + 1] - path[i])) for i in range(len(path) - 1)
    )
    return cost, first_step


def _find_shortest_open_tour(
    candidate_offsets: list[float], curr_azimuth: float
) -> list[float]:
    """Determine candidate permutation with minimal cumulative angular distance."""
    if len(candidate_offsets) <= 1:
        return list(candidate_offsets)

    pts = sorted(
        candidate_offsets,
        key=lambda c: (c - curr_azimuth) % (2.0 * math.pi),
    )
    min_cost = float("inf")
    min_first_step = float("inf")
    best_perm = list(candidate_offsets)

    for p in _generate_candidate_paths(pts):
        cost, first_step = _compute_path_cost(p, curr_azimuth)
        is_strictly_better = cost < min_cost - 1e-9
        is_tied_with_less_first_step = (
            abs(cost - min_cost) <= 1e-9 and first_step < min_first_step
        )
        if is_strictly_better or is_tied_with_less_first_step:
            min_cost = cost
            min_first_step = first_step
            best_perm = list(p)

    return best_perm


def rank_by_azimuth(
    curr_x: float,
    curr_y: float,
    candidate_offsets: list[float],
    standoff_radius: float,
    board_frame: str = "chessboard_frame",
    fallback_yaw: float = 0.0,
) -> list[RankedViewpoint]:
    """Rank candidate viewpoints as a global shortest open tour along the standoff perimeter.

    Finds the exact permutation of candidate offsets that minimizes total arc travel
    distance from the robot's starting azimuth, avoiding back-and-forth oscillations.

    Args:
        curr_x: Current X position in `board_frame` (meters).
        curr_y: Current Y position in `board_frame` (meters).
        candidate_offsets: Candidate azimuth angles (radians) in `board_frame`.
        standoff_radius: Observation radius R (meters).
        board_frame: Coordinate frame ID of the chessboard.
        fallback_yaw: Fallback angle if current robot position is at board origin.

    Returns:
        List of RankedViewpoint instances ordered as a global minimal-distance tour.
    """
    if not candidate_offsets:
        return []

    curr_azimuth = _resolve_azimuth(curr_x, curr_y, fallback_yaw)
    best_perm = _find_shortest_open_tour(candidate_offsets, curr_azimuth)

    ranked: list[RankedViewpoint] = []
    ref_azimuth = curr_azimuth
    accumulated_cost = 0.0

    for next_offset in best_perm:
        vp = _build_viewpoint(
            target_offset=next_offset,
            ref_azimuth=ref_azimuth,
            standoff_radius=standoff_radius,
            board_frame=board_frame,
            cum_cost=accumulated_cost,
        )
        ranked.append(vp)
        ref_azimuth = next_offset
        accumulated_cost = vp.cumulative_cost

    return ranked


__all__ = [
    "RankedViewpoint",
    "rank_by_azimuth",
]
