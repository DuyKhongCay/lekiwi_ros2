# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""
Analytical Geometric Kinematics Engine for LeKiwi SO-101 5-DOF Arm.

Solves closed-form IK for top-down chess pick-and-place:
1. arm_shoulder_pan (Joint 1, Yaw around Z)
2. arm_shoulder_lift (Joint 2, Pitch)
3. arm_elbow_flex (Joint 3, Pitch)
4. arm_wrist_flex (Joint 4, Pitch)
5. arm_wrist_roll (Joint 5, Roll along gripper axis)

Enforces strict Elbow-Up configuration (Elbow-Down is physically outside joint limits).
Provides Base Standoff Pose calculation for mobile base docking around the chessboard.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

# Physical constants extracted from duykhongcay_lekiwi.urdf
BASE_OFFSET_X = 0.0461807
BASE_OFFSET_Y = 1.27994e-05
BASE_OFFSET_Z = 0.1696

# Link lengths
L1 = 0.115998  # Shoulder lift to elbow flex (m)
L2 = 0.135000  # Elbow flex to wrist flex (m)
L3 = 0.105368  # Wrist flex to gripperframe (m)

# Joint limits from duykhongcay_lekiwi.urdf (rad)
JOINT_LIMITS: Dict[str, Tuple[float, float]] = {
    "arm_shoulder_pan": (-1.74919, 1.74147),
    "arm_shoulder_lift": (-1.74533, 1.74533),
    "arm_elbow_flex": (-1.65765, 1.57121),
    "arm_wrist_flex": (-1.56161, 1.57999),
    "arm_wrist_roll": (-2.64577, 2.76475),
}

DEFAULT_PITCH_ANGLE = -1.57079632679  # -90 degrees (pointing straight down)
DEFAULT_CLEARANCE_PADDING = 0.03  # 3 cm safety buffer from board edge


def forward_kinematics_2d(
    q1: float, q2: float, q3: float, q4: float
) -> Tuple[float, float, float, float]:
    """
    Compute forward kinematics for joints 1-4.
    Returns (x, y, z, pitch_angle) in base_footprint frame.
    """
    # Planar reach in arm plane
    # Shoulder angle theta2, elbow angle theta3, wrist angle theta4
    a1 = q2
    a2 = q2 + q3
    a3 = q2 + q3 + q4

    r_arm = L1 * math.cos(a1) + L2 * math.cos(a2) + L3 * math.cos(a3)
    z_arm = L1 * math.sin(a1) + L2 * math.sin(a2) + L3 * math.sin(a3)

    # 3D projection using pan angle q1
    x = BASE_OFFSET_X + r_arm * math.cos(q1)
    y = BASE_OFFSET_Y + r_arm * math.sin(q1)
    z = BASE_OFFSET_Z + z_arm
    pitch = a3

    return x, y, z, pitch


def check_joint_limits(joints: Dict[str, float]) -> Tuple[bool, Optional[str]]:
    """Verify all joint angles are within physical URDF limits."""
    for name, angle in joints.items():
        if name in JOINT_LIMITS:
            lower, upper = JOINT_LIMITS[name]
            if angle < lower or angle > upper:
                return (
                    False,
                    f"Joint {name} value {angle:.4f} rad exceeds limits [{lower:.4f}, {upper:.4f}]",
                )
    return True, None


def solve_analytical_ik(
    target_x: float,
    target_y: float,
    target_z: float,
    required_pitch: float = DEFAULT_PITCH_ANGLE,
    target_roll: float = 0.0,
) -> Tuple[bool, Optional[Dict[str, float]], float, str]:
    """
    Solve closed-form analytical IK for 5-DOF LeKiwi arm.
    Strictly selects Elbow-Up configuration (Elbow-Down is outside joint limits).

    Parameters:
        target_x, target_y, target_z: Target coordinates in base_footprint frame (m).
        required_pitch: Desired approach angle in vertical plane (rad).
        target_roll: Desired wrist roll angle (rad).

    Returns:
        (is_reachable, joint_dict, distance_to_target, message)
    """
    # Relative target vector from shoulder pan base
    dx = target_x - BASE_OFFSET_X
    dy = target_y - BASE_OFFSET_Y
    dz = target_z - BASE_OFFSET_Z

    distance_to_target = math.hypot(dx, dy)

    # 1. Joint 1: arm_shoulder_pan (Yaw)
    q1 = math.atan2(dy, dx)
    pan_lower, pan_upper = JOINT_LIMITS["arm_shoulder_pan"]
    if q1 < pan_lower or q1 > pan_upper:
        return (
            False,
            None,
            distance_to_target,
            f"Target yaw {q1:.3f} rad exceeds shoulder_pan limits",
        )

    # Planar radial distance from shoulder axis
    r = math.hypot(dx, dy)

    # 2. Subtract end-effector link L3 along required pitch angle to find wrist center
    r_w = r - L3 * math.cos(required_pitch)
    z_w = dz - L3 * math.sin(required_pitch)

    dist_sq = r_w**2 + z_w**2
    if dist_sq < 1e-8:
        return (
            False,
            None,
            distance_to_target,
            "Wrist center too close to shoulder singularity",
        )

    # 3. Law of Cosines for 2-link planar arm (L1, L2)
    cos_q3 = (dist_sq - L1**2 - L2**2) / (2.0 * L1 * L2)

    if abs(cos_q3) > 1.0:
        return (
            False,
            None,
            distance_to_target,
            f"Target geometrically unreachable (cos_q3 = {cos_q3:.3f})",
        )

    # Elbow-Up configuration: q3 <= 0 (Elbow-Down has q3 > 0, which violates elbow_flex limits)
    q3 = -math.acos(cos_q3)

    # 4. Joint 2: arm_shoulder_lift
    phi = math.atan2(z_w, r_w)
    psi = math.atan2(L2 * math.sin(q3), L1 + L2 * math.cos(q3))
    q2 = phi - psi

    # 5. Joint 4: arm_wrist_flex
    q4 = required_pitch - (q2 + q3)

    # 6. Joint 5: arm_wrist_roll
    q5 = target_roll

    solution = {
        "arm_shoulder_pan": q1,
        "arm_shoulder_lift": q2,
        "arm_elbow_flex": q3,
        "arm_wrist_flex": q4,
        "arm_wrist_roll": q5,
    }

    # 7. Check joint limits
    is_within_limits, limit_err = check_joint_limits(solution)
    if not is_within_limits:
        return (
            False,
            None,
            distance_to_target,
            f"Kinematic solution violates limits: {limit_err}",
        )

    return True, solution, distance_to_target, "REACHABLE (Elbow-Up)"


def compute_standoff_pose(
    target_x: float,
    target_y: float,
    board_w: float,
    board_h: float,
    robot_radius: float,
    d_clearance: float = DEFAULT_CLEARANCE_PADDING,
) -> Tuple[float, float, float, str]:
    """
    Compute optimal mobile base parking pose in chessboard_frame (ORIGIN AT CENTER).
    Selects the closest board perimeter edge (South, North, West, East) to target.

    In centered chessboard_frame:
        x in [-board_w/2, +board_w/2]
        y in [-board_h/2, +board_h/2]

    Returns:
        (x_standoff, y_standoff, theta_standoff, chosen_edge)
        where theta_standoff is the Yaw heading of the robot base in chessboard_frame.
    """
    half_w = board_w / 2.0
    half_h = board_h / 2.0
    d_margin_x = half_w + robot_radius + d_clearance
    d_margin_y = half_h + robot_radius + d_clearance

    # Distance to each edge from (target_x, target_y)
    d_south = target_y - (-half_h)  # distance to Y = -half_h
    d_north = half_h - target_y  # distance to Y = +half_h
    d_west = target_x - (-half_w)  # distance to X = -half_w
    d_east = half_w - target_x  # distance to X = +half_w

    edges = {
        "SOUTH": d_south,
        "NORTH": d_north,
        "WEST": d_west,
        "EAST": d_east,
    }

    closest_edge = min(edges, key=edges.get)

    # Clamping range along the edge to keep base centered within board bounds
    max_span_x = max(0.0, half_w - robot_radius)
    max_span_y = max(0.0, half_h - robot_radius)

    if closest_edge == "SOUTH":
        x_standoff = max(-max_span_x, min(target_x, max_span_x))
        y_standoff = -d_margin_y
        theta_standoff = math.pi / 2.0  # +90 deg (facing +Y towards center)

    elif closest_edge == "NORTH":
        x_standoff = max(-max_span_x, min(target_x, max_span_x))
        y_standoff = d_margin_y
        theta_standoff = -math.pi / 2.0  # -90 deg (facing -Y towards center)

    elif closest_edge == "WEST":
        x_standoff = -d_margin_x
        y_standoff = max(-max_span_y, min(target_y, max_span_y))
        theta_standoff = 0.0  # 0 deg (facing +X towards center)

    else:  # EAST
        x_standoff = d_margin_x
        y_standoff = max(-max_span_y, min(target_y, max_span_y))
        theta_standoff = math.pi  # 180 deg (facing -X towards center)

    return x_standoff, y_standoff, theta_standoff, closest_edge


def is_single_base_geometrically_possible(
    p1_x: float,
    p1_y: float,
    p2_x: float,
    p2_y: float,
    max_reach: float = 0.22,  # Effective top-down horizontal reach (~0.20-0.22m)
    max_pan_angle: float = 1.74,  # ~100 deg
) -> bool:
    """
    Tier 1 Early-Exit Geometry Pruning:
    Check if distance between pick and place allows both to be reached from a single shoulder point.
    """
    dist = math.hypot(p1_x - p2_x, p1_y - p2_y)
    # Theoretical maximum chord length within arm reach sector
    max_chord = 2.0 * max_reach * math.sin(min(max_pan_angle, math.pi / 2.0))
    return dist <= max_chord


def transform_point_to_base_frame(
    px: float, py: float, pz: float, base_x: float, base_y: float, base_theta: float
) -> Tuple[float, float, float]:
    """Transform point (px, py, pz) from chessboard_frame to base_footprint frame."""
    dx = px - base_x
    dy = py - base_y
    c = math.cos(base_theta)
    s = math.sin(base_theta)
    bx = c * dx + s * dy
    by = -s * dx + c * dy
    bz = pz
    return bx, by, bz


def find_common_standoff_pose(
    pick_x: float,
    pick_y: float,
    pick_z: float,
    place_x: float,
    place_y: float,
    place_z: float,
    board_w: float,
    board_h: float,
    robot_radius: float,
    d_clearance: float = DEFAULT_CLEARANCE_PADDING,
    required_pitch: float = DEFAULT_PITCH_ANGLE,
    max_samples: int = 25,
) -> Optional[Tuple[float, float, float, Dict[str, float], Dict[str, float], str]]:
    """
    Tier 2 Fast 1D Perimeter Sampling (≤ 15ms):
    Find a single standoff base pose (x, y, theta) where BOTH pick and place are reachable.

    Returns:
        (base_x, base_y, base_theta, pick_ik, place_ik, edge) or None if no common pose found.
    """
    # 1. Start from candidate edges near mid-point
    mid_x = (pick_x + place_x) / 2.0
    mid_y = (pick_y + place_y) / 2.0

    # Test closest standoff pose to midpoint
    base_x, base_y, base_theta, edge = compute_standoff_pose(
        mid_x, mid_y, board_w, board_h, robot_radius, d_clearance
    )

    half_w = board_w / 2.0
    half_h = board_h / 2.0
    d_margin_x = half_w + robot_radius + d_clearance
    d_margin_y = half_h + robot_radius + d_clearance

    # Determine 1D sampling direction along the chosen edge
    is_horizontal = edge in ("SOUTH", "NORTH")
    span = board_w if is_horizontal else board_h
    center_val = mid_x if is_horizontal else mid_y

    # Generate samples around midpoint
    sample_offsets = [0.0]
    step = 0.025  # 2.5 cm steps
    for i in range(1, max_samples // 2 + 1):
        sample_offsets.append(i * step)
        sample_offsets.append(-i * step)

    for offset in sample_offsets:
        cur_pos = center_val + offset
        if is_horizontal:
            cur_clamped = max(-half_w, min(cur_pos, half_w))
            cand_x = cur_clamped
            cand_y = -d_margin_y if edge == "SOUTH" else d_margin_y
            cand_th = math.pi / 2.0 if edge == "SOUTH" else -math.pi / 2.0
        else:
            cur_clamped = max(-half_h, min(cur_pos, half_h))
            cand_x = -d_margin_x if edge == "WEST" else d_margin_x
            cand_y = cur_clamped
            cand_th = 0.0 if edge == "WEST" else math.pi

        # Transform both points to this candidate base frame
        b_pick_x, b_pick_y, b_pick_z = transform_point_to_base_frame(
            pick_x, pick_y, pick_z, cand_x, cand_y, cand_th
        )
        b_place_x, b_place_y, b_place_z = transform_point_to_base_frame(
            place_x, place_y, place_z, cand_x, cand_y, cand_th
        )

        # Check IK for pick
        ok_pick, sol_pick, _, _ = solve_analytical_ik(
            b_pick_x, b_pick_y, b_pick_z, required_pitch=required_pitch
        )
        if not ok_pick:
            continue

        # Check IK for place
        ok_place, sol_place, _, _ = solve_analytical_ik(
            b_place_x, b_place_y, b_place_z, required_pitch=required_pitch
        )
        if ok_place:
            return cand_x, cand_y, cand_th, sol_pick, sol_place, edge

    return None
