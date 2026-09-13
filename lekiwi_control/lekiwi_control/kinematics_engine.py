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
    Compute optimal mobile base parking pose in chessboard_frame.
    Selects the closest board perimeter edge (South, North, West, East) to target.

    Parameters:
        target_x, target_y: Target chess piece location in chessboard_frame (m).
        board_w: Board width along X (A1 -> H1) in m.
        board_h: Board height along Y (A1 -> A8) in m.
        robot_radius: Calibrated robot chassis radius in m.
        d_clearance: Safety padding beyond robot radius in m.

    Returns:
        (x_standoff, y_standoff, theta_standoff, chosen_edge)
        where theta_standoff is the Yaw heading of the robot base in chessboard_frame.
    """
    d_margin = robot_radius + d_clearance

    # Distance to each edge from (target_x, target_y)
    d_south = target_y  # Y = 0 (Rank 1)
    d_north = board_h - target_y  # Y = board_h (Rank 8)
    d_west = target_x  # X = 0 (File A)
    d_east = board_w - target_x  # X = board_w (File H)

    edges = {
        "SOUTH": d_south,
        "NORTH": d_north,
        "WEST": d_west,
        "EAST": d_east,
    }

    closest_edge = min(edges, key=edges.get)

    if closest_edge == "SOUTH":
        x_standoff = max(robot_radius, min(target_x, board_w - robot_radius))
        y_standoff = -d_margin
        theta_standoff = math.pi / 2.0  # +90 deg (facing +Y towards board)

    elif closest_edge == "NORTH":
        x_standoff = max(robot_radius, min(target_x, board_w - robot_radius))
        y_standoff = board_h + d_margin
        theta_standoff = -math.pi / 2.0  # -90 deg (facing -Y towards board)

    elif closest_edge == "WEST":
        x_standoff = -d_margin
        y_standoff = max(robot_radius, min(target_y, board_h - robot_radius))
        theta_standoff = 0.0  # 0 deg (facing +X towards board)

    else:  # EAST
        x_standoff = board_w + d_margin
        y_standoff = max(robot_radius, min(target_y, board_h - robot_radius))
        theta_standoff = math.pi  # 180 deg (facing -X towards board)

    return x_standoff, y_standoff, theta_standoff, closest_edge
