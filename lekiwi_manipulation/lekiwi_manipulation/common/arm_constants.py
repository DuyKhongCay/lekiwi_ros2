"""Single Source of Truth (SSOT) for LeKiwi Arm Joint Definitions, Limits, and Named Poses.

Calibration bounds derived from:
`lekiwi_description/config/calibration/sts3215_servos_calib.yaml`
and `lekiwi_description/urdf/duykhongcay_lekiwi.urdf`.
"""

import math
from typing import Dict, List, Tuple

# Standard joint ordering used by arm_trajectory_controller and arm_forward_controller
ARM_JOINTS: List[str] = [
    "arm_shoulder_pan",
    "arm_shoulder_lift",
    "arm_elbow_flex",
    "arm_wrist_flex",
    "arm_wrist_roll",
    "arm_gripper",
]

# Resolution of Feetech STS3215 (12-bit, 4096 ticks / 360 deg)
ENCODER_RESOLUTION: int = 4096
ENCODER_CENTER_TICKS: int = 2048
RADIANS_PER_TICK: float = (2.0 * math.pi) / ENCODER_RESOLUTION
TICKS_PER_RADIAN: float = ENCODER_RESOLUTION / (2.0 * math.pi)


def ticks_to_radians(ticks: int, center_ticks: int = ENCODER_CENTER_TICKS) -> float:
    """Convert raw servo encoder ticks to radians around center."""
    return (float(ticks) - float(center_ticks)) * RADIANS_PER_TICK


def radians_to_ticks(rad: float, center_ticks: int = ENCODER_CENTER_TICKS) -> int:
    """Convert radians to raw servo encoder ticks."""
    return int(round(rad * TICKS_PER_RADIAN)) + center_ticks


# Safe joint limits in radians [min_rad, max_rad]
# Intersection of URDF limits and STS3215 calibration range_min / range_max:
# - arm_shoulder_pan: calib [844, 3345] -> [-1.846, 1.989] rad; URDF [-1.85, 1.85] -> [-1.84, 1.84]
# - arm_shoulder_lift: calib [888, 3226] -> [-1.779, 1.806] rad; URDF [-1.85, 1.85] -> [-1.77, 1.80]
# - arm_elbow_flex: calib [948, 3075] -> [-1.687, 1.575] rad; URDF [-1.75, 1.68] -> [-1.68, 1.57]
# - arm_wrist_flex: calib [909, 3095] -> [-1.747, 1.605] rad; URDF [-1.68, 1.68] -> [-1.68, 1.60]
# - arm_wrist_roll: calib [0, 4095]; URDF [-2.80, 2.85] -> [-2.80, 2.85]
# - arm_gripper: calib [2032, 3474] -> [-0.024, 2.186] rad; URDF [-0.15, 1.75] -> [-0.02, 1.75]
JOINT_LIMITS: Dict[str, Tuple[float, float]] = {
    "arm_shoulder_pan": (-1.84, 1.84),
    "arm_shoulder_lift": (-1.77, 1.80),
    "arm_elbow_flex": (-1.68, 1.57),
    "arm_wrist_flex": (-1.68, 1.60),
    "arm_wrist_roll": (-2.80, 2.85),
    "arm_gripper": (-0.02, 1.75),
}

# Gripper operational constants
GRIPPER_OPEN: float = 1.50   # Fully open jaw
GRIPPER_CLOSED: float = 0.00 # Closed / neutral contact

# Standard named poses (rad) ordered by ARM_JOINTS
NAMED_POSES: Dict[str, List[float]] = {
    "zero": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "home": [0.0, 0.0, 0.0, 0.0, 0.0, GRIPPER_CLOSED],
    "ready": [0.0, -0.50, 0.50, 0.0, 0.0, 0.50],
    "stow": [0.0, -1.57, 1.57, 0.75, 0.0, GRIPPER_CLOSED],
    "observe": [0.0, -0.80, 1.20, -0.40, 0.0, GRIPPER_CLOSED],
}


def validate_joint_limits(
    joint_targets: Dict[str, float]
) -> Tuple[bool, str]:
    """Validate target joint positions against hardware & calibration limits.

    Args:
        joint_targets: Dictionary mapping joint name to target angle in radians.

    Returns:
        Tuple of (is_valid: bool, error_message: str).
    """
    for joint_name, target_pos in joint_targets.items():
        if joint_name not in JOINT_LIMITS:
            continue
        min_limit, max_limit = JOINT_LIMITS[joint_name]
        # Include small numerical epsilon (1e-4) to avoid floating-point boundary issues
        if target_pos < (min_limit - 1e-4) or target_pos > (max_limit + 1e-4):
            return (
                False,
                f"Joint '{joint_name}' target {target_pos:.4f} rad exceeds valid range "
                f"[{min_limit:.4f}, {max_limit:.4f}] rad (from sts3215_calib).",
            )
    return True, ""
