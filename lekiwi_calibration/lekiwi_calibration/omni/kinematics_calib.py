"""Pure mathematical algorithms for LeKiwi omni base kinematic calibration."""

import math
from typing import Dict, Tuple


def normalize_angle(angle: float) -> float:
    """Normalizes angle to the range [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def compute_wheel_radius_calib(
    curr_wheel_radius: float, target_dist: float, measured_dist: float
) -> float:
    """Computes updated wheel radius based on target distance versus measured travel.

    Scales wheel radius linearly with actual ground truth translation distance.
    """
    if target_dist <= 0.0:
        return curr_wheel_radius
    return curr_wheel_radius * (measured_dist / target_dist)


def compute_robot_radius_calib(
    curr_robot_radius: float, target_rot_rad: float, measured_rot_rad: float
) -> float:
    """Computes updated robot radius based on target vs measured angular rotation.

    Scales robot radius inversely with ground truth angular rotation:
    When robot turns physically less than target (measured < target), effective radius is larger.
    """
    if abs(measured_rot_rad) <= 1e-6 or abs(target_rot_rad) <= 1e-6:
        return curr_robot_radius
    return curr_robot_radius * (target_rot_rad / measured_rot_rad)


def compute_square_umbmark(
    ccw_res: Tuple[float, float, float],
    cw_res: Tuple[float, float, float],
    nom_edge: float,
) -> Dict[str, float]:
    """Evaluates UMBmark benchmark metrics for bidirectional square test runs.

    Calculates systematic odometry error factors according to the UMBmark procedure.
    """
    x_ccw, y_ccw, _ = ccw_res
    x_cw, y_cw, _ = cw_res

    alpha = (x_cw + x_ccw) / (-4.0 * nom_edge)
    beta = (x_cw - x_ccw) / (-4.0 * nom_edge)

    return {
        "alpha_rad": alpha,
        "beta_rad": beta,
        "ccw_err_dist": math.hypot(x_ccw, y_ccw),
        "cw_err_dist": math.hypot(x_cw, y_cw),
    }
