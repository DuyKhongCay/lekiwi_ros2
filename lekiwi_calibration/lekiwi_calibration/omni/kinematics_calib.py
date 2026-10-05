"""Pure mathematical algorithms for LeKiwi omni base kinematic calibration."""

import math
from typing import Dict, Tuple


def normalize_angle(angle: float) -> float:
    """Normalize planar angle in radians to the principal interval [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def compute_wheel_radius_calib(
    curr_wheel_radius: float, target_dist: float, measured_dist: float
) -> float:
    """Compute updated wheel radius based on target distance versus measured travel.

    Scales nominal wheel radius linearly with actual ground truth translation distance:
    R_new = R_curr * (measured_dist / target_dist).

    Args:
        curr_wheel_radius: Current active wheel radius parameter (meters).
        target_dist: Commanded or nominal travel distance (meters).
        measured_dist: Ground truth translation distance from external measurement (meters).

    Returns:
        Updated calibrated wheel radius (meters).
    """
    if target_dist <= 0.0:
        return curr_wheel_radius
    return curr_wheel_radius * (measured_dist / target_dist)


def compute_robot_radius_calib(
    curr_robot_radius: float, target_rot_rad: float, measured_rot_rad: float
) -> float:
    """Compute updated robot base radius based on target versus measured angular rotation.

    Scales robot radius inversely with ground truth angular rotation:
    When the robot turns physically less than commanded, the effective base radius is larger.

    Args:
        curr_robot_radius: Current robot footprint radius from center to wheels (meters).
        target_rot_rad: Commanded total rotation in radians.
        measured_rot_rad: Ground truth rotation from external gyroscope or optical tracker (radians).

    Returns:
        Updated calibrated robot base radius (meters).
    """
    if abs(measured_rot_rad) <= 1e-6 or abs(target_rot_rad) <= 1e-6:
        return curr_robot_radius
    return curr_robot_radius * (target_rot_rad / measured_rot_rad)


def compute_square_umbmark(
    ccw_res: Tuple[float, float, float],
    cw_res: Tuple[float, float, float],
    nom_edge: float,
) -> Dict[str, float]:
    """Evaluate UMBmark benchmark metrics for bidirectional square path test runs.

    Calculates systematic odometry error factors (alpha and beta) according to UMBmark procedure.

    Args:
        ccw_res: Final (x, y, yaw) stopping error tuple after counter-clockwise run.
        cw_res: Final (x, y, yaw) stopping error tuple after clockwise run.
        nom_edge: Nominal edge length of the test square trajectory (meters).

    Returns:
        Dictionary containing calculated alpha_rad, beta_rad, ccw_err_dist, and cw_err_dist.
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
