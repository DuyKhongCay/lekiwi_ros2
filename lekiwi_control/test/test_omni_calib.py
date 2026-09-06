# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

import math
from lekiwi_control.omni_base_calibrator import (
    compute_robot_radius_calib,
    compute_square_umbmark,
    compute_wheel_radius_calib,
)


# Verifies wheel radius scaling under nominal and scaled travel scenarios.
def test_compute_wheel_radius_calib():
    curr_r = 0.065
    # Exact match scenario
    assert math.isclose(compute_wheel_radius_calib(curr_r, 1.0, 1.0), 0.065)
    # Undershoot (actual distance longer than odom reported)
    assert math.isclose(compute_wheel_radius_calib(curr_r, 1.0, 1.1), 0.065 * 1.1)
    # Zero target distance edge case handling
    assert math.isclose(compute_wheel_radius_calib(curr_r, 0.0, 1.0), 0.065)


# Verifies robot radius scaling under varying angular displacement conditions.
def test_compute_robot_radius_calib():
    curr_R = 0.1268
    target_rot = 10.0 * math.pi
    actual_rot = 10.2 * math.pi
    expected = curr_R * (10.2 / 10.0)
    assert math.isclose(
        compute_robot_radius_calib(curr_R, target_rot, actual_rot), expected
    )
    # Zero angle edge case
    assert math.isclose(compute_robot_radius_calib(curr_R, 0.0, actual_rot), curr_R)


# Verifies UMBmark systematic error metrics calculation for bidirectional square test runs.
def test_compute_square_umbmark():
    ccw_res = (0.02, 0.03, 0.01)
    cw_res = (-0.01, 0.04, -0.01)
    nom_edge = 1.0

    stats = compute_square_umbmark(ccw_res, cw_res, nom_edge)
    assert "alpha_rad" in stats
    assert "beta_rad" in stats
    assert math.isclose(stats["ccw_err_dist"], math.hypot(0.02, 0.03))
    assert math.isclose(stats["cw_err_dist"], math.hypot(-0.01, 0.04))
