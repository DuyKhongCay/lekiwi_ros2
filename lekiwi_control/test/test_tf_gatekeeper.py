# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

import math
import numpy as np
import pytest

from lekiwi_control.tf_gatekeeper import (
    DEFAULT_REQUIRED_ARM_JOINTS,
    check_robot_stationary,
    check_covariance_converged,
    check_arm_joints_complete,
    check_stamp_freshness,
    TfReadinessGatekeeper,
)

# ================= Pure Logic Unit Tests =================


def test_robot_stationary_detection():
    # Inside standstill limits (speed = hypot(0.01, 0.01) = 0.0141 <= 0.02, wz = 0.02 <= 0.05)
    assert check_robot_stationary(
        0.01, 0.01, 0.02, max_linear_vel=0.02, max_angular_vel=0.05
    )

    # Exactly zero
    assert check_robot_stationary(0.0, 0.0, 0.0)

    # Exceeding linear velocity limit (0.05 m/s > 0.02 m/s)
    assert not check_robot_stationary(
        0.05, 0.0, 0.0, max_linear_vel=0.02, max_angular_vel=0.05
    )

    # Exceeding angular velocity limit (0.1 rad/s > 0.05 rad/s)
    assert not check_robot_stationary(
        0.005, 0.005, 0.1, max_linear_vel=0.02, max_angular_vel=0.05
    )


def test_covariance_convergence():
    # Build 6x6 covariance matrix (flattened 36 elements)
    cov = [0.0] * 36
    cov[0] = 0.0001  # var(x)
    cov[7] = 0.0001  # var(y) -> sum = 0.0002 <= 0.0004
    cov[35] = 0.0010  # var(yaw) <= 0.0020

    converged, pos_var, yaw_var = check_covariance_converged(
        cov, max_pos_var=0.0004, max_yaw_var=0.0020
    )
    assert converged
    assert math.isclose(pos_var, 0.0002)
    assert math.isclose(yaw_var, 0.0010)

    # High position variance (0.0005 > 0.0004)
    cov_high_pos = list(cov)
    cov_high_pos[0] = 0.0004
    cov_high_pos[7] = 0.0002  # sum = 0.0006
    converged, _, _ = check_covariance_converged(
        cov_high_pos, max_pos_var=0.0004, max_yaw_var=0.0020
    )
    assert not converged

    # High yaw variance (0.0050 > 0.0020)
    cov_high_yaw = list(cov)
    cov_high_yaw[35] = 0.0050
    converged, _, _ = check_covariance_converged(
        cov_high_yaw, max_pos_var=0.0004, max_yaw_var=0.0020
    )
    assert not converged

    # Invalid length
    converged, _, _ = check_covariance_converged([0.0] * 10)
    assert not converged

    # NaN in covariance
    cov_nan = list(cov)
    cov_nan[0] = float("nan")
    converged, _, _ = check_covariance_converged(cov_nan)
    assert not converged


def test_arm_joints_completeness():
    required = set(DEFAULT_REQUIRED_ARM_JOINTS)
    received_all = required | {"base_left_wheel", "base_back_wheel", "base_right_wheel"}

    complete, missing = check_arm_joints_complete(received_all, required)
    assert complete
    assert len(missing) == 0

    # Missing arm_gripper
    received_partial = set(required)
    received_partial.remove("arm_gripper")
    complete, missing = check_arm_joints_complete(received_partial, required)
    assert not complete
    assert missing == {"arm_gripper"}


def test_stamp_freshness():
    # Fresh (age 0.05s <= 0.15s)
    is_fresh, age = check_stamp_freshness(
        current_time_sec=100.05, stamp_time_sec=100.0, max_age_sec=0.15
    )
    assert is_fresh
    assert math.isclose(age, 0.05)

    # Stale (age 0.25s > 0.15s)
    is_fresh, age = check_stamp_freshness(
        current_time_sec=100.25, stamp_time_sec=100.0, max_age_sec=0.15
    )
    assert not is_fresh
    assert math.isclose(age, 0.25)


# ================= ROS Node Interface Tests =================


def test_node_instantiation_without_ros_spin():
    """Verify node initializes parameters and publishes initial false ready state."""
    import rclpy

    if not rclpy.ok():
        rclpy.init()

    try:
        node = TfReadinessGatekeeper()
        assert not node.is_tf_ready
        assert node._max_age == 0.30
        assert node._max_pos_var == 0.0012
        assert node._max_yaw_var == 0.0030
        assert node._max_vel == 0.03
        assert node._max_ang_vel == 0.08
        assert node._map_frame == "map"
        assert node._base_frame == "base_footprint"
        assert node._ee_frame == "gripperframe"
        assert node._board_frame == "chessboard_frame"
        assert node._trigger_nav2 is True
        assert node._nav2_service_name == "/lifecycle_manager_navigation/manage_nodes"
        assert node._auto_pause_nav2 is False
        node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
