# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

import math
import numpy as np
import pytest

from lekiwi_control.kinematics_engine import (
    BASE_OFFSET_X,
    BASE_OFFSET_Y,
    BASE_OFFSET_Z,
    DEFAULT_PITCH_ANGLE,
    JOINT_LIMITS,
    L1,
    L2,
    L3,
    check_joint_limits,
    compute_standoff_pose,
    forward_kinematics_2d,
    solve_analytical_ik,
)

# ================= Pure Kinematics & Analytical IK Tests =================


def test_analytical_ik_sweet_spot():
    """Verify IK reaches a typical sweet spot chess pick position."""
    # Target in front of robot in base_footprint frame
    target_x = 0.22
    target_y = 0.0
    target_z = 0.04
    pitch = DEFAULT_PITCH_ANGLE  # -90 deg

    reachable, solution, dist, msg = solve_analytical_ik(
        target_x, target_y, target_z, required_pitch=pitch
    )

    assert reachable, f"Failed sweet spot reach: {msg}"
    assert solution is not None
    assert "REACHABLE (Elbow-Up)" in msg

    # Elbow must strictly be Elbow-Up (negative angle for arm_elbow_flex)
    assert solution["arm_elbow_flex"] <= 0.0

    # Shoulder pan must be ~0 for target on centerline
    assert math.isclose(solution["arm_shoulder_pan"], 0.0, abs_tol=1e-3)

    # All joints must be within URDF limits
    is_valid, err = check_joint_limits(solution)
    assert is_valid, err


def test_round_trip_ik_fk():
    """Solve IK for multiple valid workspace points and check FK returns original point."""
    test_points = [
        (0.20, 0.02, 0.03),
        (0.24, -0.03, 0.05),
        (0.18, 0.05, 0.02),
        (0.22, 0.0, 0.08),
    ]

    for tx, ty, tz in test_points:
        reachable, sol, dist, msg = solve_analytical_ik(
            tx, ty, tz, required_pitch=DEFAULT_PITCH_ANGLE
        )
        if reachable and sol is not None:
            fk_x, fk_y, fk_z, fk_pitch = forward_kinematics_2d(
                sol["arm_shoulder_pan"],
                sol["arm_shoulder_lift"],
                sol["arm_elbow_flex"],
                sol["arm_wrist_flex"],
            )
            # Position must match within 1 mm
            assert math.isclose(fk_x, tx, abs_tol=1e-3)
            assert math.isclose(fk_y, ty, abs_tol=1e-3)
            assert math.isclose(fk_z, tz, abs_tol=1e-3)
            assert math.isclose(fk_pitch, DEFAULT_PITCH_ANGLE, abs_tol=1e-3)


def test_out_of_reach_detection():
    """Points too far, behind robot, or geometrically impossible must return unreachable."""
    # 1. Target way too far (0.80 m)
    reachable, sol, dist, msg = solve_analytical_ik(0.80, 0.0, 0.05)
    assert not reachable
    assert sol is None
    assert "geometrically unreachable" in msg

    # 2. Target behind robot (yaw exceeds shoulder_pan limit)
    reachable, sol, dist, msg = solve_analytical_ik(-0.30, 0.0, 0.05)
    assert not reachable
    assert sol is None
    assert "exceeds shoulder_pan limits" in msg

    # 3. Target way too close/below ground
    reachable, sol, dist, msg = solve_analytical_ik(0.046, 0.0, -0.20)
    assert not reachable


# ================= Base Standoff Pose Generator Tests =================


def test_standoff_pose_south_edge():
    """Piece near South edge (Rank 1/2) must pick SOUTH edge and face +Y."""
    board_w = 0.390
    board_h = 0.390
    robot_r = 0.1437
    clearance = 0.03
    d_margin = robot_r + clearance

    # Piece at e2 (x ~ 0.195, y ~ 0.05)
    x_st, y_st, th_st, edge = compute_standoff_pose(
        target_x=0.195,
        target_y=0.05,
        board_w=board_w,
        board_h=board_h,
        robot_radius=robot_r,
        d_clearance=clearance,
    )

    assert edge == "SOUTH"
    assert math.isclose(y_st, -d_margin, abs_tol=1e-4)
    assert math.isclose(x_st, 0.195, abs_tol=1e-4)
    assert math.isclose(th_st, math.pi / 2.0, abs_tol=1e-4)  # +90 deg


def test_standoff_pose_north_edge():
    """Piece near North edge (Rank 7/8) must pick NORTH edge and face -Y."""
    board_w = 0.390
    board_h = 0.390
    robot_r = 0.1437
    clearance = 0.03
    d_margin = robot_r + clearance

    # Piece at e7 (x ~ 0.195, y ~ 0.34)
    x_st, y_st, th_st, edge = compute_standoff_pose(
        target_x=0.195,
        target_y=0.34,
        board_w=board_w,
        board_h=board_h,
        robot_radius=robot_r,
        d_clearance=clearance,
    )

    assert edge == "NORTH"
    assert math.isclose(y_st, board_h + d_margin, abs_tol=1e-4)
    assert math.isclose(x_st, 0.195, abs_tol=1e-4)
    assert math.isclose(th_st, -math.pi / 2.0, abs_tol=1e-4)  # -90 deg


def test_standoff_pose_west_edge():
    """Piece near West edge (File A/B) must pick WEST edge and face +X."""
    board_w = 0.390
    board_h = 0.390
    robot_r = 0.1437
    clearance = 0.03
    d_margin = robot_r + clearance

    # Piece at a4 (x ~ 0.03, y ~ 0.195)
    x_st, y_st, th_st, edge = compute_standoff_pose(
        target_x=0.03,
        target_y=0.195,
        board_w=board_w,
        board_h=board_h,
        robot_radius=robot_r,
        d_clearance=clearance,
    )

    assert edge == "WEST"
    assert math.isclose(x_st, -d_margin, abs_tol=1e-4)
    assert math.isclose(y_st, 0.195, abs_tol=1e-4)
    assert math.isclose(th_st, 0.0, abs_tol=1e-4)


def test_standoff_pose_east_edge():
    """Piece near East edge (File G/H) must pick EAST edge and face -X."""
    board_w = 0.390
    board_h = 0.390
    robot_r = 0.1437
    clearance = 0.03
    d_margin = robot_r + clearance

    # Piece at h4 (x ~ 0.36, y ~ 0.195)
    x_st, y_st, th_st, edge = compute_standoff_pose(
        target_x=0.36,
        target_y=0.195,
        board_w=board_w,
        board_h=board_h,
        robot_radius=robot_r,
        d_clearance=clearance,
    )

    assert edge == "EAST"
    assert math.isclose(x_st, board_w + d_margin, abs_tol=1e-4)
    assert math.isclose(y_st, 0.195, abs_tol=1e-4)
    assert math.isclose(th_st, math.pi, abs_tol=1e-4)


# ================= Strict Failsafe Node Integration Tests =================


def test_failsafe_rejection_when_unconfigured():
    """Node must immediately reject service calls with failsafe message if not configured."""
    import rclpy
    from geometry_msgs.msg import Point
    from lekiwi_control.workspace_checker import WorkspaceCheckerNode
    from lekiwi_interfaces.srv import CheckReachability

    rclpy.init()
    try:
        node = WorkspaceCheckerNode()
        # Node starts unconfigured
        assert not node._is_configured

        req = CheckReachability.Request()
        req.target_point = Point(x=0.20, y=0.0, z=0.04)
        res = CheckReachability.Response()

        handled_res = node.handle_check_reachability(req, res)
        assert not handled_res.reachable
        assert "FAILSAFE" in handled_res.message
        assert "Service rejected for safety" in handled_res.message

        node.destroy_node()
    finally:
        rclpy.shutdown()
