# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for ViewpointCostScheduler, Radial Standoff Guard, and Azimuth Metric."""

from __future__ import annotations

import math
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import PoseStamped
from lekiwi_orchestrator.board_geometry import (
    RankedViewpoint,
    compute_radial_entry_pose,
    is_on_standoff_circle,
    normalize_angle,
    rank_by_azimuth,
    yaw_to_quaternion,
)


def test_normalize_angle():
    assert math.isclose(normalize_angle(0.0), 0.0)
    assert math.isclose(normalize_angle(math.pi), math.pi, abs_tol=1e-6)
    assert math.isclose(normalize_angle(-math.pi), -math.pi, abs_tol=1e-6)
    assert math.isclose(normalize_angle(3 * math.pi), math.pi, abs_tol=1e-6)
    assert math.isclose(normalize_angle(-3 * math.pi), -math.pi, abs_tol=1e-6)
    assert math.isclose(normalize_angle(math.pi / 2), math.pi / 2)


def test_is_on_standoff_circle():
    R = 0.56
    tol = 0.04

    # Exactly on circle
    assert is_on_standoff_circle(0.56, 0.0, R, tol)
    assert is_on_standoff_circle(0.0, -0.56, R, tol)
    assert is_on_standoff_circle(-0.56, 0.0, R, tol)

    # Within tolerance band [0.52, 0.60]
    assert is_on_standoff_circle(0.53, 0.0, R, tol)
    assert is_on_standoff_circle(0.59, 0.0, R, tol)

    # Inside circle (grasping position, e.g. 0.35m) -> False
    assert not is_on_standoff_circle(0.35, 0.0, R, tol)
    assert not is_on_standoff_circle(0.0, 0.0, R, tol)

    # Far outside circle (e.g. 0.70m) -> False
    assert not is_on_standoff_circle(0.70, 0.0, R, tol)


def test_compute_radial_entry_pose_from_inside():
    # Robot is at (0.3, 0.0) - inside circle of R=0.56 along +X
    pose = compute_radial_entry_pose(0.3, 0.0, standoff_radius=0.56)
    assert pose.header.frame_id == "chessboard_frame"
    assert math.isclose(pose.pose.position.x, 0.56, abs_tol=1e-3)
    assert math.isclose(pose.pose.position.y, 0.0, abs_tol=1e-3)


def test_compute_radial_entry_pose_diagonal():
    # Robot is at (0.2, 0.2), angle = pi/4
    pose = compute_radial_entry_pose(0.2, 0.2, standoff_radius=0.56)
    expected_x = 0.56 * math.cos(math.pi / 4)
    expected_y = 0.56 * math.sin(math.pi / 4)
    assert math.isclose(pose.pose.position.x, expected_x, abs_tol=1e-3)
    assert math.isclose(pose.pose.position.y, expected_y, abs_tol=1e-3)


def test_compute_radial_entry_pose_at_origin():
    # Robot is at (0.0, 0.0), default fallback_yaw=0.0 rad along +X axis of chessboard_frame
    pose_default = compute_radial_entry_pose(0.0, 0.0, standoff_radius=0.56)
    assert math.isclose(pose_default.pose.position.x, 0.56, abs_tol=1e-3)
    assert math.isclose(pose_default.pose.position.y, 0.0, abs_tol=1e-3)

    # With explicit fallback_yaw
    pose = compute_radial_entry_pose(
        0.0, 0.0, standoff_radius=0.56, fallback_yaw=math.pi / 2
    )
    assert math.isclose(pose.pose.position.x, 0.0, abs_tol=1e-3)
    assert math.isclose(pose.pose.position.y, 0.56, abs_tol=1e-3)


def test_rank_by_azimuth_sorting():
    # Current robot is on circle at angle 0.0: (0.56, 0.0)
    candidates = [math.pi / 2, 0.1, -0.3, math.pi]
    ranked = rank_by_azimuth(
        0.56, 0.0, candidate_offsets=candidates, standoff_radius=0.56
    )

    assert len(ranked) == 4
    # Closest to 0.0 is 0.1, then -0.3, then pi/2, then pi
    assert math.isclose(ranked[0].angle_offset, 0.1)
    assert math.isclose(ranked[1].angle_offset, -0.3)
    assert math.isclose(ranked[2].angle_offset, math.pi / 2)
    assert math.isclose(ranked[3].angle_offset, math.pi)

    # Cost is monotonic ascending
    for i in range(len(ranked) - 1):
        assert ranked[i].arc_cost <= ranked[i + 1].arc_cost


def test_rank_by_azimuth_wraparound():
    # Current robot is at +3.1 rad (near +pi)
    # Candidate A is -3.1 rad (near -pi): diff across boundary is 0.083 rad
    # Candidate B is 0.0 rad: diff is 3.1 rad
    candidates = [0.0, -3.1]
    curr_x = 0.56 * math.cos(3.1)
    curr_y = 0.56 * math.sin(3.1)

    ranked = rank_by_azimuth(
        curr_x, curr_y, candidate_offsets=candidates, standoff_radius=0.56
    )

    assert len(ranked) == 2
    assert math.isclose(ranked[0].angle_offset, -3.1)
    assert math.isclose(ranked[1].angle_offset, 0.0)


def test_rank_by_azimuth_empty_and_origin():
    # Empty candidates
    ranked = rank_by_azimuth(0.56, 0.0, candidate_offsets=[], standoff_radius=0.56)
    assert ranked == []

    # At origin with fallback yaw = pi
    candidates = [0.0, math.pi - 0.1]
    ranked_orig = rank_by_azimuth(
        0.0,
        0.0,
        candidate_offsets=candidates,
        standoff_radius=0.56,
        fallback_yaw=math.pi,
    )
    assert math.isclose(ranked_orig[0].angle_offset, math.pi - 0.1)


def test_yaw_to_quaternion():
    q0 = yaw_to_quaternion(0.0)
    assert q0.x == 0.0
    assert q0.y == 0.0
    assert abs(q0.z) < 1e-6
    assert abs(q0.w - 1.0) < 1e-6

    q_pi = yaw_to_quaternion(math.pi)
    assert abs(q_pi.z - 1.0) < 1e-6
    assert abs(q_pi.w) < 1e-6

    q_half_pi = yaw_to_quaternion(math.pi / 2.0)
    assert math.isclose(q_half_pi.z, math.sin(math.pi / 4.0), abs_tol=1e-5)
    assert math.isclose(q_half_pi.w, math.cos(math.pi / 4.0), abs_tol=1e-5)




