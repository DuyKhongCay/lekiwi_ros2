# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for ChessTrajectoryGenerator and Quintic Spline Interpolation."""

import math
import pytest

from lekiwi_manipulation.trajectory_generator import (
    ARM_JOINTS_DEFAULT,
    ChessTrajectoryGenerator,
    DEFAULT_HOME_POSE,
    DEFAULT_STOW_POSE,
    LinearBlendStrategy,
    ManipulationPhase,
    QuinticSplineStrategy,
)


def test_quintic_spline_boundary_conditions():
    strategy = QuinticSplineStrategy()
    s0 = 0.5
    s1 = 2.0
    dur = 2.0

    # Start: tau = 0
    pos_0, vel_0, acc_0 = strategy.interpolate_scalar(s0, s1, 0.0, dur)
    assert math.isclose(pos_0, s0, abs_tol=1e-6)
    assert math.isclose(vel_0, 0.0, abs_tol=1e-6)
    assert math.isclose(acc_0, 0.0, abs_tol=1e-6)

    # End: tau = 1
    pos_1, vel_1, acc_1 = strategy.interpolate_scalar(s0, s1, 1.0, dur)
    assert math.isclose(pos_1, s1, abs_tol=1e-6)
    assert math.isclose(vel_1, 0.0, abs_tol=1e-6)
    assert math.isclose(acc_1, 0.0, abs_tol=1e-6)

    # Midpoint: tau = 0.5
    pos_mid, vel_mid, _ = strategy.interpolate_scalar(s0, s1, 0.5, dur)
    expected_mid = (s0 + s1) / 2.0
    assert math.isclose(pos_mid, expected_mid, abs_tol=1e-6)
    assert vel_mid > 0.0  # Positive forward velocity at midpoint


def test_linear_blend_fallback():
    strategy = LinearBlendStrategy()
    s0 = 1.0
    s1 = 3.0
    dur = 4.0

    pos_mid, vel_mid, acc_mid = strategy.interpolate_scalar(s0, s1, 0.5, dur)
    assert math.isclose(pos_mid, 2.0)
    assert math.isclose(vel_mid, 0.5)  # (3.0 - 1.0) / 4.0 = 0.5
    assert math.isclose(acc_mid, 0.0)


def test_single_phase_trajectory_generation():
    gen = ChessTrajectoryGenerator()
    q_start = dict(DEFAULT_STOW_POSE)
    q_target = dict(DEFAULT_HOME_POSE)

    pts = gen.generate_single_phase_trajectory(
        q_start=q_start,
        q_target=q_target,
        duration_sec=1.0,
        num_samples=5,
        start_time_offset=0.0,
    )

    assert len(pts) == 5
    # Check joint dimensions match 6 arm joints
    for p in pts:
        assert len(p.positions) == len(ARM_JOINTS_DEFAULT)
        assert len(p.velocities) == len(ARM_JOINTS_DEFAULT)
        assert len(p.accelerations) == len(ARM_JOINTS_DEFAULT)

    # Monotonic time check
    times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in pts]
    assert all(t1 < t2 for t1, t2 in zip(times, times[1:]))


def test_full_chess_move_trajectory_phases():
    gen = ChessTrajectoryGenerator()
    q_current = dict(DEFAULT_STOW_POSE)
    q_pick = dict(DEFAULT_HOME_POSE)
    q_place = dict(DEFAULT_HOME_POSE)
    q_place["arm_shoulder_pan"] = 0.5  # Moved sideways

    traj, timeline = gen.build_full_chess_move_trajectory(
        q_current=q_current,
        q_pick=q_pick,
        q_place=q_place,
        is_capture=False,
        phase_duration=0.5,
        samples_per_phase=4,
    )

    assert len(timeline) == 8  # 8 distinct phases
    expected_phases = [
        ManipulationPhase.APPROACH_PICK,
        ManipulationPhase.DESCEND_PICK,
        ManipulationPhase.GRASP,
        ManipulationPhase.LIFT,
        ManipulationPhase.TRANSIT_PLACE,
        ManipulationPhase.DESCEND_PLACE,
        ManipulationPhase.RELEASE,
        ManipulationPhase.RETRACT_STOW,
    ]
    actual_phases = [phase for phase, _ in timeline]
    assert actual_phases == expected_phases

    # Total points = 8 phases * 4 samples = 32 points
    assert len(traj.points) == 32
    assert traj.joint_names == list(ARM_JOINTS_DEFAULT)


def test_capture_trajectory_routes_to_chassis_bin():
    gen = ChessTrajectoryGenerator()
    q_current = dict(DEFAULT_STOW_POSE)
    q_pick = dict(DEFAULT_HOME_POSE)
    q_place = dict(DEFAULT_HOME_POSE)

    traj, timeline = gen.build_full_chess_move_trajectory(
        q_current=q_current,
        q_pick=q_pick,
        q_place=q_place,
        is_capture=True,
        phase_duration=0.5,
        samples_per_phase=4,
    )

    assert len(timeline) == 8
    # Last sample of TRANSIT_PLACE should position shoulder pan at ~1.57 rad (chassis bin)
    # Phase 5 is index 4 in timeline
    # Point index for end of phase 5 is 5 * 4 - 1 = 19
    transit_end_point = traj.points[19]
    pan_idx = traj.joint_names.index("arm_shoulder_pan")
    assert math.isclose(transit_end_point.positions[pan_idx], 1.57, abs_tol=0.05)
