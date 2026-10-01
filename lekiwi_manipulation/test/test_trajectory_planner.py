"""Unit tests for QuinticTrajectoryPlanner and C2 continuity."""

import pytest
from lekiwi_manipulation.common.arm_constants import ARM_JOINTS, NAMED_POSES
from lekiwi_manipulation.kinematics.trajectory_planner import (
    QuinticTrajectoryPlanner,
    create_named_pose_trajectory,
)


def test_trajectory_planner_basic_generation():
    planner = QuinticTrajectoryPlanner(default_rate_hz=50.0)
    q_start = [0.0] * 6
    q_target = [0.2, -0.3, 0.4, -0.1, 0.5, 0.0]
    duration = 2.0

    traj = planner.plan_trajectory(q_start, q_target, duration_sec=duration)

    assert traj.joint_names == ARM_JOINTS
    assert len(traj.points) > 10

    # Start point check
    p0 = traj.points[0]
    for i in range(6):
        assert pytest.approx(p0.positions[i], abs=1e-5) == q_start[i]
        assert pytest.approx(p0.velocities[i], abs=1e-5) == 0.0
        assert pytest.approx(p0.accelerations[i], abs=1e-5) == 0.0

    # End point check (C2 continuity: v=0, a=0 at end)
    pf = traj.points[-1]
    for i in range(6):
        assert pytest.approx(pf.positions[i], abs=1e-5) == q_target[i]
        assert pytest.approx(pf.velocities[i], abs=1e-5) == 0.0
        assert pytest.approx(pf.accelerations[i], abs=1e-5) == 0.0
    assert pf.time_from_start.sec == int(duration)


def test_trajectory_planner_rejects_out_of_bounds():
    planner = QuinticTrajectoryPlanner()
    q_start = [0.0] * 6
    q_invalid_target = [0.0, 3.5, 0.0, 0.0, 0.0, 0.0]  # Exceeds max 1.80

    with pytest.raises(ValueError, match="Trajectory planning rejected: Joint 'arm_shoulder_lift'"):
        planner.plan_trajectory(q_start, q_invalid_target, duration_sec=2.0)


def test_trajectory_planner_invalid_duration():
    planner = QuinticTrajectoryPlanner()
    with pytest.raises(ValueError, match="duration must be positive"):
        planner.plan_trajectory([0.0] * 6, [0.1] * 6, duration_sec=0.0)


def test_create_named_pose_trajectory():
    traj = create_named_pose_trajectory([0.0] * 6, "stow", duration_sec=1.5)
    assert len(traj.points) > 0
    assert pytest.approx(traj.points[-1].positions, abs=1e-4) == NAMED_POSES["stow"]

    with pytest.raises(KeyError, match="Unknown named pose"):
        create_named_pose_trajectory([0.0] * 6, "non_existent_pose")
