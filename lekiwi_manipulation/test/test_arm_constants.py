"""Unit tests for LeKiwi arm constants and joint limit validation."""

import pytest
from lekiwi_manipulation.common.arm_constants import (
    ARM_JOINTS,
    GRIPPER_CLOSED,
    GRIPPER_OPEN,
    JOINT_LIMITS,
    NAMED_POSES,
    radians_to_ticks,
    ticks_to_radians,
    validate_joint_limits,
)


def test_arm_joints_count():
    assert len(ARM_JOINTS) == 6
    assert "arm_shoulder_pan" in ARM_JOINTS
    assert "arm_gripper" in ARM_JOINTS


def test_joint_limits_consistency():
    for joint in ARM_JOINTS:
        assert joint in JOINT_LIMITS, f"Missing limits for {joint}"
        min_lim, max_lim = JOINT_LIMITS[joint]
        assert min_lim < max_lim, f"Min limit must be less than max limit for {joint}"


def test_named_poses_validity():
    assert "home" in NAMED_POSES
    assert "stow" in NAMED_POSES
    assert "ready" in NAMED_POSES

    for pose_name, positions in NAMED_POSES.items():
        assert len(positions) == len(ARM_JOINTS), f"Pose {pose_name} length mismatch"
        pose_dict = dict(zip(ARM_JOINTS, positions))
        is_valid, msg = validate_joint_limits(pose_dict)
        assert is_valid, f"Pose '{pose_name}' violates joint limits: {msg}"


def test_validate_joint_limits_pass():
    valid_targets = {
        "arm_shoulder_pan": 0.0,
        "arm_shoulder_lift": 0.5,
        "arm_elbow_flex": -0.5,
        "arm_wrist_flex": 0.2,
        "arm_wrist_roll": 1.0,
        "arm_gripper": GRIPPER_OPEN,
    }
    is_valid, msg = validate_joint_limits(valid_targets)
    assert is_valid
    assert msg == ""


def test_validate_joint_limits_violation():
    invalid_targets = {
        "arm_shoulder_lift": 2.50,  # Max is 1.80
    }
    is_valid, msg = validate_joint_limits(invalid_targets)
    assert not is_valid
    assert "arm_shoulder_lift" in msg
    assert "exceeds valid range" in msg


def test_ticks_radians_conversion_roundtrip():
    center_ticks = 2048
    rad = ticks_to_radians(center_ticks)
    assert pytest.approx(rad, abs=1e-5) == 0.0

    ticks = radians_to_ticks(0.0)
    assert ticks == center_ticks
