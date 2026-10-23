# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for Arm Pose Calibrator node helpers and state transitions."""

import pytest
import yaml
from sensor_msgs.msg import JointState

from lekiwi_calibration.arm.arm_pose_calibrator_node import (
    DEFAULT_ARM_JOINTS,
    DEFAULT_TARGET_POSES,
    extract_joint_positions,
    format_arm_poses_yaml,
)


def test_extract_joint_positions_normal_order() -> None:
    """Verify correct joint position extraction when msg order matches target."""
    msg = JointState()
    msg.name = list(DEFAULT_ARM_JOINTS)
    msg.position = [0.1, -0.2, 0.3, -0.4, 0.5, 0.6]

    res = extract_joint_positions(msg, DEFAULT_ARM_JOINTS)
    assert res == [0.1, -0.2, 0.3, -0.4, 0.5, 0.6]


def test_extract_joint_positions_permuted_order() -> None:
    """Verify safe extraction when joint names are scrambled in publisher (Pitfall 17)."""
    msg = JointState()
    # Scramble joint order and add unrelated joints (e.g. base wheels)
    msg.name = [
        "base_left_wheel",
        "arm_gripper",
        "arm_elbow_flex",
        "arm_shoulder_pan",
        "arm_wrist_roll",
        "arm_shoulder_lift",
        "arm_wrist_flex",
    ]
    msg.position = [99.0, 0.6, 0.3, 0.1, 0.5, -0.2, -0.4]

    res = extract_joint_positions(msg, DEFAULT_ARM_JOINTS)
    # Expected order: pan (0.1), lift (-0.2), elbow (0.3), wrist_flex (-0.4), wrist_roll (0.5), gripper (0.6)
    assert pytest.approx(res, rel=1e-5) == [0.1, -0.2, 0.3, -0.4, 0.5, 0.6]


def test_extract_joint_positions_missing_joint() -> None:
    """Verify ValueError is raised when required arm joint is absent."""
    msg = JointState()
    msg.name = ["arm_shoulder_pan", "arm_shoulder_lift"]
    msg.position = [0.1, -0.2]

    with pytest.raises(ValueError, match="missing required joints"):
        extract_joint_positions(msg, DEFAULT_ARM_JOINTS)


def test_format_arm_poses_yaml_structure() -> None:
    """Verify output YAML structure conforms to manipulation_action_server schema."""
    calibrated_poses = {
        "approach": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "stow": [0.0, -1.57079, 1.57079, 0.75, 0.0, 0.0],
        "clear_left": [1.2, -0.6, 0.8, 0.0, 0.0, 0.0],
        "clear_right": [-1.2, -0.6, 0.8, 0.0, 0.0, 0.0],
    }

    yaml_str = format_arm_poses_yaml(
        existing_data={},
        calibrated_poses=calibrated_poses,
        arm_joints=DEFAULT_ARM_JOINTS,
        precision=4,
    )

    assert "manipulation_action_server:" in yaml_str
    assert "named_poses:" in yaml_str
    assert "approach: [0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000]" in yaml_str
    assert "stow: [0.0000, -1.5708, 1.5708, 0.7500, 0.0000, 0.0000]" in yaml_str
    assert "clear_left: [1.2000, -0.6000, 0.8000, 0.0000, 0.0000, 0.0000]" in yaml_str
    assert "clear_right: [-1.2000, -0.6000, 0.8000, 0.0000, 0.0000, 0.0000]" in yaml_str

    parsed = yaml.safe_load(yaml_str)
    assert "manipulation_action_server" in parsed
    poses = parsed["manipulation_action_server"]["ros__parameters"]["named_poses"]
    for p in DEFAULT_TARGET_POSES:
        assert p in poses
        assert len(poses[p]) == 6


def test_format_arm_poses_yaml_merge_existing() -> None:
    """Verify preserving unrelated configuration in existing YAML."""
    existing = {
        "other_subsystem": {"key": 123},
        "manipulation_action_server": {
            "ros__parameters": {
                "max_velocity": 1.5,
                "named_poses": {
                    "custom_pose": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
                },
            }
        },
    }

    calibrated_poses = {
        "approach": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    }

    yaml_str = format_arm_poses_yaml(
        existing_data=existing,
        calibrated_poses=calibrated_poses,
        arm_joints=DEFAULT_ARM_JOINTS,
    )

    parsed = yaml.safe_load(yaml_str)
    assert parsed["other_subsystem"]["key"] == 123
    assert parsed["manipulation_action_server"]["ros__parameters"]["max_velocity"] == 1.5
    assert "approach" in parsed["manipulation_action_server"]["ros__parameters"]["named_poses"]


def test_node_fsm_capture_undo_cycle(tmp_path) -> None:
    """Verify ArmPoseCalibratorNode FSM capture sequence and undo behavior."""
    import rclpy
    from lekiwi_calibration.arm.arm_pose_calibrator_node import ArmPoseCalibratorNode

    if not rclpy.ok():
        rclpy.init()

    out_file = str(tmp_path / "test_poses.yaml")
    node = ArmPoseCalibratorNode()
    node.output_yaml = out_file

    try:
        # Mock JointState
        js = JointState()
        js.name = list(DEFAULT_ARM_JOINTS)
        js.position = [0.0, -1.0, 1.0, 0.5, 0.0, 1.5]
        node.on_joint_state(js)

        # Initially step 0 (approach)
        assert node.current_step_idx == 0
        assert len(node.calibrated_poses) == 0

        # Capture 1: approach
        node.capture_active_pose()
        assert node.current_step_idx == 1
        assert "approach" in node.calibrated_poses

        # Capture 2: stow
        node.capture_active_pose()
        assert node.current_step_idx == 2
        assert "stow" in node.calibrated_poses

        # Undo stow
        node.undo_last_pose()
        assert node.current_step_idx == 1
        assert "stow" not in node.calibrated_poses
        assert "approach" in node.calibrated_poses

        # Re-capture stow
        node.capture_active_pose()
        assert node.current_step_idx == 2

        # Capture 3: clear_left
        node.capture_active_pose()
        assert node.current_step_idx == 3

        # Attempt to save before completing all 4 (should fail if require_all_poses=True)
        success, _ = node.save_poses_to_yaml()
        assert not success

        # Capture 4: clear_right
        node.capture_active_pose()
        assert node.current_step_idx == 4
        assert len(node.calibrated_poses) == 4

        # Save to YAML
        success, msg = node.save_poses_to_yaml()
        assert success
        assert "Successfully exported" in msg

        # Verify exported file
        with open(out_file, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        assert "manipulation_action_server" in data
        assert len(data["manipulation_action_server"]["ros__parameters"]["named_poses"]) == 4

    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_format_arm_poses_yaml_with_landmarks() -> None:
    """Verify output YAML contains both joint space and Cartesian landmarks blocks."""
    calibrated_poses = {
        "stow": [1.6582, -1.6214, 1.5815, 0.5768, 0.0169, 0.0905],
        "clear_left": [-1.4021, 0.0660, -0.0061, 1.1536, 0.1120, 0.0905],
        "clear_right": [1.5079, 0.0660, -0.0061, 1.1628, 0.1120, 0.0905],
    }
    landmarks = {
        "stow": [0.0704, -0.1882, 0.1757],
        "clear_left": [-0.0029, 0.1435, 0.1702],
        "clear_right": [-0.0027, -0.1451, 0.1633],
    }

    yaml_str = format_arm_poses_yaml(
        existing_data={},
        calibrated_poses=calibrated_poses,
        arm_joints=DEFAULT_ARM_JOINTS,
        calibrated_landmarks=landmarks,
        precision=4,
    )

    parsed = yaml.safe_load(yaml_str)
    assert "manipulation_action_server" in parsed
    assert "chessboard" in parsed
    assert "landmarks" in parsed["chessboard"]

    lm = parsed["chessboard"]["landmarks"]
    assert lm["stow"] == [0.0704, -0.1882, 0.1757]
    assert lm["clear_left"] == [-0.0029, 0.1435, 0.1702]
    assert lm["clear_right"] == [-0.0027, -0.1451, 0.1633]

    assert "chessboard:" in yaml_str
    assert "landmarks:" in yaml_str
    assert "stow: [0.0704, -0.1882, 0.1757]" in yaml_str


def test_node_cartesian_tf_capture(tmp_path) -> None:
    """Verify node captures Cartesian coordinates via TF and exports landmarks."""
    import rclpy
    from unittest.mock import MagicMock
    from geometry_msgs.msg import TransformStamped
    from lekiwi_calibration.arm.arm_pose_calibrator_node import ArmPoseCalibratorNode

    if not rclpy.ok():
        rclpy.init()

    out_file = str(tmp_path / "test_landmarks_poses.yaml")
    node = ArmPoseCalibratorNode()
    node.output_yaml = out_file

    try:
        # Mock TF Buffer lookup_transform
        mock_tf = TransformStamped()
        mock_tf.transform.translation.x = 0.0704
        mock_tf.transform.translation.y = -0.1882
        mock_tf.transform.translation.z = 0.1757
        node.tf_buffer.lookup_transform = MagicMock(return_value=mock_tf)

        js = JointState()
        js.name = list(DEFAULT_ARM_JOINTS)
        js.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        node.on_joint_state(js)

        # Capture approach
        node.capture_active_pose()
        assert "approach" in node.calibrated_landmarks
        assert node.calibrated_landmarks["approach"] == [0.0704, -0.1882, 0.1757]

        # Undo
        node.undo_last_pose()
        assert "approach" not in node.calibrated_landmarks
    finally:
        node.destroy_node()
        rclpy.shutdown()

