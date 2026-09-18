# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for LeRobotArmBridge adapter and calibration units conversion."""

import math
import tempfile
from pathlib import Path
import pytest
import rclpy
from sensor_msgs.msg import JointState

from lekiwi_manipulation.lerobot_arm_bridge import (
    ARM_JOINTS,
    JointCalibration,
    LeRobotArmBridge,
    RADIANS_PER_RAW,
    RAW_POSITION_SPAN,
    load_joint_config,
)


@pytest.fixture(scope="module")
def ros_context():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_load_joint_config_valid():
    yaml_content = """
joints:
  arm_shoulder_pan: {range_min: 1000, range_max: 3000}
  arm_shoulder_lift: {range_min: 1000, range_max: 3000}
  arm_elbow_flex: {range_min: 1000, range_max: 3000}
  arm_wrist_flex: {range_min: 1000, range_max: 3000}
  arm_wrist_roll: {range_min: 1000, range_max: 3000}
  arm_gripper: {range_min: 2000, range_max: 3000}
"""
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
        f.write(yaml_content)
        temp_path = f.name

    try:
        calib = load_joint_config(temp_path)
        assert len(calib) == len(ARM_JOINTS)
        assert math.isclose(calib["arm_shoulder_pan"].midpoint, 2000.0)
        assert math.isclose(calib["arm_gripper"].midpoint, 2500.0)
    finally:
        Path(temp_path).unlink(missing_ok=True)


def test_load_joint_config_invalid():
    with pytest.raises(ValueError):
        load_joint_config("/non_existent_file.yaml")

    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
        f.write("invalid: yaml: [")
        bad_yaml = f.name

    try:
        with pytest.raises(ValueError):
            load_joint_config(bad_yaml)
    finally:
        Path(bad_yaml).unlink(missing_ok=True)


def test_bridge_raw_to_radians_conversion(ros_context):
    node = LeRobotArmBridge()
    try:
        # Default midpoint = 2048.0
        # At midpoint (2048), radians should be 0.0
        rad_mid = node._to_radians("arm_shoulder_pan", 2048.0)
        assert math.isclose(rad_mid, 0.0, abs_tol=1e-6)

        # Above midpoint by 1024 ticks -> (1024 / 4095) * 2pi ~= 1.57 rad (pi/2)
        rad_high = node._to_radians("arm_shoulder_pan", 2048.0 + 1023.75)
        assert math.isclose(rad_high, math.pi / 2.0, abs_tol=1e-3)
    finally:
        node.destroy_node()


def test_named_values_validation(ros_context):
    node = LeRobotArmBridge()
    try:
        # Complete message
        msg = JointState()
        msg.name = list(ARM_JOINTS)
        msg.position = [0.0] * len(ARM_JOINTS)
        values = node._named_values(msg)
        assert values is not None
        assert len(values) == len(ARM_JOINTS)

        # Missing a joint
        msg_missing = JointState()
        msg_missing.name = list(ARM_JOINTS[:-1])
        msg_missing.position = [0.0] * len(msg_missing.name)
        assert node._named_values(msg_missing) is None

        # Mismatched lengths
        msg_bad = JointState()
        msg_bad.name = list(ARM_JOINTS)
        msg_bad.position = [0.0, 1.0]  # Only 2 positions for 6 joints
        assert node._named_values(msg_bad) is None
    finally:
        node.destroy_node()
