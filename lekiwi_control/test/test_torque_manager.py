# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

import pytest
import rclpy
from lekiwi_interfaces.srv import SetTorqueEnabled
from lekiwi_control.torque_manager import (
    TorqueManagerNode,
    DEFAULT_ARM_JOINTS,
    DEFAULT_BASE_JOINTS,
)


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_torque_manager_target_arm(ros_context):
    node = TorqueManagerNode()
    try:
        # Request disable arm torque
        req = SetTorqueEnabled.Request()
        req.target = SetTorqueEnabled.Request.TARGET_ARM
        req.enabled = False
        resp = SetTorqueEnabled.Response()

        result = node._handle_set_torque_enabled(req, resp)
        assert result.success is True
        assert "Arm torque disabled" in result.message

        # Arm joints should be 0.0, base joints 1.0
        for i in range(len(DEFAULT_ARM_JOINTS)):
            assert node._current_torque_states[i] == 0.0
        for i in range(len(DEFAULT_ARM_JOINTS), node.total_joints):
            assert node._current_torque_states[i] == 1.0

        # Request enable arm torque
        req.enabled = True
        result = node._handle_set_torque_enabled(req, resp)
        assert result.success is True
        assert "Arm torque enabled" in result.message
        for i in range(node.total_joints):
            assert node._current_torque_states[i] == 1.0
    finally:
        node.destroy_node()


def test_torque_manager_target_base(ros_context):
    node = TorqueManagerNode()
    try:
        req = SetTorqueEnabled.Request()
        req.target = SetTorqueEnabled.Request.TARGET_BASE
        req.enabled = False
        resp = SetTorqueEnabled.Response()

        result = node._handle_set_torque_enabled(req, resp)
        assert result.success is True
        assert "Base torque disabled" in result.message

        # Arm joints should be 1.0, base joints 0.0
        for i in range(len(DEFAULT_ARM_JOINTS)):
            assert node._current_torque_states[i] == 1.0
        for i in range(len(DEFAULT_ARM_JOINTS), node.total_joints):
            assert node._current_torque_states[i] == 0.0
    finally:
        node.destroy_node()


def test_torque_manager_invalid_target(ros_context):
    node = TorqueManagerNode()
    try:
        req = SetTorqueEnabled.Request()
        req.target = 99  # Invalid
        req.enabled = False
        resp = SetTorqueEnabled.Response()

        result = node._handle_set_torque_enabled(req, resp)
        assert result.success is False
        assert "Invalid target" in result.message
    finally:
        node.destroy_node()
