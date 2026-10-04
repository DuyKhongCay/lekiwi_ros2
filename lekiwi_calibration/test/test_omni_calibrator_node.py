# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests validating TF tree validation and readiness logic in OmniBaseCalibratorNode."""

import sys
import unittest
from unittest.mock import MagicMock

# Provide mock ROS 2 runtime if rclpy is missing
if "rclpy" not in sys.modules:
    try:
        import rclpy  # noqa: F401
    except ImportError:
        mock_rclpy = MagicMock()
        mock_node_mod = MagicMock()
        mock_qos_mod = MagicMock()

        class _MockNode:
            """Minimal Mock Node for testing without ROS 2 binary middleware."""

            def __init__(self, node_name: str = "test_node", *args, **kwargs):
                self._node_name = node_name
                self._logger = MagicMock()
                self._params = {}

            def get_name(self):
                return self._node_name

            def get_logger(self):
                return self._logger

            def declare_parameter(self, name, val):
                class _Param:
                    def __init__(self, v):
                        self.value = v

                self._params[name] = _Param(val)

            def get_parameter(self, name):
                class _Param:
                    def __init__(self, v):
                        self.value = v

                return self._params.get(name, _Param(None))

            def create_publisher(self, *args, **kwargs):
                return MagicMock()

            def create_subscription(self, *args, **kwargs):
                return MagicMock()

            def create_client(self, *args, **kwargs):
                return MagicMock()

            def destroy_node(self):
                pass

        mock_node_mod.Node = _MockNode
        sys.modules["rclpy"] = mock_rclpy
        sys.modules["rclpy.node"] = mock_node_mod
        sys.modules["rclpy.executors"] = MagicMock()
        sys.modules["rclpy.time"] = MagicMock()
        sys.modules["rclpy.time"].Time = MagicMock

for mod_name in [
    "rclpy",
    "rclpy.node",
    "rclpy.executors",
    "rclpy.time",
    "geometry_msgs",
    "geometry_msgs.msg",
    "nav_msgs",
    "nav_msgs.msg",
    "sensor_msgs",
    "sensor_msgs.msg",
    "rcl_interfaces",
    "rcl_interfaces.msg",
    "rcl_interfaces.srv",
    "tf2_ros",
]:
    if mod_name not in sys.modules:
        sys.modules[mod_name] = MagicMock()

if "rclpy.time" in sys.modules and not hasattr(sys.modules["rclpy.time"], "Time"):
    sys.modules["rclpy.time"].Time = MagicMock

from lekiwi_calibration.omni.omni_base_calibrator_node import OmniBaseCalibratorNode


class TestOmniBaseCalibratorNodeTfTree(unittest.TestCase):
    """Test suite verifying TF tree validation and readiness checks."""

    def setUp(self):
        self.node = OmniBaseCalibratorNode()
        self.node.tf_buffer = MagicMock()

    def test_initial_is_ready_false(self):
        """Initial state without odom and imu messages should not be ready."""
        self.assertFalse(self.node.is_ready())

    def test_check_tf_tree_missing_odom_base(self):
        """Should fail if odom -> base_footprint transform is missing."""
        self.node.tf_buffer.can_transform.return_value = False
        ok, reason = self.node.check_tf_tree()
        self.assertFalse(ok)
        self.assertIn("Missing transform: odom -> base_footprint", reason)

    def test_check_tf_tree_missing_map_odom_when_required(self):
        """Should fail if map -> odom transform is missing when require_tf_tree is True."""
        def can_transform_mock(target, source, time_pt):
            if target == "odom" and source == "base_footprint":
                return True
            if target == "map" and source == "odom":
                return False
            return False

        self.node.tf_buffer.can_transform.side_effect = can_transform_mock
        self.node._require_tf_tree = True
        ok, reason = self.node.check_tf_tree()
        self.assertFalse(ok)
        self.assertIn("Missing transform: map -> odom", reason)

    def test_check_tf_tree_success(self):
        """Should succeed when both transforms exist."""
        self.node.tf_buffer.can_transform.return_value = True
        ok, reason = self.node.check_tf_tree()
        self.assertTrue(ok)
        self.assertEqual(reason, "TF tree complete")

    def test_check_tf_tree_bypass_map_odom_when_not_required(self):
        """Should succeed if require_tf_tree is False even if map -> odom is missing."""
        def can_transform_mock(target, source, time_pt):
            if target == "odom" and source == "base_footprint":
                return True
            return False

        self.node.tf_buffer.can_transform.side_effect = can_transform_mock
        self.node._require_tf_tree = False
        ok, reason = self.node.check_tf_tree()
        self.assertTrue(ok)
        self.assertEqual(reason, "TF tree complete")


if __name__ == "__main__":
    unittest.main()
