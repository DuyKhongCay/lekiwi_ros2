# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for PerceptionContextManager."""

from __future__ import annotations

import pytest
import rclpy
from lekiwi_interfaces.msg import PerceptionContext
from lekiwi_interfaces.srv import SetPerceptionContext
from lekiwi_orchestrator.perception_context import PerceptionContextManager
from rclpy.node import Node


@pytest.fixture(scope="module")
def ros_context():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_perception_manager_lifecycle(ros_context):
    node = Node("test_perception_manager_node")
    try:
        manager = PerceptionContextManager(
            node=node,
            perception_context_topic="/test/perception_context",
            set_perception_service_name="/test/set_context",
        )
        assert manager.context == PerceptionContext.IDLE_STANDBY

        # Valid transition to TF_TRACKING_AND_NAV
        success = manager.set_context(PerceptionContext.TF_TRACKING_AND_NAV)
        assert success
        assert manager.context == PerceptionContext.TF_TRACKING_AND_NAV

        # Valid transition to BOARD_STATE_SCAN
        success = manager.set_context(PerceptionContext.BOARD_STATE_SCAN)
        assert success
        assert manager.context == PerceptionContext.BOARD_STATE_SCAN

        # Illegal direct jump to CALIBRATION_STREAM
        success_bad = manager.set_context(PerceptionContext.CALIBRATION_STREAM)
        assert not success_bad
        assert manager.context == PerceptionContext.BOARD_STATE_SCAN

        # Test service callback
        req = SetPerceptionContext.Request()
        req.requested_context.value = PerceptionContext.MANIPULATION_ACTOR
        resp = SetPerceptionContext.Response()
        result_resp = manager.handle_set_context_service(req, resp)
        assert result_resp.success
        assert manager.context == PerceptionContext.MANIPULATION_ACTOR
        assert result_resp.applied_context.value == PerceptionContext.MANIPULATION_ACTOR

    finally:
        node.destroy_node()
