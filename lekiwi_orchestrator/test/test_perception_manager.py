# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for PerceptionManager and ObservationGeometry."""

from __future__ import annotations

import math

import pytest
import rclpy
from lekiwi_orchestrator.perception_manager import (
    PerceptionContextCoordinator,
    compute_observation_pose,
    generate_candidate_observation_poses,
    yaw_to_quaternion,
)
from rclpy.node import Node

from lekiwi_interfaces.msg import PerceptionContext
from lekiwi_interfaces.srv import SetPerceptionContext


@pytest.fixture(scope="module")
def ros_context():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_yaw_to_quaternion():
    q0 = yaw_to_quaternion(0.0)
    assert q0.x == 0.0
    assert q0.y == 0.0
    assert abs(q0.z) < 1e-6
    assert abs(q0.w - 1.0) < 1e-6

    q_pi = yaw_to_quaternion(math.pi)
    assert abs(q_pi.z - 1.0) < 1e-6
    assert abs(q_pi.w) < 1e-6


def test_compute_observation_pose_baseline():
    pose = compute_observation_pose(
        board_x=1.0,
        board_y=2.0,
        board_yaw=0.0,
        standoff_distance=0.6,
        angle_offset=0.0,
    )
    assert abs(pose.pose.position.x - 0.4) < 1e-5
    assert abs(pose.pose.position.y - 2.0) < 1e-5
    assert abs(pose.pose.orientation.z) < 1e-5
    assert abs(pose.pose.orientation.w - 1.0) < 1e-5


def test_compute_observation_pose_quarter_turn():
    pose = compute_observation_pose(
        board_x=0.0,
        board_y=0.0,
        board_yaw=0.0,
        standoff_distance=1.0,
        angle_offset=math.pi / 2.0,
    )
    assert abs(pose.pose.position.x) < 1e-5
    assert abs(pose.pose.position.y - (-1.0)) < 1e-5
    half_angle = (math.pi / 2.0) * 0.5
    assert abs(pose.pose.orientation.z - math.sin(half_angle)) < 1e-5
    assert abs(pose.pose.orientation.w - math.cos(half_angle)) < 1e-5


def test_generate_candidate_observation_poses():
    poses = generate_candidate_observation_poses(
        board_x=0.5,
        board_y=0.5,
        board_yaw=0.0,
        standoff_distance=0.7,
    )
    assert len(poses) == 5
    for p in poses:
        dist = math.hypot(
            p.pose.position.x - 0.5,
            p.pose.position.y - 0.5,
        )
        assert abs(dist - 0.7) < 1e-4


def test_perception_coordinator_lifecycle(ros_context):
    node = Node("test_perception_coordinator_node")
    try:
        coordinator = PerceptionContextCoordinator(
            node=node,
            perception_context_topic="/test/perception_context",
            set_perception_service_name="/test/set_context",
        )
        assert coordinator.context == PerceptionContext.IDLE_STANDBY

        # Valid transition to TF_TRACKING_AND_NAV
        success = coordinator.set_context(PerceptionContext.TF_TRACKING_AND_NAV)
        assert success
        assert coordinator.context == PerceptionContext.TF_TRACKING_AND_NAV

        # Valid transition to BOARD_STATE_SCAN
        success = coordinator.set_context(PerceptionContext.BOARD_STATE_SCAN)
        assert success
        assert coordinator.context == PerceptionContext.BOARD_STATE_SCAN

        # Illegal direct jump to CALIBRATION_STREAM
        success_bad = coordinator.set_context(PerceptionContext.CALIBRATION_STREAM)
        assert not success_bad
        assert coordinator.context == PerceptionContext.BOARD_STATE_SCAN

        # Test service callback
        req = SetPerceptionContext.Request()
        req.requested_context.value = PerceptionContext.MANIPULATION_ACTOR
        resp = SetPerceptionContext.Response()
        result_resp = coordinator.handle_set_context_service(req, resp)
        assert result_resp.success
        assert coordinator.context == PerceptionContext.MANIPULATION_ACTOR
        assert result_resp.applied_context.value == PerceptionContext.MANIPULATION_ACTOR
    finally:
        node.destroy_node()
