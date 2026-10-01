# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for PerceptionManager and ObservationGeometry."""

from __future__ import annotations

import math

import pytest
import rclpy
from lekiwi_interfaces.msg import PerceptionContext
from lekiwi_interfaces.srv import SetPerceptionContext
from lekiwi_orchestrator.perception_manager import (
    PerceptionContextCoordinator,
    compute_observation_pose,
    generate_candidate_observation_poses,
    yaw_to_quaternion,
)
from rclpy.node import Node


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


def test_compute_observation_pose_black_baseline():
    pose = compute_observation_pose(
        board_x=0.0,
        board_y=0.0,
        board_yaw=0.0,
        standoff_distance=0.65,
        angle_offset=0.0,
        robot_color="b",
    )
    # Black sits at +Y, facing -Y towards (0, 0)
    assert abs(pose.pose.position.x) < 1e-5
    assert abs(pose.pose.position.y - 0.65) < 1e-5
    # facing_yaw = -pi/2 -> quaternion z = sin(-pi/4) = -0.7071, w = cos(-pi/4) = 0.7071
    assert math.isclose(pose.pose.orientation.z, -math.sin(math.pi / 4.0), abs_tol=1e-4)
    assert math.isclose(pose.pose.orientation.w, math.cos(math.pi / 4.0), abs_tol=1e-4)


def test_compute_observation_pose_white_baseline():
    pose = compute_observation_pose(
        board_x=0.0,
        board_y=0.0,
        board_yaw=0.0,
        standoff_distance=0.65,
        angle_offset=0.0,
        robot_color="w",
    )
    # White sits at -Y, facing +Y towards (0, 0)
    assert abs(pose.pose.position.x) < 1e-5
    assert abs(pose.pose.position.y - (-0.65)) < 1e-5
    # facing_yaw = +pi/2 -> quaternion z = sin(pi/4) = 0.7071, w = cos(pi/4) = 0.7071
    assert math.isclose(pose.pose.orientation.z, math.sin(math.pi / 4.0), abs_tol=1e-4)
    assert math.isclose(pose.pose.orientation.w, math.cos(math.pi / 4.0), abs_tol=1e-4)


def test_generate_candidate_observation_poses_black_stays_in_hemisphere():
    poses = generate_candidate_observation_poses(
        board_x=0.0,
        board_y=0.0,
        board_yaw=0.0,
        standoff_distance=0.65,
        robot_color="b",
    )
    assert len(poses) == 5
    for p in poses:
        dist = math.hypot(p.pose.position.x, p.pose.position.y)
        assert abs(dist - 0.65) < 1e-4
        # All candidate poses for Black MUST remain strictly on Black's side (y > 0)
        assert p.pose.position.y > 0.50


def test_generate_candidate_observation_poses_white_stays_in_hemisphere():
    poses = generate_candidate_observation_poses(
        board_x=0.0,
        board_y=0.0,
        board_yaw=0.0,
        standoff_distance=0.65,
        robot_color="w",
    )
    assert len(poses) == 5
    for p in poses:
        dist = math.hypot(p.pose.position.x, p.pose.position.y)
        assert abs(dist - 0.65) < 1e-4
        # All candidate poses for White MUST remain strictly on White's side (y < 0)
        assert p.pose.position.y < -0.50


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
