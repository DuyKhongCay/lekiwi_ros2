# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for MotionClient and ObsNavigator."""

from __future__ import annotations

import math
from unittest.mock import Mock

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from lekiwi_orchestrator.mission_types import (
    ActionResult,
    ChessMoveGoal,
    ObservationIntent,
)
from lekiwi_orchestrator.motion_client import (
    NavigationClient,
    RosMotionClient,
)
from lekiwi_orchestrator.obs_navigator import ObsNavigator
from rclpy.node import Node


@pytest.fixture(scope="module")
def ros_context():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_ros_action_dispatcher_mock_nav2(monkeypatch, ros_context):
    """Verify RosMotionClient with mock_nav2=True bypasses Nav2 without creating Nav2 client."""
    monkeypatch.setattr(
        "lekiwi_orchestrator.motion_client.ActionClient",
        lambda *args, **kwargs: Mock(),
    )
    node = Node("test_mock_nav_disp_node")
    dispatcher = RosMotionClient(node=node, mock_nav2=True)

    mock_feas = Mock()
    mock_feas.service_is_ready.return_value = True

    def fake_call_async(req):
        fut = Mock()
        fut.done.return_value = True
        fut.result.return_value = Mock(feasible=True)
        fut.add_done_callback = lambda cb: cb(fut)
        return fut

    mock_feas.call_async = fake_call_async
    dispatcher._feasibility_client = mock_feas

    mock_manip = Mock()
    mock_manip.server_is_ready.return_value = True

    def fake_send_goal(goal, feedback_callback=None):
        fut = Mock()
        fut.done.return_value = True
        gh = Mock(accepted=True)
        res_fut = Mock()
        res_val = Mock()
        res_val.success = True
        res_val.message = "OK"
        res_val.execution_time_sec = 0.1
        res_wrapped = Mock()
        res_wrapped.result = res_val
        res_fut.result.return_value = res_wrapped
        res_fut.add_done_callback = lambda cb: cb(res_fut)
        gh.get_result_async = lambda: res_fut
        fut.result.return_value = gh
        fut.add_done_callback = lambda cb: cb(fut)
        return fut

    mock_manip.send_goal_async = fake_send_goal
    dispatcher._manipulation_client = mock_manip

    try:
        # Feasibility check
        feasibility_result = []
        success = dispatcher.check_feasibility(
            goal=ChessMoveGoal(uci="e2e4", from_square="e2", to_square="e4"),
            on_success=lambda resp: feasibility_result.append(resp.feasible),
        )
        assert success
        assert feasibility_result == [True]
        assert dispatcher._nav2_client is None
        assert dispatcher._manipulation_client is not None

        # Navigation goal completes immediately in mock mode
        nav_results = []
        target_pose = PoseStamped()
        target_pose.pose.position.x = 1.0
        success = dispatcher.send_navigation_goal(
            target_pose=target_pose,
            on_completed=lambda res: nav_results.append(res.success),
        )
        assert success
        assert nav_results == [True]

        # Manipulation goal
        manip_results = []
        success = dispatcher.send_manipulation_goal(
            goal="test_goal",
            on_completed=lambda res: manip_results.append(res.success),
        )
        assert success
        assert manip_results == [True]
    finally:
        dispatcher.destroy()
        node.destroy_node()


@pytest.mark.parametrize(
    ("status", "result_obj", "expected_success", "expected_msg_snippet"),
    [
        (
            GoalStatus.STATUS_SUCCEEDED,
            type("Nav2Result", (), {"error_code": 0, "execution_time_sec": 1.25})(),
            True,
            "Action succeeded",
        ),
        (
            GoalStatus.STATUS_ABORTED,
            type("Nav2Result", (), {"error_code": 100})(),
            False,
            f"Action failed with status {GoalStatus.STATUS_ABORTED}",
        ),
        (
            GoalStatus.STATUS_SUCCEEDED,
            type(
                "CustomManipResult",
                (),
                {"success": False, "error_msg": "Kinematic limit reached", "execution_time_sec": 0.5},
            )(),
            False,
            "Kinematic limit reached",
        ),
    ],
)
def test_wrap_completed_action_status_matrix(
    monkeypatch, status, result_obj, expected_success, expected_msg_snippet
):
    """Verify standard and custom ROS 2 Action result unpacking across status codes."""
    monkeypatch.setattr(
        "lekiwi_orchestrator.motion_client.ActionClient",
        lambda *args, **kwargs: Mock(),
    )
    dispatcher = RosMotionClient(node=Mock(), mock_nav2=True)

    mock_wrapped_res = Mock()
    mock_wrapped_res.status = status
    mock_wrapped_res.result = result_obj

    mock_future = Mock()
    mock_future.result.return_value = mock_wrapped_res

    outcomes = []
    dispatcher._wrap_completed(
        future=mock_future,
        on_completed=lambda res: outcomes.append(res),
    )

    assert len(outcomes) == 1
    result = outcomes[0]
    assert result.success is expected_success
    assert expected_msg_snippet in result.message


def test_obs_navigator_direct_geodesic_reposition(ros_context):
    """Verify reposition_to_next_viewpoint directly dispatches to calibrated angle_offsets in single goal."""
    dispatched_poses = []

    class DirectTrackingNav(NavigationClient):
        def send_navigation_goal(self, target_pose, timeout_sec=60.0, on_completed=None) -> bool:
            dispatched_poses.append(target_pose)
            if on_completed:
                on_completed(ActionResult(success=True, message="Reached viewpoint"))
            return True

        def cancel_active_goal(self) -> None:
            pass

    node = Node("test_direct_geodesic_node")
    dispatcher = DirectTrackingNav()
    candidates = [0.0, 0.314, -0.314]
    navigator = ObsNavigator(
        node=node,
        dispatcher=dispatcher,
        map_frame="map",
        board_frame="chessboard_frame",
        standoff_distance=0.65,
        angle_offsets_map={ObservationIntent.POST_MOVE_VERIFY: candidates},
    )

    try:
        # Robot is inside standoff circle at (0.35, 0.0) relative to board
        ref_pose = PoseStamped()
        ref_pose.header.frame_id = "chessboard_frame"
        ref_pose.pose.position.x = 0.35
        ref_pose.pose.position.y = 0.0

        final_results = []
        dispatched = navigator.reposition_to_next_viewpoint(
            intent=ObservationIntent.POST_MOVE_VERIFY,
            reference_pose=ref_pose,
            on_completed=lambda res: final_results.append(res),
        )

        # Directly dispatched to closest candidate (0.0 rad) without intermediate uncalibrated retreat
        assert dispatched is True
        assert len(dispatched_poses) == 1
        vp_goal = dispatched_poses[0]
        assert math.isclose(vp_goal.pose.position.x, 0.65, abs_tol=1e-3)
        assert math.isclose(vp_goal.pose.position.y, 0.0, abs_tol=1e-3)
        assert len(final_results) == 1
        assert final_results[0].success is True
        assert navigator.get_viewpoint_index(ObservationIntent.POST_MOVE_VERIFY) == 1
    finally:
        node.destroy_node()


def test_compute_observation_pose_selects_closest_calibrated_viewpoint(ros_context):
    """Verify compute_observation_pose_for_base selects closest angle_offsets viewpoint and advances index."""
    node = Node("test_compute_obs_calibrated_node")
    dispatcher = Mock()
    candidates = [math.pi / 2, -math.pi / 2, 0.0]
    navigator = ObsNavigator(
        node=node,
        dispatcher=dispatcher,
        map_frame="map",
        board_frame="chessboard_frame",
        standoff_distance=0.65,
        angle_offsets_map={ObservationIntent.POST_MOVE_VERIFY: candidates},
    )

    try:
        # Manipulation base at South (y = -0.4, angle = -pi/2)
        base_pose = PoseStamped()
        base_pose.header.frame_id = "chessboard_frame"
        base_pose.pose.position.x = 0.0
        base_pose.pose.position.y = -0.4

        obs_pose = navigator.compute_observation_pose_for_base(base_pose)
        assert obs_pose is not None

        # Must select candidate -pi/2 (closest to South)
        assert math.isclose(obs_pose.pose.position.x, 0.0, abs_tol=1e-3)
        assert math.isclose(obs_pose.pose.position.y, -0.65, abs_tol=1e-3)

        # viewpoint_index must be advanced to 1 for next reposition
        assert navigator.get_viewpoint_index(ObservationIntent.POST_MOVE_VERIFY) == 1
    finally:
        node.destroy_node()


def test_azimuth_viewpoint_ranking_and_exhaustion(ros_context):
    """Verify that viewpoints are ranked by azimuth distance and exhaustion is tracked."""
    dispatched_poses = []

    class MockNav(NavigationClient):
        def send_navigation_goal(self, target_pose, timeout_sec=60.0, on_completed=None) -> bool:
            dispatched_poses.append(target_pose)
            if on_completed:
                on_completed(ActionResult(success=True, message="OK"))
            return True

        def cancel_active_goal(self) -> None:
            pass

    node = Node("test_azimuth_ranking_node")
    candidates = [-math.pi / 2.0, 0.0, math.pi / 2.0]
    navigator = ObsNavigator(
        node=node,
        dispatcher=MockNav(),
        standoff_distance=1.0,
        angle_offsets_map={ObservationIntent.RELOCALIZE: candidates},
    )

    try:
        assert navigator.get_max_attempts(ObservationIntent.RELOCALIZE) == 3
        assert navigator.has_exhausted_viewpoints(ObservationIntent.RELOCALIZE) is False

        ref_pose = PoseStamped()
        ref_pose.header.frame_id = "chessboard_frame"
        ref_pose.pose.position.x = 0.0
        ref_pose.pose.position.y = 1.0

        # Attempt 1: closest candidate pi/2 (distance 0)
        navigator.reposition_to_next_viewpoint(intent=ObservationIntent.RELOCALIZE, reference_pose=ref_pose)
        assert len(dispatched_poses) == 1
        p1 = dispatched_poses[0].pose.position
        assert math.isclose(p1.x, 0.0, abs_tol=1e-2)
        assert math.isclose(p1.y, 1.0, abs_tol=1e-2)

        # Attempt 2: next closest is candidate 0.0 (distance pi/2)
        navigator.reposition_to_next_viewpoint(intent=ObservationIntent.RELOCALIZE, reference_pose=ref_pose)
        assert len(dispatched_poses) == 2
        p2 = dispatched_poses[1].pose.position
        assert math.isclose(p2.x, 1.0, abs_tol=1e-2)
        assert math.isclose(p2.y, 0.0, abs_tol=1e-2)

        # Attempt 3: farthest candidate -pi/2 (distance pi) -> triggers exhaustion
        navigator.reposition_to_next_viewpoint(intent=ObservationIntent.RELOCALIZE, reference_pose=ref_pose)
        assert len(dispatched_poses) == 3
        assert navigator.has_exhausted_viewpoints(ObservationIntent.RELOCALIZE) is True
    finally:
        node.destroy_node()


def test_obs_navigator_cancellation_and_tf_fallback(ros_context):
    """Verify ObsNavigator cancellation and TF exception fallback handling."""
    node = Node("test_obs_nav_cancel_tf_node")
    mock_dispatcher = Mock()
    mock_dispatcher.send_navigation_goal.return_value = True

    mock_tf_buffer = Mock()
    mock_transform = Mock()
    mock_transform.transform.translation.x = 0.65
    mock_transform.transform.translation.y = 0.0
    mock_tf_buffer.lookup_transform.return_value = mock_transform

    mock_tf_pose = PoseStamped()
    mock_tf_pose.header.frame_id = "map"
    mock_tf_pose.pose.position.x = 2.0
    mock_tf_pose.pose.position.y = 3.0
    mock_tf_buffer.transform.return_value = mock_tf_pose

    navigator = ObsNavigator(
        node=node,
        dispatcher=mock_dispatcher,
        standoff_distance=0.65,
        tf_buffer=mock_tf_buffer,
    )

    try:
        # 1. Cancellation propagation
        navigator.cancel()
        mock_dispatcher.cancel_active_goal.assert_called_once()

        # 2. TF lookup and transform succeeds
        navigator.reposition_to_next_viewpoint(intent=ObservationIntent.POST_MOVE_VERIFY)
        mock_tf_buffer.lookup_transform.assert_called_once()
        mock_tf_buffer.transform.assert_called_once()

        # 3. TF lookup/transform failure fails safe
        mock_tf_buffer.lookup_transform.side_effect = RuntimeError("TF timeout")
        mock_tf_buffer.transform.side_effect = RuntimeError("TF transform error")
        ref_pose = PoseStamped()
        ref_pose.header.frame_id = "map"
        ref_pose.pose.position.x = 1.0
        ref_pose.pose.position.y = 0.0
        success = navigator.reposition_to_next_viewpoint(
            intent=ObservationIntent.POST_MOVE_VERIFY,
            reference_pose=ref_pose,
        )
        assert success is False
    finally:
        node.destroy_node()
