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
from lekiwi_orchestrator.core.mission_types import (
    ActionResult,
    ChessMoveGoal,
    ObservationIntent,
)
from lekiwi_orchestrator.core.utils import get_node_now_sec
from lekiwi_orchestrator.execution.motion_client import (
    FakeMotionClient,
    NavigationClient,
    RosMotionClient,
)
from lekiwi_orchestrator.planning.obs_navigator import ObsNavigator
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
        "lekiwi_orchestrator.execution.motion_client.ActionClient",
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
            goal=ChessMoveGoal(uci="e2e4"),
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

        # Action watchdog disabled when <= 0.0
        dispatcher._start_action_watchdog("Test zero timeout", timeout_sec=0.0)
        assert dispatcher._action_timer is None
        dispatcher._start_action_watchdog("Test negative timeout", timeout_sec=-1.0)
        assert dispatcher._action_timer is None
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
                {"success": False, "message": "Kinematic limit reached", "execution_time_sec": 0.5},
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
        "lekiwi_orchestrator.execution.motion_client.ActionClient",
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


def test_ros_motion_client_is_navigating_lifecycle(monkeypatch, ros_context):
    """Verify is_navigating correctly reflects Nav2 goal accepted, completed, cancelled, and rejected states."""
    monkeypatch.setattr(
        "lekiwi_orchestrator.execution.motion_client.ActionClient",
        lambda *args, **kwargs: Mock(),
    )
    node = Node("test_is_navigating_node")
    dispatcher = RosMotionClient(node=node, mock_nav2=False)

    mock_nav = Mock()
    mock_nav.server_is_ready.return_value = True
    dispatcher._nav2_client = mock_nav

    try:
        assert not dispatcher.is_navigating

        # 1. Goal accepted -> is_navigating becomes True
        res_callback_holder = []
        fake_goal_handle = Mock(accepted=True)
        res_fut = Mock()
        res_fut.add_done_callback = lambda cb: res_callback_holder.append(cb)
        fake_goal_handle.get_result_async.return_value = res_fut

        send_fut = Mock()
        send_fut.result.return_value = fake_goal_handle
        mock_nav.send_goal_async.return_value = send_fut

        target_pose = PoseStamped()
        success = dispatcher.send_navigation_goal(target_pose=target_pose)
        assert success
        # Fire goal response callback
        send_fut.add_done_callback.call_args[0][0](send_fut)
        assert dispatcher.is_navigating

        # 2. Complete goal -> is_navigating becomes False
        assert len(res_callback_holder) == 1
        res_obj = Mock()
        res_obj.status = GoalStatus.STATUS_SUCCEEDED
        res_obj.result = Mock(success=True, message="Reached", execution_time_sec=1.0)
        res_fut.result.return_value = res_obj
        res_callback_holder[0](res_fut)
        assert not dispatcher.is_navigating

        # 3. Goal cancelled -> is_navigating becomes False
        send_fut.add_done_callback.call_args[0][0](send_fut)
        assert dispatcher.is_navigating
        dispatcher.cancel_active_goal()
        assert not dispatcher.is_navigating

        # 4. Goal rejected -> is_navigating remains False
        rejected_handle = Mock(accepted=False)
        rejected_fut = Mock()
        rejected_fut.result.return_value = rejected_handle
        mock_nav.send_goal_async.return_value = rejected_fut
        success = dispatcher.send_navigation_goal(target_pose=target_pose)
        assert success
        rejected_fut.add_done_callback.call_args[0][0](rejected_fut)
        assert not dispatcher.is_navigating
    finally:
        dispatcher.destroy()
        node.destroy_node()


def test_fake_motion_client():
    """Verify FakeMotionClient contract and state tracking for unit testing."""
    client = FakeMotionClient()
    assert not client.is_navigating
    assert not client.cancelled
    assert len(client.nav_goals) == 0
    assert len(client.manip_goals) == 0

    assert client.check_feasibility(ChessMoveGoal("e2e4"))
    assert client.send_navigation_goal(PoseStamped())
    assert len(client.nav_goals) == 1
    assert client.send_manipulation_goal(Mock(instruction="move"))
    assert len(client.manip_goals) == 1

    client.cancel_active_goal()
    assert client.cancelled
    client.destroy()


