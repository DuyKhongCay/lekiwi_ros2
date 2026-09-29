# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for MotionDispatcher and ActiveObservationNavigator."""

from __future__ import annotations

from unittest.mock import Mock

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from lekiwi_orchestrator.motion_dispatcher import (
    ActionDispatcherInterface,
    ActionResult,
    ActiveObservationNavigator,
    IFeasibilityChecker,
    IManipulationDispatcher,
    INavigationDispatcher,
    RosActionDispatcher,
    SimulatedActionDispatcher,
)
from lekiwi_orchestrator.move_pipeline import ChessMoveGoal
from rclpy.node import Node


@pytest.fixture(scope="module")
def ros_context():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_ros_action_dispatcher_mock_nav2(monkeypatch, ros_context):
    """Verify RosActionDispatcher with mock_nav2=True bypasses Nav2 without creating Nav2 client."""
    monkeypatch.setattr(
        "lekiwi_orchestrator.motion_dispatcher.ActionClient",
        lambda *args, **kwargs: Mock(),
    )
    node = Node("test_mock_nav_disp_node")
    dispatcher = RosActionDispatcher(node=node, mock_nav2=True)
    try:
        # Feasibility check
        feasibility_result = []
        success = dispatcher.check_feasibility(
            goal=ChessMoveGoal(uci="e2e4", from_square="e2", to_square="e4"),
            on_success=lambda resp: feasibility_result.append(resp.feasible),
        )
        assert success
        assert len(feasibility_result) == 1
        assert feasibility_result[0] is True
        assert dispatcher._nav2_client is None
        assert dispatcher._manipulation_client is not None

        # Navigation goal
        # Navigation goal should complete immediately with mock success
        nav_results = []
        target_pose = PoseStamped()
        target_pose.pose.position.x = 1.0
        success = dispatcher.send_navigation_goal(
            target_pose=target_pose,
            on_completed=lambda res: nav_results.append(res.success),
        )
        assert success
        assert len(nav_results) == 1
        assert nav_results[0] is True

        # Manipulation goal
        manip_results = []
        success = dispatcher.send_manipulation_goal(
            goal="test_goal",
            on_completed=lambda res: manip_results.append(res.success),
        )
        assert success
        assert len(manip_results) == 1
        assert manip_results[0] is True
    finally:
        dispatcher.destroy()
        node.destroy_node()


def test_active_observation_navigator(ros_context):
    class PureNavMock(INavigationDispatcher):
        def __init__(self):
            self.dispatched_poses = []

        def send_navigation_goal(
            self, target_pose, timeout_sec=60.0, on_completed=None
        ) -> bool:
            self.dispatched_poses.append(target_pose)
            if on_completed:
                on_completed(ActionResult(success=True, message="Mock nav ok"))
            return True

        def cancel_active_goal(self) -> None:
            pass

    node = Node("test_active_obs_nav_node")
    dispatcher = PureNavMock()
    navigator = ActiveObservationNavigator(
        node=node,
        dispatcher=dispatcher,
        map_frame="map",
        standoff_distance=0.65,
    )
    try:
        assert navigator.viewpoint_index == 0

        # Reposition 1: first viewpoint
        completed_results = []
        dispatched = navigator.reposition_to_next_viewpoint(
            board_x=1.0,
            board_y=1.0,
            on_completed=lambda res: completed_results.append(res),
        )
        assert dispatched
        assert navigator.viewpoint_index == 1
        assert len(completed_results) == 1
        assert completed_results[0].success is True

        # Reposition 2: second viewpoint
        dispatched_2 = navigator.reposition_to_next_viewpoint(
            board_x=1.0,
            board_y=1.0,
            on_completed=lambda res: completed_results.append(res),
        )
        assert dispatched_2
        assert navigator.viewpoint_index == 2
        assert len(completed_results) == 2

        # Reset
        navigator.reset_viewpoint_index()
        assert navigator.viewpoint_index == 0
    finally:
        node.destroy_node()


def test_segregated_interfaces_compliance():
    """Verify classes satisfy ISP segregated interfaces."""
    assert issubclass(RosActionDispatcher, IFeasibilityChecker)
    assert issubclass(RosActionDispatcher, INavigationDispatcher)
    assert issubclass(RosActionDispatcher, IManipulationDispatcher)
    assert issubclass(RosActionDispatcher, ActionDispatcherInterface)

    assert issubclass(SimulatedActionDispatcher, IFeasibilityChecker)
    assert issubclass(SimulatedActionDispatcher, INavigationDispatcher)
    assert issubclass(SimulatedActionDispatcher, IManipulationDispatcher)
    assert issubclass(SimulatedActionDispatcher, ActionDispatcherInterface)


def test_ros_action_dispatcher_initialization_and_destroy(monkeypatch):
    """Verify RosActionDispatcher initializes cleanly without lifecycle management and destroys clients."""
    monkeypatch.setattr(
        "lekiwi_orchestrator.motion_dispatcher.ActionClient",
        lambda *args, **kwargs: Mock(),
    )
    node = Mock()
    feas_client = Mock()
    node.create_client.return_value = feas_client

    dispatcher = RosActionDispatcher(node=node)
    assert dispatcher._feasibility_client is feas_client
    assert dispatcher._nav2_client is not None
    assert dispatcher._manipulation_client is not None

    dispatcher.destroy()
    node.destroy_client.assert_called_once_with(feas_client)
    assert dispatcher._feasibility_client is None
    assert dispatcher._nav2_client is None
    assert dispatcher._manipulation_client is None
