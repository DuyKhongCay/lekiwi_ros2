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


def test_simulated_action_dispatcher(ros_context):
    node = Node("test_sim_dispatcher_node")
    dispatcher = SimulatedActionDispatcher(node)
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

        # Navigation goal
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
    node = Node("test_active_obs_nav_node")
    dispatcher = SimulatedActionDispatcher(node)
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


def test_active_observation_navigator_accepts_pure_navigation_dispatcher(ros_context):
    """Verify ISP: ActiveObservationNavigator only requires INavigationDispatcher, not a fat interface."""

    class PureNavigationMock(INavigationDispatcher):
        def __init__(self):
            self.dispatched_poses = []
            self.cancel_called = False

        def send_navigation_goal(
            self,
            target_pose: PoseStamped,
            timeout_sec: float = 60.0,
            on_completed=None,
        ) -> bool:
            self.dispatched_poses.append(target_pose)
            if on_completed:
                on_completed(ActionResult(success=True, message="Pure nav ok"))
            return True

        def cancel_active_goal(self) -> None:
            self.cancel_called = True

    node = Node("test_isp_navigator_node")
    try:
        nav_dispatcher = PureNavigationMock()
        navigator = ActiveObservationNavigator(node=node, dispatcher=nav_dispatcher)

        dispatched = navigator.reposition_to_next_viewpoint(board_x=0.0, board_y=0.0)
        assert dispatched is True
        assert len(nav_dispatcher.dispatched_poses) == 1
        assert navigator.viewpoint_index == 1
    finally:
        node.destroy_node()


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
