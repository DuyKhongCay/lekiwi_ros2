# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for MotionClient and ObsNavigator."""

from __future__ import annotations

import math
from unittest.mock import Mock

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from lekiwi_orchestrator.mission_types import (
    ActionResult,
    ChessMoveGoal,
    ObservationIntent,
)
from lekiwi_orchestrator.motion_client import (
    FakeMotionClient,
    FeasibilityClient,
    ManipulationClient,
    MotionClient,
    NavigationClient,
    RosMotionClient,
    _MockGoalHandle,
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
    class PureNavMock(NavigationClient):
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
    navigator = ObsNavigator(
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



def test_wrap_completed_nav2_standard_action_status_succeeded(monkeypatch):
    """Verify standard ROS 2 Action without result.success (e.g. Nav2 NavigateToPose) evaluates GoalStatus.STATUS_SUCCEEDED as True."""
    from action_msgs.msg import GoalStatus

    monkeypatch.setattr(
        "lekiwi_orchestrator.motion_client.ActionClient",
        lambda *args, **kwargs: Mock(),
    )
    node = Mock()
    dispatcher = RosMotionClient(node=node, mock_nav2=True)

    # Simulated Nav2 result object (has no 'success' field, only error_code or empty)
    class Nav2Result:
        error_code = 0
        execution_time_sec = 1.25

    mock_wrapped_res = Mock()
    mock_wrapped_res.status = GoalStatus.STATUS_SUCCEEDED
    mock_wrapped_res.result = Nav2Result()

    mock_future = Mock()
    mock_future.result.return_value = mock_wrapped_res

    outcomes = []
    dispatcher._wrap_completed(
        future=mock_future,
        on_completed=lambda res: outcomes.append(res),
    )

    assert len(outcomes) == 1
    result = outcomes[0]
    assert result.success is True
    assert result.message == "Action succeeded"
    assert math.isclose(result.execution_time_sec, 1.25, abs_tol=1e-3)


def test_wrap_completed_nav2_standard_action_status_failed(monkeypatch):
    """Verify standard ROS 2 Action without result.success evaluates aborted status as False with meaningful message."""
    from action_msgs.msg import GoalStatus

    monkeypatch.setattr(
        "lekiwi_orchestrator.motion_client.ActionClient",
        lambda *args, **kwargs: Mock(),
    )
    node = Mock()
    dispatcher = RosMotionClient(node=node, mock_nav2=True)

    class Nav2Result:
        error_code = 100

    mock_wrapped_res = Mock()
    mock_wrapped_res.status = GoalStatus.STATUS_ABORTED
    mock_wrapped_res.result = Nav2Result()

    mock_future = Mock()
    mock_future.result.return_value = mock_wrapped_res

    outcomes = []
    dispatcher._wrap_completed(
        future=mock_future,
        on_completed=lambda res: outcomes.append(res),
    )

    assert len(outcomes) == 1
    result = outcomes[0]
    assert result.success is False
    assert result.message == f"Action failed with status {GoalStatus.STATUS_ABORTED}"


def test_wrap_completed_with_custom_error_msg_and_success_precedence(monkeypatch):
    """Verify actions with custom success attribute and error_msg are respected."""
    from action_msgs.msg import GoalStatus

    monkeypatch.setattr(
        "lekiwi_orchestrator.motion_client.ActionClient",
        lambda *args, **kwargs: Mock(),
    )
    node = Mock()
    dispatcher = RosMotionClient(node=node, mock_nav2=True)

    class CustomManipResult:
        success = False
        error_msg = "Kinematic limit reached"
        execution_time_sec = 0.5

    mock_wrapped_res = Mock()
    mock_wrapped_res.status = GoalStatus.STATUS_SUCCEEDED
    mock_wrapped_res.result = CustomManipResult()

    mock_future = Mock()
    mock_future.result.return_value = mock_wrapped_res

    outcomes = []
    dispatcher._wrap_completed(
        future=mock_future,
        on_completed=lambda res: outcomes.append(res),
    )

    assert len(outcomes) == 1
    result = outcomes[0]
    assert result.success is False
    assert result.message == "Kinematic limit reached"
    assert math.isclose(result.execution_time_sec, 0.5, abs_tol=1e-3)


def test_radial_standoff_guard_two_phase_retreat(ros_context):
    """Verify that when robot is inside standoff circle, radial retreat is executed before viewpoint."""
    class StepTrackingNav(NavigationClient):
        def __init__(self):
            self.dispatched_poses = []
            self.callbacks = []

        def send_navigation_goal(
            self, target_pose, timeout_sec=60.0, on_completed=None
        ) -> bool:
            self.dispatched_poses.append(target_pose)
            self.callbacks.append(on_completed)
            return True

        def cancel_active_goal(self) -> None:
            pass

    node = Node("test_radial_guard_2phase_node")
    dispatcher = StepTrackingNav()
    navigator = ObsNavigator(
        node=node,
        dispatcher=dispatcher,
        map_frame="map",
        board_frame="chessboard_frame",
        standoff_distance=0.65,
        radius_tolerance=0.04,
        angle_offsets_map={
            ObservationIntent.POST_MOVE_VERIFY: [0.0, 0.314, -0.314]
        },
    )

    try:
        # Robot starts inside standoff circle at (0.35, 0.0) relative to board
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
        assert dispatched is True
        # Phase 1: Radial retreat goal dispatched
        assert len(dispatcher.dispatched_poses) == 1
        radial_goal = dispatcher.dispatched_poses[0]
        # Radial retreat should be at (0.65, 0.0)
        assert math.isclose(radial_goal.pose.position.x, 0.65, abs_tol=1e-3)
        assert math.isclose(radial_goal.pose.position.y, 0.0, abs_tol=1e-3)
        assert len(final_results) == 0

        # Simulate Phase 1 completion
        cb1 = dispatcher.callbacks[0]
        cb1(ActionResult(success=True, message="Reached standoff circle"))

        # Phase 2: Azimuth viewpoint goal automatically dispatched
        assert len(dispatcher.dispatched_poses) == 2
        vp_goal = dispatcher.dispatched_poses[1]
        assert math.isclose(vp_goal.pose.position.x, 0.65, abs_tol=1e-3)
        assert math.isclose(vp_goal.pose.position.y, 0.0, abs_tol=1e-3)
        assert len(final_results) == 0

        # Simulate Phase 2 completion
        cb2 = dispatcher.callbacks[1]
        cb2(ActionResult(success=True, message="Reached viewpoint"))

        assert len(final_results) == 1
        assert final_results[0].success is True
        assert navigator.get_viewpoint_index(ObservationIntent.POST_MOVE_VERIFY) == 1
    finally:
        node.destroy_node()


def test_radial_standoff_guard_retreat_failure(ros_context):
    """Verify that when radial retreat fails, Phase 2 is aborted and failure is returned."""
    class StepTrackingNav(NavigationClient):
        def __init__(self):
            self.dispatched_poses = []
            self.callbacks = []

        def send_navigation_goal(
            self, target_pose, timeout_sec=60.0, on_completed=None
        ) -> bool:
            self.dispatched_poses.append(target_pose)
            self.callbacks.append(on_completed)
            return True

        def cancel_active_goal(self) -> None:
            pass

    node = Node("test_radial_guard_fail_node")
    dispatcher = StepTrackingNav()
    navigator = ObsNavigator(
        node=node,
        dispatcher=dispatcher,
        standoff_distance=0.65,
        radius_tolerance=0.04,
    )

    try:
        ref_pose = PoseStamped()
        ref_pose.header.frame_id = "chessboard_frame"
        ref_pose.pose.position.x = 0.20
        ref_pose.pose.position.y = 0.0

        final_results = []
        navigator.reposition_to_next_viewpoint(
            intent=ObservationIntent.POST_MOVE_VERIFY,
            reference_pose=ref_pose,
            on_completed=lambda res: final_results.append(res),
        )
        assert len(dispatcher.dispatched_poses) == 1

        # Simulate radial retreat failure
        dispatcher.callbacks[0](ActionResult(success=False, message="Nav2 path blocked"))

        # No 2nd goal dispatched
        assert len(dispatcher.dispatched_poses) == 1
        assert len(final_results) == 1
        assert final_results[0].success is False
        assert "Nav2 path blocked" in final_results[0].message
        assert navigator.get_viewpoint_index(ObservationIntent.POST_MOVE_VERIFY) == 0
    finally:
        node.destroy_node()


def test_radial_standoff_guard_already_on_circle(ros_context):
    """Verify that when robot is already on standoff circle, Phase 1 is skipped."""
    dispatched_poses = []

    class StepTrackingNav(NavigationClient):
        def send_navigation_goal(
            self, target_pose, timeout_sec=60.0, on_completed=None
        ) -> bool:
            dispatched_poses.append(target_pose)
            if on_completed:
                on_completed(ActionResult(success=True, message="OK"))
            return True

        def cancel_active_goal(self) -> None:
            pass

    node = Node("test_already_on_circle_node")
    navigator = ObsNavigator(
        node=node,
        dispatcher=StepTrackingNav(),
        standoff_distance=0.65,
        radius_tolerance=0.04,
    )

    try:
        ref_pose = PoseStamped()
        ref_pose.header.frame_id = "chessboard_frame"
        ref_pose.pose.position.x = 0.65
        ref_pose.pose.position.y = 0.0

        final_results = []
        dispatched = navigator.reposition_to_next_viewpoint(
            intent=ObservationIntent.POST_MOVE_VERIFY,
            reference_pose=ref_pose,
            on_completed=lambda res: final_results.append(res),
        )
        assert dispatched is True
        # Only 1 goal dispatched directly
        assert len(dispatched_poses) == 1
        assert len(final_results) == 1
        assert final_results[0].success is True
        assert navigator.get_viewpoint_index(ObservationIntent.POST_MOVE_VERIFY) == 1
    finally:
        node.destroy_node()


def test_azimuth_viewpoint_ranking_and_exhaustion(ros_context):
    """Verify that viewpoints are ranked by azimuth distance and exhaustion is tracked."""
    dispatched_poses = []

    class MockNav(NavigationClient):
        def send_navigation_goal(
            self, target_pose, timeout_sec=60.0, on_completed=None
        ) -> bool:
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
        radius_tolerance=0.04,
        angle_offsets_map={ObservationIntent.RELOCALIZE: candidates},
    )

    try:
        assert navigator.get_max_attempts(ObservationIntent.RELOCALIZE) == 3
        assert navigator.has_exhausted_viewpoints(ObservationIntent.RELOCALIZE) is False

        # Current robot position is at azimuth pi/2: (0.0, 1.0)
        ref_pose = PoseStamped()
        ref_pose.header.frame_id = "chessboard_frame"
        ref_pose.pose.position.x = 0.0
        ref_pose.pose.position.y = 1.0

        # Attempt 1: closest should be candidate pi/2 (distance 0)
        navigator.reposition_to_next_viewpoint(
            intent=ObservationIntent.RELOCALIZE,
            reference_pose=ref_pose,
        )
        assert len(dispatched_poses) == 1
        p1 = dispatched_poses[0].pose.position
        assert math.isclose(p1.x, 0.0, abs_tol=1e-2)
        assert math.isclose(p1.y, 1.0, abs_tol=1e-2)

        # Attempt 2: next closest is candidate 0.0 (distance pi/2)
        navigator.reposition_to_next_viewpoint(
            intent=ObservationIntent.RELOCALIZE,
            reference_pose=ref_pose,
        )
        assert len(dispatched_poses) == 2
        p2 = dispatched_poses[1].pose.position
        assert math.isclose(p2.x, 1.0, abs_tol=1e-2)
        assert math.isclose(p2.y, 0.0, abs_tol=1e-2)

        # Attempt 3: farthest candidate -pi/2 (distance pi)
        navigator.reposition_to_next_viewpoint(
            intent=ObservationIntent.RELOCALIZE,
            reference_pose=ref_pose,
        )
        assert len(dispatched_poses) == 3
        assert navigator.has_exhausted_viewpoints(ObservationIntent.RELOCALIZE) is True

        # Attempt 4: cycles modulo len(ranked)
        assert navigator.reposition_to_next_viewpoint(
            intent=ObservationIntent.RELOCALIZE,
            reference_pose=ref_pose,
        ) is True
    finally:
        node.destroy_node()


def test_tf_buffer_lookup_and_fallback(ros_context):
    """Verify ObsNavigator uses tf_buffer if present and falls back gracefully on exception."""
    node = Node("test_tf_buffer_node")
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
        # TF lookup succeeds
        navigator.reposition_to_next_viewpoint(intent=ObservationIntent.POST_MOVE_VERIFY)
        mock_tf_buffer.lookup_transform.assert_called_once()
        mock_tf_buffer.transform.assert_called_once()
        assert mock_dispatcher.send_navigation_goal.called

        # TF lookup raises exception -> fallback to geometric planar transform
        mock_tf_buffer.lookup_transform.side_effect = RuntimeError("TF timeout")
        mock_tf_buffer.transform.side_effect = RuntimeError("TF transform error")

        ref_pose = PoseStamped()
        ref_pose.header.frame_id = "map"
        ref_pose.pose.position.x = 1.0
        ref_pose.pose.position.y = 0.0

        success = navigator.reposition_to_next_viewpoint(
            intent=ObservationIntent.POST_MOVE_VERIFY,
            reference_pose=ref_pose,
            board_x=0.35,
            board_y=0.0,
        )
        assert success is True

        # Check properties and setters
        assert navigator.robot_color == "b"
        navigator.robot_color = "w"
        assert navigator.robot_color == "w"

        # Check empty offsets
        navigator._angle_offsets_map[ObservationIntent.RELOCALIZE] = []
        assert navigator.reposition_to_next_viewpoint(intent=ObservationIntent.RELOCALIZE) is False
        assert navigator.get_max_attempts(ObservationIntent.RELOCALIZE) == 0
    finally:
        node.destroy_node()


def test_fake_motion_client_details(ros_context):
    """Verify FakeMotionClient and _MockGoalHandle methods."""
    node = Node("test_sim_disp_details_node")
    disp = FakeMotionClient(node)

    # Feasibility
    res_list = []
    success = disp.check_feasibility(
        goal=ChessMoveGoal("e2e4", "e2", "e4"),
        on_success=lambda r: res_list.append(r.feasible),
    )
    assert success is True
    assert res_list == [True]

    # Manipulation
    class FakeGoal:
        instruction = "pick pawn"

    manip_res = []
    disp.send_manipulation_goal(
        goal=FakeGoal(),
        on_completed=lambda r: manip_res.append(r.success),
    )
    assert manip_res == [True]

    # Cancel active goal with active handle
    disp._active_handle = _MockGoalHandle()
    disp.cancel_active_goal()
    assert disp._active_handle is None

    # MockGoalHandle
    gh = _MockGoalHandle()
    assert gh.accepted is True
    assert gh.cancelled is False
    gh.cancel_goal_async()
    assert gh.cancelled is True

    node.destroy_node()


def test_obs_navigator_cancellation_and_empty_cases(ros_context):
    """Verify navigator cancel method, empty offsets, and empty rank behavior."""
    node = Node("test_navigator_more_node")
    mock_dispatcher = Mock()
    navigator = ObsNavigator(
        node=node,
        dispatcher=mock_dispatcher,
        standoff_distance=0.65,
    )

    try:
        # Cancel active goal
        navigator.cancel()
        mock_dispatcher.cancel_active_goal.assert_called_once()

        # Empty offsets in _dispatch_azimuth_viewpoint
        navigator._angle_offsets_map[ObservationIntent.RELOCALIZE] = []
        ret = navigator._dispatch_azimuth_viewpoint(
            intent=ObservationIntent.RELOCALIZE,
            curr_bx=0.65,
            curr_by=0.0,
            board_x=0.0,
            board_y=0.0,
            board_yaw=0.0,
            timeout_sec=10.0,
            on_completed=None,
        )
        assert ret is False

        # Reset specific intent index
        navigator._viewpoint_indices[ObservationIntent.RELOCALIZE] = 2
        navigator.reset_viewpoint_index(ObservationIntent.RELOCALIZE)
        assert navigator.get_viewpoint_index(ObservationIntent.RELOCALIZE) == 0
    finally:
        node.destroy_node()


def test_ros_action_dispatcher_feasibility_edge_cases(monkeypatch, ros_context):
    """Verify RosMotionClient handles promotion, castling, rejection, and errors."""
    monkeypatch.setattr(
        "lekiwi_orchestrator.motion_client.ActionClient",
        lambda *args, **kwargs: Mock(),
    )
    node = Node("test_feas_edge_node")
    dispatcher = RosMotionClient(node=node, mock_nav2=True)

    # 1. Feasibility service client unavailable
    dispatcher._feasibility_client = None
    err_msgs = []
    success = dispatcher.check_feasibility(
        goal=ChessMoveGoal("e2e4", "e2", "e4"),
        on_error=lambda msg: err_msgs.append(msg),
    )
    assert success is False
    assert len(err_msgs) == 1

    # 2. Feasibility service not ready
    mock_feas = Mock()
    mock_feas.service_is_ready.return_value = False
    dispatcher._feasibility_client = mock_feas
    err_msgs.clear()
    success = dispatcher.check_feasibility(
        goal=ChessMoveGoal("e2e4", "e2", "e4"),
        on_error=lambda msg: err_msgs.append(msg),
    )
    assert success is False

    # 3. Feasibility promotion and castling goal packing
    mock_feas.service_is_ready.return_value = True
    captured_reqs = []

    def fake_call_async(req):
        captured_reqs.append(req)
        fut = Mock()
        fut.add_done_callback = lambda cb: None  # don't trigger yet
        return fut

    mock_feas.call_async = fake_call_async
    special_goal = ChessMoveGoal(
        uci="e7e8q",
        from_square="e7",
        to_square="e8",
        promotion="q",
        castling_rook_from="h8",
        castling_rook_to="f8",
    )
    dispatcher.check_feasibility(goal=special_goal)
    assert len(captured_reqs) == 1
    assert captured_reqs[0].move.promotion_piece == "q"
    assert captured_reqs[0].move.is_castling is True
    assert captured_reqs[0].move.castling_rook_from == "h8"

    # 4. Feasibility rejected (feasible=False)
    def fake_call_async_rejected(req):
        fut = Mock()
        resp = Mock(feasible=False, message="Path blocked")
        fut.result.return_value = resp
        fut.add_done_callback = lambda cb: cb(fut)
        return fut

    mock_feas.call_async = fake_call_async_rejected
    err_msgs.clear()
    dispatcher.check_feasibility(
        goal=ChessMoveGoal("e2e4", "e2", "e4"),
        on_error=lambda msg: err_msgs.append(msg),
    )
    assert len(err_msgs) == 1
    assert "Path blocked" in err_msgs[0]

    # 5. Feasibility service call raises exception
    def fake_call_async_error(req):
        fut = Mock()
        fut.result.side_effect = RuntimeError("Service crashed")
        fut.add_done_callback = lambda cb: cb(fut)
        return fut

    mock_feas.call_async = fake_call_async_error
    err_msgs.clear()
    dispatcher.check_feasibility(
        goal=ChessMoveGoal("e2e4", "e2", "e4"),
        on_error=lambda msg: err_msgs.append(msg),
    )
    assert len(err_msgs) == 1
    assert "Service crashed" in err_msgs[0]

    dispatcher.destroy()
    node.destroy_node()


def test_ros_motion_client_live_navigation_branches(monkeypatch, ros_context):
    monkeypatch.setattr(
        "lekiwi_orchestrator.motion_client.ActionClient",
        lambda *args, **kwargs: Mock(),
    )
    node = Node("test_live_nav_branches_node")
    client = RosMotionClient(node=node, mock_nav2=False)

    # 1. Server not ready
    mock_nav_action = Mock()
    mock_nav_action.server_is_ready.return_value = False
    client._nav2_client = mock_nav_action
    completed_res = []
    success = client.send_navigation_goal(
        PoseStamped(),
        on_completed=lambda r: completed_res.append(r),
    )
    assert success is False
    assert len(completed_res) == 1
    assert "unavailable or server is not ready" in completed_res[0].message

    # 2. Server ready, goal rejected
    mock_nav_action.server_is_ready.return_value = True
    reject_future = Mock()
    mock_handle = Mock()
    mock_handle.accepted = False
    reject_future.result.return_value = mock_handle
    reject_future.add_done_callback = lambda cb: cb(reject_future)
    mock_nav_action.send_goal_async.return_value = reject_future

    completed_res.clear()
    success = client.send_navigation_goal(
        PoseStamped(),
        on_completed=lambda r: completed_res.append(r),
    )
    assert success is True
    assert len(completed_res) == 1
    assert completed_res[0].success is False
    assert "rejected navigation goal" in completed_res[0].message

    # 3. Server ready, goal accepted and completed
    accept_future = Mock()
    mock_handle_accepted = Mock()
    mock_handle_accepted.accepted = True
    res_future = Mock()
    mock_res_obj = Mock()
    mock_res_obj.result = Mock(error_code=0, execution_time_sec=0.1)
    mock_res_obj.status = 4  # SUCCEEDED
    res_future.result.return_value = mock_res_obj
    res_future.add_done_callback = lambda cb: cb(res_future)
    mock_handle_accepted.get_result_async.return_value = res_future

    accept_future.result.return_value = mock_handle_accepted
    accept_future.add_done_callback = lambda cb: cb(accept_future)
    mock_nav_action.send_goal_async.return_value = accept_future

    completed_res.clear()
    success = client.send_navigation_goal(
        PoseStamped(),
        on_completed=lambda r: completed_res.append(r),
    )
    assert success is True
    assert len(completed_res) == 1
    assert completed_res[0].success is True

    # 4. Exception in goal submission
    error_future = Mock()
    error_future.result.side_effect = RuntimeError("Submission error")
    error_future.add_done_callback = lambda cb: cb(error_future)
    mock_nav_action.send_goal_async.return_value = error_future

    completed_res.clear()
    client.send_navigation_goal(
        PoseStamped(),
        on_completed=lambda r: completed_res.append(r),
    )
    assert len(completed_res) == 1
    assert "Submission error" in completed_res[0].message

    client.destroy()
    node.destroy_node()


def test_ros_motion_client_live_manipulation_branches(monkeypatch, ros_context):
    monkeypatch.setattr(
        "lekiwi_orchestrator.motion_client.ActionClient",
        lambda *args, **kwargs: Mock(),
    )
    node = Node("test_live_manip_branches_node")
    client = RosMotionClient(node=node, mock_nav2=True)

    # 1. Manipulation server not ready
    mock_manip = Mock()
    mock_manip.server_is_ready.return_value = False
    client._manipulation_client = mock_manip
    completed_res = []
    success = client.send_manipulation_goal(
        goal=Mock(),
        on_completed=lambda r: completed_res.append(r),
    )
    assert success is False
    assert len(completed_res) == 1
    assert "unavailable or server is not ready" in completed_res[0].message

    # 2. Manipulation goal rejected
    mock_manip.server_is_ready.return_value = True
    reject_future = Mock()
    mock_handle = Mock()
    mock_handle.accepted = False
    reject_future.result.return_value = mock_handle
    reject_future.add_done_callback = lambda cb: cb(reject_future)
    mock_manip.send_goal_async.return_value = reject_future

    completed_res.clear()
    client.send_manipulation_goal(
        goal=Mock(),
        on_completed=lambda r: completed_res.append(r),
    )
    assert len(completed_res) == 1
    assert completed_res[0].success is False

    # 3. Manipulation goal accepted with feedback and success
    accept_future = Mock()
    mock_handle_accepted = Mock()
    mock_handle_accepted.accepted = True
    res_future = Mock()
    mock_res_obj = Mock()
    mock_res_obj.result = Mock(success=True, message="Piece moved", execution_time_sec=0.5)
    mock_res_obj.status = 4
    res_future.result.return_value = mock_res_obj
    res_future.add_done_callback = lambda cb: cb(res_future)
    mock_handle_accepted.get_result_async.return_value = res_future

    accept_future.result.return_value = mock_handle_accepted
    accept_future.add_done_callback = lambda cb: cb(accept_future)
    mock_manip.send_goal_async.return_value = accept_future

    feedbacks = []
    completed_res.clear()
    client.send_manipulation_goal(
        goal=Mock(),
        on_feedback=lambda fb: feedbacks.append(fb),
        on_completed=lambda r: completed_res.append(r),
    )
    assert len(completed_res) == 1
    assert completed_res[0].success is True
    assert "Piece moved" in completed_res[0].message

    # Test feedback callback wiring
    _, kwargs = mock_manip.send_goal_async.call_args
    assert "feedback_callback" in kwargs
    kwargs["feedback_callback"](Mock(feedback="step 1"))
    assert len(feedbacks) == 1

    # 4. Cancel active goal
    client._active_goal_handle = mock_handle_accepted
    client.cancel_active_goal()
    assert mock_handle_accepted.cancel_goal_async.called

    client.destroy()
    node.destroy_node()







