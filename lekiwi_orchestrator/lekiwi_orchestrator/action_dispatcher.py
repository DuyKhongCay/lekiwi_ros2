# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""
Action Dispatcher Gateway abstractions following Dependency Inversion Principle (DIP).

Decouples high-level mission orchestration logic from low-level ROS 2 action client
mechanics, supporting clean separation between production hardware execution and
in-memory simulated execution (for tabletop test & CI/CD).
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Callable

from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node

try:
    from nav2_msgs.action import NavigateToPose
except ImportError:
    NavigateToPose = None

try:
    from lekiwi_interfaces.action import ExecuteChessMove
except ImportError:
    ExecuteChessMove = None


class ActionDispatcherInterface(ABC):
    """Abstract interface defining the contract for executing motion actions."""

    @abstractmethod
    def send_navigation_goal(
        self,
        target_pose: PoseStamped,
        on_accepted: Callable[[any], None],
        on_completed: Callable[[any], None],
    ) -> bool:
        """Send navigation goal to base."""

    @abstractmethod
    def send_manipulation_goal(
        self,
        goal: any,
        on_feedback: Callable[[any], None],
        on_accepted: Callable[[any], None],
        on_completed: Callable[[any], None],
    ) -> bool:
        """Send manipulation goal to robot arm."""

    @abstractmethod
    def cancel_active_goal(self) -> None:
        """Cancel active navigation or manipulation goal handle."""

    @abstractmethod
    def destroy(self) -> None:
        """Clean up action clients or timers."""


class RosActionDispatcher(ActionDispatcherInterface):
    """Concrete production action dispatcher communicating over ROS 2 Action servers."""

    def __init__(
        self,
        node: Node,
        nav2_action_name: str = "/navigate_to_pose",
        manipulation_action_name: str = "/manipulation/execute_chess_move",
        callback_group: CallbackGroup | None = None,
    ) -> None:
        self._node = node
        self._active_goal_handle = None

        self._nav2_client = (
            ActionClient(
                node,
                NavigateToPose,
                nav2_action_name,
                callback_group=callback_group,
            )
            if NavigateToPose is not None
            else None
        )
        self._manipulation_client = (
            ActionClient(
                node,
                ExecuteChessMove,
                manipulation_action_name,
                callback_group=callback_group,
            )
            if ExecuteChessMove is not None
            else None
        )

    def send_navigation_goal(
        self,
        target_pose: PoseStamped,
        on_accepted: Callable[[any], None],
        on_completed: Callable[[any], None],
    ) -> bool:
        if self._nav2_client is None or not self._nav2_client.server_is_ready():
            self._node.get_logger().error(
                "Nav2 action client is unavailable or server is not ready!"
            )
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose

        send_future = self._nav2_client.send_goal_async(goal_msg)

        def _on_goal_response(fut):
            try:
                handle = fut.result()
                if not handle.accepted:
                    on_accepted(None)
                    return
                self._active_goal_handle = handle
                on_accepted(handle)
                res_future = handle.get_result_async()
                res_future.add_done_callback(
                    lambda rf: self._wrap_completed(rf, on_completed)
                )
            except Exception as exc:  # noqa: BLE001
                self._node.get_logger().error(
                    f"Navigation goal submission error: {exc}"
                )
                on_accepted(None)

        send_future.add_done_callback(_on_goal_response)
        return True

    def send_manipulation_goal(
        self,
        goal: any,
        on_feedback: Callable[[any], None],
        on_accepted: Callable[[any], None],
        on_completed: Callable[[any], None],
    ) -> bool:
        if (
            self._manipulation_client is None
            or not self._manipulation_client.server_is_ready()
        ):
            self._node.get_logger().error(
                "Manipulation action client is unavailable or server is not ready!"
            )
            return False

        send_future = self._manipulation_client.send_goal_async(
            goal, feedback_callback=on_feedback
        )

        def _on_goal_response(fut):
            try:
                handle = fut.result()
                if not handle.accepted:
                    on_accepted(None)
                    return
                self._active_goal_handle = handle
                on_accepted(handle)
                res_future = handle.get_result_async()
                res_future.add_done_callback(
                    lambda rf: self._wrap_completed(rf, on_completed)
                )
            except Exception as exc:  # noqa: BLE001
                self._node.get_logger().error(
                    f"Manipulation goal submission error: {exc}"
                )
                on_accepted(None)

        send_future.add_done_callback(_on_goal_response)
        return True

    def _wrap_completed(self, future, on_completed: Callable[[any], None]) -> None:
        self._active_goal_handle = None
        on_completed(future)

    def cancel_active_goal(self) -> None:
        if self._active_goal_handle is not None:
            try:
                self._active_goal_handle.cancel_goal_async()
            except Exception as exc:  # noqa: BLE001
                self._node.get_logger().warn(f"Failed to cancel active goal: {exc}")
            self._active_goal_handle = None

    def destroy(self) -> None:
        self.cancel_active_goal()
        if self._nav2_client is not None:
            self._nav2_client.destroy()
        if self._manipulation_client is not None:
            self._manipulation_client.destroy()


class _MockGoalHandle:
    """Mock handle returned during simulated execution."""

    def __init__(self):
        self.accepted = True
        self.cancelled = False

    def cancel_goal_async(self):
        self.cancelled = True


class _MockWrappedResult:
    """Mock result wrapped in object matching ROS 2 action result structure."""

    def __init__(self, success: bool = True, message: str = "Simulated success"):
        class _Result:
            def __init__(self, s, m):
                self.success = s
                self.message = m
                self.execution_time_sec = 0.05

        self.result = _Result(success, message)


class _MockCompletedFuture:
    """Future returning simulated action result."""

    def __init__(self, result):
        self._res = result

    def result(self):
        return self._res


class SimulatedActionDispatcher(ActionDispatcherInterface):
    """
    In-memory simulated dispatcher for tabletop simulation and unit tests.
    Does not require live ROS 2 action servers, preventing production code pollution.
    """

    def __init__(self, node: Node) -> None:
        self._node = node
        self._active_handle = None

    def send_navigation_goal(
        self,
        target_pose: PoseStamped,
        on_accepted: Callable[[any], None],
        on_completed: Callable[[any], None],
    ) -> bool:
        self._node.get_logger().info(
            f"[SIMULATED NAV2] Navigating to ({target_pose.pose.position.x:.2f}, "
            f"{target_pose.pose.position.y:.2f})"
        )
        handle = _MockGoalHandle()
        self._active_handle = handle
        on_accepted(handle)
        mock_future = _MockCompletedFuture(
            _MockWrappedResult(True, "Simulated navigation OK")
        )
        self._active_handle = None
        on_completed(mock_future)
        return True

    def send_manipulation_goal(
        self,
        goal: any,
        on_feedback: Callable[[any], None],
        on_accepted: Callable[[any], None],
        on_completed: Callable[[any], None],
    ) -> bool:
        instruction = getattr(goal, "instruction", "move piece")
        self._node.get_logger().info(
            f"[SIMULATED MANIPULATION] Executing: {instruction}"
        )
        handle = _MockGoalHandle()
        self._active_handle = handle
        on_accepted(handle)
        mock_future = _MockCompletedFuture(
            _MockWrappedResult(True, f"Simulated manipulation '{instruction}' OK")
        )
        self._active_handle = None
        on_completed(mock_future)
        return True

    def cancel_active_goal(self) -> None:
        if self._active_handle is not None:
            self._active_handle.cancel_goal_async()
            self._active_handle = None

    def destroy(self) -> None:
        self.cancel_active_goal()
