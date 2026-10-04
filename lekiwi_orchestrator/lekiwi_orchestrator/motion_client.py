# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""
Motion execution clients and feasibility dispatchers.

Provides:
- Abstract gateway interfaces (IFeasibilityChecker, INavigationDispatcher, IManipulationDispatcher, ActionDispatcherInterface)
- Production ROS 2 ActionDispatcher (RosActionDispatcher) with integrated action watchdogs
- Simulated in-memory dispatcher for headless tests (SimulatedActionDispatcher)
"""

from __future__ import annotations

import threading
from abc import ABC, abstractmethod
from collections.abc import Callable
from typing import TYPE_CHECKING, Any

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from lekiwi_interfaces.srv import CheckMoveFeasibility
from rclpy.action import ActionClient
from rclpy.callback_groups import CallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.node import Node

from lekiwi_orchestrator.mission_types import ActionResult, ChessMoveGoal

try:
    from nav2_msgs.action import NavigateToPose
except ImportError:
    NavigateToPose = None  # type: ignore[assignment, misc]

try:
    from lekiwi_interfaces.action import ExecuteChessMove
except ImportError:
    ExecuteChessMove = None  # type: ignore[assignment, misc]


# ==============================================================================
# Abstract Motion Interfaces (ISP / DIP)
# ==============================================================================


class FeasibilityClient(ABC):
    """Interface for querying workspace reachability and motion feasibility."""

    @abstractmethod
    def check_feasibility(
        self,
        goal: ChessMoveGoal,
        timeout_sec: float = 5.0,
        on_success: Callable[[CheckMoveFeasibility.Response], None] | None = None,
        on_error: Callable[[str], None] | None = None,
    ) -> bool:
        """Query workspace feasibility checker service with integrated watchdog."""


class NavigationClient(ABC):
    """Interface for dispatching mobile base navigation goals."""

    @abstractmethod
    def send_navigation_goal(
        self,
        target_pose: PoseStamped,
        timeout_sec: float = 60.0,
        on_completed: Callable[[ActionResult], None] | None = None,
    ) -> bool:
        """Send navigation goal to base with integrated watchdog."""

    @abstractmethod
    def cancel_active_goal(self) -> None:
        """Cancel active navigation goal."""


class ManipulationClient(ABC):
    """Interface for dispatching robot arm manipulation goals."""

    @abstractmethod
    def send_manipulation_goal(
        self,
        goal: Any,
        timeout_sec: float = 60.0,
        on_feedback: Callable[[Any], None] | None = None,
        on_completed: Callable[[ActionResult], None] | None = None,
    ) -> bool:
        """Send manipulation goal to robot arm with integrated watchdog."""

    @abstractmethod
    def cancel_active_goal(self) -> None:
        """Cancel active manipulation goal."""


class MotionClient(
    FeasibilityClient, NavigationClient, ManipulationClient, ABC
):
    """Composite interface defining full motion execution and feasibility capabilities."""

    @abstractmethod
    def destroy(self) -> None:
        """Clean up action clients, service clients, and timers."""




# ==============================================================================
# Production RosMotionClient
# ==============================================================================


class RosMotionClient(MotionClient):
    """
    Concrete production action dispatcher communicating over ROS 2 Action and Service servers.

    Supports mock_nav2 for tabletop setups where base navigation is bypassed
    while manipulation is delegated directly to ROS 2 (lekiwi_manipulation).
    """

    def __init__(
        self,
        node: Node,
        nav2_action_name: str = "/navigate_to_pose",
        manipulation_action_name: str = "/manipulation/execute_chess_move",
        check_feasibility_service_name: str = "/workspace/check_move_feasibility",
        callback_group: CallbackGroup | None = None,
        mock_nav2: bool = False,
    ) -> None:
        self._node = node
        self._callback_group = callback_group
        self._mock_nav2 = mock_nav2
        self._active_goal_handle: Any = None
        self._action_timer: Any = None
        self._feasibility_timer: Any = None
        self._action_watchdog_desc: str = ""

        self._feasibility_client = node.create_client(
            CheckMoveFeasibility,
            check_feasibility_service_name,
            callback_group=callback_group,
        )

        self._nav2_client = (
            ActionClient(
                node,
                NavigateToPose,
                nav2_action_name,
                callback_group=callback_group,
            )
            if (NavigateToPose is not None and not mock_nav2)
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

    def _start_action_watchdog(
        self,
        desc: str,
        timeout_sec: float,
        on_timeout: Callable[[], None] | None = None,
    ) -> None:
        self._cancel_action_watchdog()
        self._action_watchdog_desc = desc

        def _on_watchdog_timeout() -> None:
            self._node.get_logger().error(
                f"{desc} timed out after {timeout_sec:.1f}s! Cancelling active goal."
            )
            self.cancel_active_goal()
            self._cancel_action_watchdog()
            if on_timeout:
                on_timeout()

        self._action_timer = self._node.create_timer(
            timeout_sec,
            _on_watchdog_timeout,
            callback_group=self._callback_group,
        )

    def _cancel_action_watchdog(self) -> None:
        if self._action_timer is not None:
            self._action_timer.cancel()
            self._node.destroy_timer(self._action_timer)
            self._action_timer = None
        self._action_watchdog_desc = ""

    def _cancel_feasibility_timer(self) -> None:
        if self._feasibility_timer is not None:
            self._feasibility_timer.cancel()
            self._node.destroy_timer(self._feasibility_timer)
            self._feasibility_timer = None

    def check_feasibility(
        self,
        goal: ChessMoveGoal,
        timeout_sec: float = 5.0,
        on_success: Callable[[CheckMoveFeasibility.Response], None] | None = None,
        on_error: Callable[[str], None] | None = None,
    ) -> bool:
        if (
            self._feasibility_client is None
            or not self._feasibility_client.service_is_ready()
        ):
            self._node.get_logger().error(
                "Feasibility service client is unavailable or server is not ready!"
            )
            if on_error:
                on_error(
                    "Feasibility service client is unavailable or server is not ready!"
                )
            return False

        req = CheckMoveFeasibility.Request()
        req.move.uci = goal.uci
        req.move.from_square = goal.from_square
        req.move.to_square = goal.to_square
        req.move.is_capture = goal.is_capture
        req.move.captured_square = goal.captured_square
        if goal.promotion:
            req.move.promotion_piece = goal.promotion
        if goal.castling_rook_from:
            req.move.castling_rook_from = goal.castling_rook_from
            req.move.castling_rook_to = goal.castling_rook_to
            req.move.is_castling = True

        self._cancel_feasibility_timer()
        future = self._feasibility_client.call_async(req)

        handled = False
        lock = threading.Lock()

        def _on_timeout() -> None:
            nonlocal handled
            with lock:
                if handled:
                    return
                handled = True
            self._cancel_feasibility_timer()
            msg = f"Feasibility query for move {goal.uci} timed out after {timeout_sec:.1f}s"
            self._node.get_logger().error(msg)
            if on_error:
                on_error(msg)

        self._feasibility_timer = self._node.create_timer(
            timeout_sec,
            _on_timeout,
            callback_group=self._callback_group,
        )

        def _on_response(fut: Any) -> None:
            nonlocal handled
            with lock:
                if handled:
                    return
                handled = True
            self._cancel_feasibility_timer()
            try:
                resp = fut.result()
                if resp.feasible:
                    if on_success:
                        on_success(resp)
                else:
                    msg = resp.message or "Move declared not feasible"
                    self._node.get_logger().error(
                        f"Move {goal.uci} declared NOT FEASIBLE: {msg}"
                    )
                    if on_error:
                        on_error(msg)
            except Exception as exc:  # noqa: BLE001
                self._node.get_logger().error(f"Feasibility query failed: {exc}")
                if on_error:
                    on_error(str(exc))

        future.add_done_callback(_on_response)
        return True

    def send_navigation_goal(
        self,
        target_pose: PoseStamped,
        timeout_sec: float = 60.0,
        on_completed: Callable[[ActionResult], None] | None = None,
    ) -> bool:
        """Dispatch target_pose to Nav2 with integrated watchdog."""
        if self._mock_nav2:
            self._node.get_logger().info(
                f"[MOCK NAV2] Base repositioning bypassed to "
                f"({target_pose.pose.position.x:.2f}, {target_pose.pose.position.y:.2f})"
            )
            if on_completed:
                on_completed(
                    ActionResult(
                        success=True,
                        message="Mock navigation completed (tabletop mode)",
                        execution_time_sec=0.05,
                    )
                )
            return True

        if self._nav2_client is None or not self._nav2_client.server_is_ready():
            self._node.get_logger().error(
                "Nav2 action client is unavailable or server is not ready!"
            )
            if on_completed:
                on_completed(
                    ActionResult(
                        False,
                        "Nav2 action client is unavailable or server is not ready!",
                    )
                )
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose

        self._start_action_watchdog(
            "Nav2 navigation",
            timeout_sec=timeout_sec,
            on_timeout=lambda: (
                on_completed(ActionResult(False, "Nav2 navigation timed out"))
                if on_completed
                else None
            ),
        )

        send_future = self._nav2_client.send_goal_async(goal_msg)

        def _on_goal_response(fut: Any) -> None:
            try:
                handle = fut.result()
                if not handle.accepted:
                    self._cancel_action_watchdog()
                    self._node.get_logger().error("Nav2 rejected navigation goal!")
                    if on_completed:
                        on_completed(
                            ActionResult(False, "Nav2 rejected navigation goal")
                        )
                    return
                self._active_goal_handle = handle
                self._node.get_logger().info("Nav2 navigation goal accepted.")
                res_future = handle.get_result_async()
                res_future.add_done_callback(
                    lambda rf: self._wrap_completed(rf, on_completed)
                )
            except Exception as exc:  # noqa: BLE001
                self._cancel_action_watchdog()
                self._node.get_logger().error(
                    f"Navigation goal submission error: {exc}"
                )
                if on_completed:
                    on_completed(ActionResult(False, str(exc)))

        send_future.add_done_callback(_on_goal_response)
        return True

    def send_manipulation_goal(
        self,
        goal: Any,
        timeout_sec: float = 60.0,
        on_feedback: Callable[[Any], None] | None = None,
        on_completed: Callable[[ActionResult], None] | None = None,
    ) -> bool:
        """Dispatch manipulation goal to Arm Manipulation action server with integrated watchdog."""
        if (
            self._manipulation_client is None
            or not self._manipulation_client.server_is_ready()
        ):
            self._node.get_logger().error(
                "Manipulation action client is unavailable or server is not ready!"
            )
            if on_completed:
                on_completed(
                    ActionResult(
                        False,
                        "Manipulation action client is unavailable or server is not ready!",
                    )
                )
            return False

        instruction = getattr(goal, "instruction", "manipulation goal")
        self._start_action_watchdog(
            f"Manipulation goal: '{instruction}'",
            timeout_sec=timeout_sec,
            on_timeout=lambda: (
                on_completed(
                    ActionResult(False, f"Manipulation goal '{instruction}' timed out")
                )
                if on_completed
                else None
            ),
        )

        send_future = self._manipulation_client.send_goal_async(
            goal, feedback_callback=on_feedback
        )

        def _on_goal_response(fut: Any) -> None:
            try:
                handle = fut.result()
                if not handle.accepted:
                    self._cancel_action_watchdog()
                    self._node.get_logger().error(
                        "Manipulation server rejected chess move goal!"
                    )
                    if on_completed:
                        on_completed(
                            ActionResult(False, "Manipulation server rejected goal")
                        )
                    return
                self._active_goal_handle = handle
                self._node.get_logger().info("Manipulation goal accepted.")
                res_future = handle.get_result_async()
                res_future.add_done_callback(
                    lambda rf: self._wrap_completed(rf, on_completed)
                )
            except Exception as exc:  # noqa: BLE001
                self._cancel_action_watchdog()
                self._node.get_logger().error(
                    f"Manipulation goal submission error: {exc}"
                )
                if on_completed:
                    on_completed(ActionResult(False, str(exc)))

        send_future.add_done_callback(_on_goal_response)
        return True

    @staticmethod
    def _parse_action_success(result: Any, status: int | None) -> bool:
        """Evaluate action success combining ActionServer GoalStatus and result.success."""
        has_status = isinstance(status, int)
        is_goal_succeeded = (
            status == GoalStatus.STATUS_SUCCEEDED if has_status else False
        )

        if hasattr(result, "success"):
            return bool(result.success) and (not has_status or is_goal_succeeded)
        return is_goal_succeeded

    @staticmethod
    def _parse_action_message(result: Any, status: int | None, success: bool) -> str:
        """Resolve user-facing outcome message from action result."""
        if hasattr(result, "message") and result.message:
            return str(result.message)
        if hasattr(result, "error_msg") and result.error_msg:
            return str(result.error_msg)
        return "Action succeeded" if success else f"Action failed with status {status}"

    def _wrap_completed(
        self, future: Any, on_completed: Callable[[ActionResult], None] | None
    ) -> None:
        self._cancel_action_watchdog()
        self._active_goal_handle = None
        if on_completed is None:
            return
        try:
            res_obj = future.result() if hasattr(future, "result") else future
            result = getattr(res_obj, "result", res_obj)
            status = getattr(res_obj, "status", None)

            success = self._parse_action_success(result, status)
            message = self._parse_action_message(result, status, success)
            exec_time = float(getattr(result, "execution_time_sec", 0.0))
            on_completed(
                ActionResult(
                    success=success,
                    message=message,
                    execution_time_sec=exec_time,
                )
            )
        except Exception as exc:  # noqa: BLE001
            on_completed(ActionResult(success=False, message=str(exc)))

    def cancel_active_goal(self) -> None:
        """Cancel active Nav2 or Manipulation goal asynchronously."""
        self._cancel_action_watchdog()
        self._cancel_feasibility_timer()
        if self._active_goal_handle is not None:
            try:
                self._node.get_logger().warn(
                    "Cancelling active ROS 2 action goal handle..."
                )
                self._active_goal_handle.cancel_goal_async()
            except Exception as e:
                self._node.get_logger().warn(
                    f"Exception cancelling active goal handle: {e}"
                )
            finally:
                self._active_goal_handle = None

    def destroy(self) -> None:
        """Clean up action clients, timers, and active goal handles."""
        self.cancel_active_goal()
        if (
            hasattr(self, "_feasibility_client")
            and self._feasibility_client is not None
        ):
            self._node.destroy_client(self._feasibility_client)
            self._feasibility_client = None
        if hasattr(self, "_nav2_client") and self._nav2_client is not None:
            self._nav2_client.destroy()
            self._nav2_client = None
        if (
            hasattr(self, "_manipulation_client")
            and self._manipulation_client is not None
        ):
            self._manipulation_client.destroy()
            self._manipulation_client = None





# ==============================================================================
# In-Memory Fake Motion Client (for tests & tabletop mock)
# ==============================================================================


class _MockGoalHandle:
    """Mock handle returned during simulated execution."""

    def __init__(self) -> None:
        self.accepted = True
        self.cancelled = False

    def cancel_goal_async(self) -> None:
        self.cancelled = True


class FakeMotionClient(MotionClient):
    """
    In-memory simulated motion client for tabletop simulation and unit tests.
    Does not require live ROS 2 action servers, preventing production code pollution.
    """

    def __init__(self, node: Node | None = None) -> None:
        self._node = node
        self._active_handle: _MockGoalHandle | None = None

    def check_feasibility(
        self,
        goal: ChessMoveGoal,
        timeout_sec: float = 5.0,
        on_success: Callable[[CheckMoveFeasibility.Response], None] | None = None,
        on_error: Callable[[str], None] | None = None,
    ) -> bool:
        resp = CheckMoveFeasibility.Response()
        resp.feasible = True
        resp.plan_type = CheckMoveFeasibility.Response.PLAN_ZERO_NAV
        resp.message = "Simulated feasibility confirmed"
        if on_success:
            on_success(resp)
        return True

    def send_navigation_goal(
        self,
        target_pose: PoseStamped,
        timeout_sec: float = 60.0,
        on_completed: Callable[[ActionResult], None] | None = None,
    ) -> bool:
        if self._node is not None:
            self._node.get_logger().info(
                f"[SIMULATED NAV2] Navigating to ({target_pose.pose.position.x:.2f}, "
                f"{target_pose.pose.position.y:.2f})"
            )
        if on_completed:
            on_completed(
                ActionResult(
                    success=True,
                    message="Simulated navigation OK",
                    execution_time_sec=0.05,
                )
            )
        return True

    def send_manipulation_goal(
        self,
        goal: Any,
        timeout_sec: float = 60.0,
        on_feedback: Callable[[Any], None] | None = None,
        on_completed: Callable[[ActionResult], None] | None = None,
    ) -> bool:
        instruction = getattr(goal, "instruction", "move piece")
        if self._node is not None:
            self._node.get_logger().info(
                f"[SIMULATED MANIPULATION] Executing: {instruction}"
            )
        if on_completed:
            on_completed(
                ActionResult(
                    success=True,
                    message=f"Simulated manipulation '{instruction}' OK",
                    execution_time_sec=0.05,
                )
            )
        return True

    def cancel_active_goal(self) -> None:
        if self._active_handle is not None:
            self._active_handle.cancel_goal_async()
            self._active_handle = None

    def destroy(self) -> None:
        self.cancel_active_goal()


__all__ = [
    "FeasibilityClient",
    "NavigationClient",
    "ManipulationClient",
    "MotionClient",
    "RosMotionClient",
    "FakeMotionClient",
]
