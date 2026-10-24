# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Stockfish chess engine action client for autonomous move computation.

Encapsulates ROS 2 action client communications for lekiwi_interfaces/action/ComputeBestMove.
"""

from __future__ import annotations

import threading
from collections.abc import Callable
from typing import TYPE_CHECKING, Any

from rclpy.action import ActionClient

if TYPE_CHECKING:
    from rclpy.callback_groups import CallbackGroup
    from rclpy.node import Node

try:
    from lekiwi_interfaces.action import ComputeBestMove
except ImportError:
    ComputeBestMove = None  # type: ignore[assignment, misc]


class EngineClient:
    """Encapsulates asynchronous Stockfish ComputeBestMove action interactions."""

    def __init__(
        self,
        node: Node,
        action_name: str = "/chess/compute_best_move",
        callback_group: CallbackGroup | None = None,
    ) -> None:
        """Initialize EngineClient with ROS 2 ActionClient endpoint.

        Args:
            node: Parent ROS 2 node handle.
            action_name: ROS 2 action server name.
            callback_group: Callback group for non-blocking asynchronous execution.
        """
        self._node = node
        self._action_name = action_name
        self._lock = threading.Lock()
        self._is_busy = False
        self._active_goal_handle: Any = None

        if ComputeBestMove is not None:
            self._client = ActionClient(
                node,
                ComputeBestMove,
                action_name,
                callback_group=callback_group,
            )
        else:
            self._client = None

    @property
    def is_busy(self) -> bool:
        """Return True if currently computing a move."""
        with self._lock:
            return self._is_busy

    def is_available(self) -> bool:
        """Return True if Action Server is ready."""
        if self._client is None:
            return False
        return self._client.server_is_ready()

    def cancel_active_search(self) -> None:
        """Cancel active engine computation if currently running."""
        with self._lock:
            if not self._is_busy or self._active_goal_handle is None:
                return
            handle = self._active_goal_handle
            self._is_busy = False
            self._active_goal_handle = None

        self._node.get_logger().info(
            f"[ENGINE CLIENT] Canceling active Stockfish goal on '{self._action_name}'"
        )
        try:
            handle.cancel_goal_async()
        except Exception as ex:
            self._node.get_logger().warn(
                f"[ENGINE CLIENT] Failed to cancel active goal: {ex}"
            )

    def compute_best_move_async(
        self,
        fen: str,
        think_time_ms: int,
        on_complete: Callable[[str | None, str], None],
    ) -> bool:
        """Send asynchronous goal to Stockfish action server.

        Args:
            fen: Canonical FEN board state string.
            think_time_ms: Evaluation time budget in milliseconds.
            on_complete: Callback invoked upon completion: (best_move_uci or None, message).

        Returns:
            True if goal submission was initiated, False otherwise.
        """
        if self._client is None:
            on_complete(None, "ComputeBestMove action definition not available")
            return False

        if not self._client.server_is_ready():
            msg = f"Stockfish action server '{self._action_name}' is not ready"
            self._node.get_logger().warn(f"[ENGINE CLIENT] {msg}")
            on_complete(None, msg)
            return False

        with self._lock:
            if self._is_busy:
                msg = "EngineClient is already evaluating another position"
                self._node.get_logger().warn(f"[ENGINE CLIENT] {msg}")
                on_complete(None, msg)
                return False
            self._is_busy = True

        goal_msg = ComputeBestMove.Goal()
        goal_msg.fen = fen
        goal_msg.think_time_ms = max(100, int(think_time_ms))
        goal_msg.depth = 0

        self._node.get_logger().info(
            f"[ENGINE CLIENT] Requesting best move for FEN: '{fen}' (Think: {goal_msg.think_time_ms} ms)"
        )

        send_future = self._client.send_goal_async(goal_msg)

        def _on_goal_response(fut: Any) -> None:
            try:
                goal_handle = fut.result()
            except Exception as ex:
                with self._lock:
                    self._is_busy = False
                err = f"Failed to send goal to engine: {ex}"
                self._node.get_logger().error(f"[ENGINE CLIENT] {err}")
                on_complete(None, err)
                return

            if not goal_handle.accepted:
                with self._lock:
                    self._is_busy = False
                err = "Engine action server rejected goal"
                self._node.get_logger().warn(f"[ENGINE CLIENT] {err}")
                on_complete(None, err)
                return

            with self._lock:
                self._active_goal_handle = goal_handle

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(_on_result)

        def _on_result(fut: Any) -> None:
            with self._lock:
                self._is_busy = False
                self._active_goal_handle = None

            try:
                wrapped_res = fut.result()
                res = wrapped_res.result
            except Exception as ex:
                err = f"Exception receiving engine result: {ex}"
                self._node.get_logger().error(f"[ENGINE CLIENT] {err}")
                on_complete(None, err)
                return

            if not res or not res.success or not res.best_move:
                msg = res.message if res else "Empty response from engine"
                self._node.get_logger().warn(f"[ENGINE CLIENT] Evaluation failed: {msg}")
                on_complete(None, msg)
                return

            self._node.get_logger().info(
                f"[ENGINE CLIENT] Received Best Move: '{res.best_move}' (Score: {res.eval_centipawns} cp, Mate: {res.is_mate})"
            )
            on_complete(res.best_move, "OK")

        send_future.add_done_callback(_on_goal_response)
        return True

    def destroy(self) -> None:
        """Clean up active action client resources upon shutdown."""
        self.cancel_active_search()
        if self._client is not None:
            self._client.destroy()
            self._client = None

