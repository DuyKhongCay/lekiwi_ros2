# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""
Mock Physical-AI Policy Server for LeKiwi Chess Manipulation.

Implements the /manipulation/execute_chess_move Action Server:
1. Receives chess move goals (instruction, pick & place coordinates, IK hints).
2. Generates smooth quintic polynomial joint trajectories across discrete phases:
   APPROACH_PICK -> DESCEND_PICK -> GRASP -> LIFT -> TRANSIT_PLACE -> DESCEND_PLACE -> RELEASE -> RETRACT_STOW
3. Emits real-time progress feedback and supports graceful preemption/cancel.
4. Operates in hardware mode (sending FollowJointTrajectory goals) or standalone simulation mode.
"""

from __future__ import annotations

import time
from typing import Dict, Optional

from enum import Enum
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from lekiwi_interfaces.action import ExecuteChessMove
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARM_JOINTS_DEFAULT: tuple[str, ...] = (
    "arm_shoulder_pan",
    "arm_shoulder_lift",
    "arm_elbow_flex",
    "arm_wrist_flex",
    "arm_wrist_roll",
    "arm_gripper",
)

DEFAULT_HOME_POSE: Dict[str, float] = {
    "arm_shoulder_pan": 0.0,
    "arm_shoulder_lift": -0.8,
    "arm_elbow_flex": -1.2,
    "arm_wrist_flex": 0.5,
    "arm_wrist_roll": 0.0,
    "arm_gripper": 0.0,
}

DEFAULT_STOW_POSE: Dict[str, float] = {
    "arm_shoulder_pan": 0.0,
    "arm_shoulder_lift": -1.5,
    "arm_elbow_flex": -1.5,
    "arm_wrist_flex": 0.0,
    "arm_wrist_roll": 0.0,
    "arm_gripper": 0.0,
}


class ManipulationPhase(str, Enum):
    """Discrete operational phases of a chess pick-and-place command."""

    IDLE = "IDLE"
    APPROACH_PICK = "APPROACH_PICK"
    DESCEND_PICK = "DESCEND_PICK"
    GRASP = "GRASP"
    LIFT = "LIFT"
    TRANSIT_PLACE = "TRANSIT_PLACE"
    DESCEND_PLACE = "DESCEND_PLACE"
    RELEASE = "RELEASE"
    RETRACT_STOW = "RETRACT_STOW"


DEFAULT_CHESS_PHASES: tuple[ManipulationPhase, ...] = (
    ManipulationPhase.APPROACH_PICK,
    ManipulationPhase.DESCEND_PICK,
    ManipulationPhase.GRASP,
    ManipulationPhase.LIFT,
    ManipulationPhase.TRANSIT_PLACE,
    ManipulationPhase.DESCEND_PLACE,
    ManipulationPhase.RELEASE,
    ManipulationPhase.RETRACT_STOW,
)

DEFAULT_PHASE_DURATION_SEC = 0.6
DEFAULT_ACTION_NAME = "/manipulation/execute_chess_move"


class MockPolicyServer(Node):
    """
    Simulates and executes manipulation tasks for closed-loop chess testing.
    """

    def __init__(
        self,
        node_name: str = "mock_policy_server",
        action_name: str = DEFAULT_ACTION_NAME,
    ) -> None:
        super().__init__(node_name)

        self.declare_parameter("phase_duration_sec", DEFAULT_PHASE_DURATION_SEC)
        self.declare_parameter("simulate_delay", True)
        self.declare_parameter("action_name", action_name)

        self._phase_duration = float(self.get_parameter("phase_duration_sec").value)
        self._simulate_delay = bool(self.get_parameter("simulate_delay").value)
        self._action_name = str(self.get_parameter("action_name").value)

        self._current_phase = ManipulationPhase.IDLE
        self._is_active = False

        # Callback Group
        self._cbg_server = ReentrantCallbackGroup()

        # Action Server
        self._action_server = ActionServer(
            self,
            ExecuteChessMove,
            self._action_name,
            execute_callback=self._execute_goal,
            goal_callback=self._handle_goal_request,
            cancel_callback=self._handle_cancel_request,
            callback_group=self._cbg_server,
        )

        # Diagnostics
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._diag_timer = self.create_timer(1.0, self._publish_diagnostics)

        self.get_logger().info(
            f"MockPolicyServer initialized on action '{self._action_name}' "
            f"(phase_duration={self._phase_duration:.2f}s, simulate_delay={self._simulate_delay})."
        )

    @property
    def current_phase(self) -> ManipulationPhase:
        return self._current_phase

    # ================= Action Callbacks =================

    def _handle_goal_request(self, goal_request: ExecuteChessMove.Goal) -> GoalResponse:
        """Accept or reject incoming goal requests."""
        if self._is_active:
            self.get_logger().warn("Rejecting new goal: server is currently busy.")
            return GoalResponse.REJECT

        self.get_logger().info(
            f"Accepted ExecuteChessMove goal: '{goal_request.instruction}' "
            f"({goal_request.from_square} -> {goal_request.to_square}, capture={goal_request.is_capture})"
        )
        return GoalResponse.ACCEPT

    def _handle_cancel_request(self, goal_handle) -> CancelResponse:
        """Acknowledge preemption / cancellation requests."""
        self.get_logger().warn(
            f"Received cancel request for goal: '{goal_handle.request.instruction}'. Preempting..."
        )
        return CancelResponse.ACCEPT

    def _execute_goal(self, goal_handle) -> ExecuteChessMove.Result:
        """Execute the multi-phase chess move sequence in pure simulation."""
        self._is_active = True
        start_time = time.monotonic()
        req: ExecuteChessMove.Goal = goal_handle.request
        result = ExecuteChessMove.Result()

        self.get_logger().info(
            f"Executing manipulation sequence for: {req.instruction}"
        )

        total_phases = len(DEFAULT_CHESS_PHASES)

        # Pure Software Mock: Step through phase timeline emitting feedback and checking for preemption
        for idx, phase in enumerate(DEFAULT_CHESS_PHASES):
            if goal_handle.is_cancel_requested:
                self.get_logger().warn(f"Goal cancelled during phase {phase.value}!")
                self._current_phase = ManipulationPhase.IDLE
                self._is_active = False
                goal_handle.canceled()
                result.success = False
                result.message = f"Preempted and cancelled during {phase.value}."
                result.execution_time_sec = float(time.monotonic() - start_time)
                return result

            self._current_phase = phase
            progress = float((idx + 1) / total_phases) * 100.0

            feedback = ExecuteChessMove.Feedback()
            feedback.current_phase = phase.value
            feedback.progress_percent = progress
            goal_handle.publish_feedback(feedback)

            self.get_logger().info(f"[PHASE] {phase.value} ({progress:.1f}%)")

            if self._simulate_delay:
                # Sleep in short increments to allow rapid preemption/cancel response
                if not self._sleep_with_cancel_check(goal_handle, self._phase_duration):
                    self.get_logger().warn(
                        f"Goal cancelled during delay in phase {phase.value}!"
                    )
                    self._current_phase = ManipulationPhase.IDLE
                    self._is_active = False
                    goal_handle.canceled()
                    result.success = False
                    result.message = f"Preempted and cancelled during {phase.value}."
                    result.execution_time_sec = float(time.monotonic() - start_time)
                    return result

        # 4. Conclude execution successfully
        elapsed = time.monotonic() - start_time
        self._current_phase = ManipulationPhase.IDLE
        self._is_active = False

        goal_handle.succeed()
        result.success = True
        result.message = f"Chess move '{req.instruction}' executed successfully."
        result.execution_time_sec = float(elapsed)

        self.get_logger().info(
            f"ExecuteChessMove completed in {elapsed:.2f}s: {result.message}"
        )
        return result

    def _sleep_with_cancel_check(self, goal_handle, duration_sec: float) -> bool:
        """Sleep incrementally while polling for cancellation. Returns False if cancelled."""
        step = 0.05
        elapsed = 0.0
        while elapsed < duration_sec:
            if goal_handle.is_cancel_requested:
                return False
            chunk = min(step, duration_sec - elapsed)
            time.sleep(chunk)
            elapsed += chunk
        return True

    def _extract_joint_dict(self, js: JointState) -> Dict[str, float]:
        """Convert a JointState message into a dictionary of joint positions."""
        if not js.name or not js.position:
            return dict(DEFAULT_HOME_POSE)
        return dict(zip(js.name, js.position))

    # ================= Diagnostics =================

    def _publish_diagnostics(self) -> None:
        """Emit periodic health and operational phase telemetry."""
        diag = DiagnosticStatus()
        diag.name = "Mock Policy Server"
        diag.hardware_id = "LeKiwi_Manipulation"
        diag.level = DiagnosticStatus.OK
        diag.message = f"Active: {self._is_active} (Phase: {self._current_phase.value})"

        diag.values = [
            KeyValue(key="is_active", value=str(self._is_active)),
            KeyValue(key="current_phase", value=self._current_phase.value),
            KeyValue(key="phase_duration_sec", value=f"{self._phase_duration:.2f}"),
            KeyValue(key="simulate_delay", value=str(self._simulate_delay)),
        ]

        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        diag_array.status.append(diag)
        self._diag_pub.publish(diag_array)


def main(args=None):
    rclpy.init(args=args)
    node = MockPolicyServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
