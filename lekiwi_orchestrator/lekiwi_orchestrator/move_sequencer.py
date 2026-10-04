# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""
Move sequencer choreography executing multi-stage chess trajectories.

Coordinates mobile base navigation docking and arm manipulation stages
driving Level 2 MotionExecutionState transitions and active relocalization recovery.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from typing import TYPE_CHECKING, Any

from geometry_msgs.msg import PoseStamped
from lekiwi_interfaces.msg import PerceptionContext
from lekiwi_interfaces.srv import CheckMoveFeasibility
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node

from lekiwi_orchestrator.fsm import (
    MOTION_STATE_NAMES,
    MotionExecutionState,
    is_motion_transition_allowed,
)
from lekiwi_orchestrator.mission_types import ActionResult, ObservationIntent
from lekiwi_orchestrator.move_planner import MoveStep, StepKind

if TYPE_CHECKING:
    from lekiwi_orchestrator.motion_client import MotionClient
    from lekiwi_orchestrator.obs_navigator import ObsNavigator
    from lekiwi_orchestrator.perception_context import PerceptionContextManager


class MoveSequencer:
    """
    Sequences a multi-stage chess move pipeline across navigation and manipulation actuators.
    Drives Level 2 MotionExecutionState transitions with integrated active self-healing.
    """

    def __init__(
        self,
        node: Node,
        dispatcher: MotionClient,
        perception: PerceptionContextManager,
        board_frame: str = "chessboard_frame",
        action_timeout_sec: float = 60.0,
        on_pipeline_completed: Callable[[], None] | None = None,
        on_pipeline_failed: Callable[[str], None] | None = None,
        on_motion_state_changed: Callable[[MotionExecutionState], None] | None = None,
        grasp_readiness_provider: Callable[[], bool] | None = None,
        pre_grasp_settle_sec: float = 2.0,
        navigation_enabled: bool = True,
        callback_group: CallbackGroup | None = None,
        observation_navigator: ObsNavigator | None = None,
    ) -> None:
        self._node = node
        self._dispatcher = dispatcher
        self._perception = perception
        self._board_frame = board_frame
        self._action_timeout_sec = action_timeout_sec

        self._on_pipeline_completed = on_pipeline_completed
        self._on_pipeline_failed = on_pipeline_failed
        self._on_motion_state_changed = on_motion_state_changed
        self._callback_group = callback_group
        self._obs_nav = observation_navigator

        self._lock = threading.RLock()
        self._cancel_event = threading.Event()
        self._motion_state = MotionExecutionState.IDLE
        self._stages: list[MoveStep] = []
        self._current_stage: MoveStep | None = None
        self._feasibility_resp: CheckMoveFeasibility.Response | None = None
        self._last_nav_pose: PoseStamped | None = None

        # Self-healing & grasp readiness parameters
        self._navigation_enabled = navigation_enabled
        self._pre_grasp_settle_sec = max(0.0, float(pre_grasp_settle_sec))
        self._grasp_readiness_provider = grasp_readiness_provider or (lambda: True)
        self._relocalize_attempts = 0
        self._pending_stage: MoveStep | None = None

    @property
    def motion_state(self) -> MotionExecutionState:
        with self._lock:
            return self._motion_state

    @property
    def current_stage(self) -> MoveStep | None:
        with self._lock:
            return self._current_stage

    @property
    def is_active(self) -> bool:
        with self._lock:
            return bool(self._stages or self._current_stage is not None)

    @property
    def pre_grasp_settle_sec(self) -> float:
        return self._pre_grasp_settle_sec

    def transition_motion_to(self, target_state: MotionExecutionState) -> bool:
        """Safely transition Level 2 MotionExecutionState validating legal transition matrix."""
        with self._lock:
            if target_state == self._motion_state:
                return True
            if not is_motion_transition_allowed(self._motion_state, target_state):
                old_name = MOTION_STATE_NAMES.get(self._motion_state, "UNKNOWN")
                new_name = MOTION_STATE_NAMES.get(target_state, "UNKNOWN")
                self._node.get_logger().error(
                    f"ILLEGAL MOTION FSM TRANSITION: Cannot move from {old_name} to {new_name}"
                )
                return False
            old_name = MOTION_STATE_NAMES.get(self._motion_state, "UNKNOWN")
            new_name = MOTION_STATE_NAMES.get(target_state, "UNKNOWN")
            self._motion_state = target_state
            self._node.get_logger().info(
                f"[MOTION FSM] Transitioned: {old_name} -> {new_name}"
            )

        if self._on_motion_state_changed:
            self._on_motion_state_changed(target_state)
        return True

    def start_pipeline(
        self,
        stages: list[MoveStep],
        feasibility_resp: CheckMoveFeasibility.Response,
    ) -> None:
        """Begin execution of a newly built list of MoveSteps."""
        self._cancel_event.clear()
        with self._lock:
            self._stages = list(stages)
            self._feasibility_resp = feasibility_resp
            self._current_stage = None
            self._last_nav_pose = None
            self._reset_recovery_state()

        self._node.get_logger().info(
            f"[MOVE SEQUENCER] Starting execution of {len(stages)} steps..."
        )
        self._advance_stage()

    def _advance_stage(self) -> None:
        """Advance to the next MoveStep or complete pipeline."""
        if self._cancel_event.is_set():
            return

        completed_cb = None
        stage: MoveStep | None = None
        with self._lock:
            if not self._stages:
                self._current_stage = None
                self._feasibility_resp = None
                self.transition_motion_to(MotionExecutionState.IDLE)
                completed_cb = self._on_pipeline_completed
            else:
                stage = self._stages.pop(0)
                self._current_stage = stage

        if stage is None:
            if completed_cb is not None:
                completed_cb()
            return

        if stage.target_pose is not None:
            self._execute_nav_stage(stage)
        else:
            if not self._wait_for_grasp_readiness(self._pre_grasp_settle_sec):
                if not self._cancel_event.is_set():
                    self._handle_grasp_unready(stage)
                return
            if not self._cancel_event.is_set():
                self._execute_manip_stage(stage)

    def _get_now_sec(self) -> float:
        """Return current timestamp in seconds using node clock, fallback to monotonic."""
        try:
            if hasattr(self._node, "get_clock"):
                clk = self._node.get_clock()
                if clk is not None:
                    now = clk.now()
                    if hasattr(now, "nanoseconds") and isinstance(now.nanoseconds, (int, float)):
                        return float(now.nanoseconds) * 1e-9
                    if hasattr(now, "seconds_nanoseconds"):
                        sec, nsec = now.seconds_nanoseconds()
                        return float(sec) + float(nsec) * 1e-9
        except Exception:
            pass
        return time.monotonic()

    # ==========================================================================
    # Grasp Readiness & Self-Healing (Non-blocking watchdog protection)
    # ==========================================================================

    def _reset_recovery_state(self) -> None:
        with self._lock:
            self._relocalize_attempts = 0
            self._pending_stage = None
            if self._obs_nav is not None:
                self._obs_nav.reset_viewpoint_index(ObservationIntent.RELOCALIZE)

    def _wait_for_grasp_readiness(self, timeout_sec: float) -> bool:
        """
        Wait for grasp readiness provider until True, timeout_sec expires, or cancel requested.

        Protected with both ROS clock and monotonic wall-clock watchdog to
        prevent deadlocks or infinite loops if simulation time (use_sim_time) is paused.
        """
        if self._cancel_event.is_set():
            return False
        if self._grasp_readiness_provider():
            return True
        if timeout_sec <= 0.0:
            return False

        start_ros = self._get_now_sec()
        start_wall = time.monotonic()
        poll_interval = min(0.02, timeout_sec / 4) if timeout_sec > 0 else 0.005

        while True:
            if self._cancel_event.wait(timeout=poll_interval):
                return False
            if self._grasp_readiness_provider():
                return True

            elapsed_ros = self._get_now_sec() - start_ros
            elapsed_wall = time.monotonic() - start_wall
            if elapsed_ros >= timeout_sec or elapsed_wall >= timeout_sec:
                break

        return (not self._cancel_event.is_set()) and self._grasp_readiness_provider()

    def _handle_grasp_unready(self, stage: MoveStep) -> None:
        """Handle unready grasp state by triggering active relocalization self-healing."""
        if self._cancel_event.is_set():
            return

        if self._obs_nav is None or not self._navigation_enabled:
            self._handle_failure(
                f"Grasp readiness check failed before stage '{stage.name}' "
                f"(settle timeout {self._pre_grasp_settle_sec:.1f}s expired, active relocalization unavailable)"
            )
            return

        with self._lock:
            max_attempts = self._obs_nav.get_max_attempts(ObservationIntent.RELOCALIZE)
            if self._relocalize_attempts >= max_attempts:
                self._handle_failure(
                    f"Grasp readiness failed before stage '{stage.name}' "
                    f"and exhausted all {max_attempts} relocalization viewpoints"
                )
                return

            self._relocalize_attempts += 1
            self._pending_stage = stage
            current_attempts = self._relocalize_attempts

        ref_pose = stage.target_pose or self._last_nav_pose

        self._node.get_logger().warn(
            f"[SELF-HEALING] Grasp readiness false before stage '{stage.name}'. "
            f"Initiating active relocalization (attempt {current_attempts}/{max_attempts})..."
        )
        self.transition_motion_to(MotionExecutionState.NAV_TO_OBS)
        self._perception.set_context(PerceptionContext.TF_TRACKING_AND_NAV)

        dispatched = self._obs_nav.reposition_to_next_viewpoint(
            intent=ObservationIntent.RELOCALIZE,
            reference_pose=ref_pose,
            timeout_sec=self._action_timeout_sec,
            on_completed=self._on_relocalize_nav_completed,
        )
        if not dispatched:
            self._handle_failure(
                f"Failed to dispatch Nav2 goal for relocalization before stage '{stage.name}'"
            )

    def _on_relocalize_nav_completed(self, result: ActionResult) -> None:
        """Called when Nav2 finishes navigating to relocalization observation viewpoint."""
        if self._cancel_event.is_set():
            return

        with self._lock:
            pending = self._pending_stage
            if pending is None:
                return

        if not result.success:
            self._node.get_logger().warn(
                f"[SELF-HEALING] Relocalization Nav2 goal failed: {result.message}. Retrying next viewpoint..."
            )
            self._handle_grasp_unready(pending)
            return

        self._node.get_logger().info(
            f"[SELF-HEALING] Reached relocalization viewpoint for stage '{pending.name}'. "
            f"Waiting for EKF convergence / grasp readiness ({self._pre_grasp_settle_sec:.1f}s)..."
        )
        if self._wait_for_grasp_readiness(self._pre_grasp_settle_sec):
            if self._cancel_event.is_set():
                return
            self._node.get_logger().info(
                f"[SELF-HEALING] Relocalization succeeded! Resuming pending stage '{pending.name}'."
            )
            with self._lock:
                self._relocalize_attempts = 0
                if self._obs_nav is not None:
                    self._obs_nav.reset_viewpoint_index(ObservationIntent.RELOCALIZE)
                stage_to_resume = self._pending_stage
                self._pending_stage = None

            if stage_to_resume is None:
                return

            return_pose = stage_to_resume.target_pose or self._last_nav_pose

            if (
                stage_to_resume.target_pose is None
                and return_pose is not None
                and self._obs_nav is not None
            ):
                self._node.get_logger().info(
                    f"[SELF-HEALING] Navigating base back to manipulation stance "
                    f"({return_pose.pose.position.x:.3f}, {return_pose.pose.position.y:.3f})..."
                )
                self.transition_motion_to(
                    stage_to_resume.nav_motion_state or MotionExecutionState.NAV_TO_PICK
                )
                self._perception.set_context(PerceptionContext.TF_TRACKING_AND_NAV)
                dispatched = self._dispatcher.send_navigation_goal(
                    target_pose=return_pose,
                    timeout_sec=self._action_timeout_sec,
                    on_completed=lambda res: (
                        self._resume_stage(stage_to_resume)
                        if res.success and not self._cancel_event.is_set()
                        else self._handle_failure(
                            f"Failed to return to manipulation pose: {res.message}"
                        )
                    ),
                )
                if not dispatched:
                    self._handle_failure(
                        f"Failed to dispatch return navigation for '{stage_to_resume.name}'"
                    )
                return

            self._resume_stage(stage_to_resume)
        else:
            if not self._cancel_event.is_set():
                self._node.get_logger().warn(
                    "[SELF-HEALING] Grasp readiness still false at current viewpoint. Retrying next viewpoint..."
                )
                self._handle_grasp_unready(pending)

    def _resume_stage(self, stage: MoveStep) -> None:
        if stage.target_pose is not None:
            self._execute_nav_stage(stage)
        else:
            self._execute_manip_stage(stage)

    # ==========================================================================
    # Action Stages Execution
    # ==========================================================================

    def _execute_nav_stage(self, stage: MoveStep) -> None:
        nav_state = stage.nav_motion_state or MotionExecutionState.NAV_TO_PICK
        self.transition_motion_to(nav_state)
        self._perception.set_context(PerceptionContext.TF_TRACKING_AND_NAV)
        self._last_nav_pose = stage.target_pose

        self._node.get_logger().info(
            f"Dispatching NavigateToPose ({stage.name} standoff) to "
            f"({stage.target_pose.pose.position.x:.3f}, {stage.target_pose.pose.position.y:.3f})..."
        )

        dispatched = self._dispatcher.send_navigation_goal(
            target_pose=stage.target_pose,
            timeout_sec=self._action_timeout_sec,
            on_completed=lambda res: self._on_nav_completed(res, stage),
        )
        if not dispatched:
            self._handle_failure(
                f"Failed to dispatch Nav2 goal for stage '{stage.name}'"
            )

    def _on_nav_completed(self, result: ActionResult, stage: MoveStep) -> None:
        if self._cancel_event.is_set():
            return

        if not result.success:
            self._handle_failure(f"Nav2 navigation failed: {result.message}")
            return

        if stage.name in (StepKind.OBSERVATION, "OBSERVATION"):
            self._node.get_logger().info(
                "Reached primary observation standoff. Physical move pipeline complete."
            )
            self._advance_stage()
            return

        settle_sec = self._pre_grasp_settle_sec
        self._node.get_logger().info(
            f"Nav2 navigation for stage '{stage.name}' completed successfully! "
            f"Checking grasp readiness (settle window: {settle_sec:.1f}s)..."
        )
        if not self._wait_for_grasp_readiness(settle_sec):
            if not self._cancel_event.is_set():
                self._handle_grasp_unready(stage)
            return

        if not self._cancel_event.is_set():
            self._execute_manip_stage(stage)

    def _execute_manip_stage(self, stage: MoveStep) -> None:
        if self._cancel_event.is_set():
            return

        self.transition_motion_to(stage.motion_state)
        self._perception.set_context(PerceptionContext.MANIPULATION_ACTOR)

        with self._lock:
            f_resp = self._feasibility_resp
        goal = stage.to_ros_goal(self._board_frame, f_resp)

        self._node.get_logger().info(
            f"Sending ExecuteChessMove goal: '{goal.instruction}' (capture={goal.is_capture})..."
        )

        dispatched = self._dispatcher.send_manipulation_goal(
            goal=goal,
            timeout_sec=self._action_timeout_sec,
            on_feedback=self._on_manip_feedback,
            on_completed=lambda res: self._on_manip_completed(res, stage),
        )
        if not dispatched:
            self._handle_failure(
                f"Failed to dispatch manipulation goal for stage '{stage.name}'"
            )

    def _on_manip_feedback(self, feedback_msg: Any) -> None:
        fb = getattr(feedback_msg, "feedback", feedback_msg)
        phase = getattr(fb, "current_phase", "EXEC")
        progress = getattr(fb, "progress_percent", 0.0)
        self._node.get_logger().info(
            f"[MANIPULATION FEEDBACK] Phase: {phase} ({progress:.1f}%)"
        )

    def _on_manip_completed(self, result: ActionResult, stage: MoveStep) -> None:
        if not result.success:
            self._handle_failure(f"Manipulation execution failed: {result.message}")
            return

        self._node.get_logger().info(
            f"Manipulation execution successful in {result.execution_time_sec:.2f}s: {result.message}"
        )

        if stage.name in (StepKind.PLACE, StepKind.MOVE, "PLACE", "MOVE"):
            if self._motion_state == MotionExecutionState.PICKING_PIECE:
                self.transition_motion_to(MotionExecutionState.PLACING_PIECE)
        self._advance_stage()

    def _handle_failure(self, error_msg: str) -> None:
        self._node.get_logger().error(error_msg)
        with self._lock:
            self._stages.clear()
            self._current_stage = None
            self._feasibility_resp = None
            self.transition_motion_to(MotionExecutionState.IDLE)
        if self._on_pipeline_failed:
            self._on_pipeline_failed(error_msg)

    def cancel(self) -> None:
        """Cancel active physical actions and reset executor state."""
        self._cancel_event.set()
        self._dispatcher.cancel_active_goal()
        with self._lock:
            self._stages.clear()
            self._current_stage = None
            self._feasibility_resp = None
            self._last_nav_pose = None
            self._reset_recovery_state()
            self.transition_motion_to(MotionExecutionState.IDLE)

    def destroy(self) -> None:
        """Clean up active timers and cancel any ongoing goals."""
        self.cancel()


__all__ = [
    "MoveSequencer",
]
