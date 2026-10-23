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

from geometry_msgs.msg import Point, PoseStamped
from lekiwi_interfaces.action import ExecuteChessMove
from lekiwi_interfaces.msg import PerceptionContext
from lekiwi_interfaces.srv import CheckMoveFeasibility
import rclpy
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node

from lekiwi_orchestrator.core.fsm import (
    MOTION_STATE_NAMES,
    MotionExecutionState,
    is_motion_transition_allowed,
)
from lekiwi_orchestrator.core.mission_types import (
    ActionResult,
    ArmLandmarks,
    ObservationIntent,
)
from lekiwi_orchestrator.core.utils import get_node_now_sec
from lekiwi_orchestrator.planning.move_planner import MoveStep, StepKind

if TYPE_CHECKING:
    import tf2_ros
    from lekiwi_orchestrator.execution.motion_client import MotionClient
    from lekiwi_orchestrator.planning.obs_navigator import ObsNavigator
    from lekiwi_orchestrator.supervision.perception_context import PerceptionContextManager


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
        chassis_settle_timeout_sec: float = 2.0,
        chassis_settle_provider: Callable[[], bool] | None = None,
        navigation_enabled: bool = True,
        callback_group: CallbackGroup | None = None,
        observation_navigator: ObsNavigator | None = None,
        tf_buffer: tf2_ros.Buffer | None = None,
        landmarks: ArmLandmarks | None = None,
        bypass_pause_sec: float = 3.0,
    ) -> None:
        self._node = node
        self._dispatcher = dispatcher
        self._perception = perception
        self._board_frame = board_frame
        self._action_timeout_sec = action_timeout_sec
        self._landmarks = landmarks if landmarks is not None else ArmLandmarks()

        self._on_pipeline_completed = on_pipeline_completed
        self._on_pipeline_failed = on_pipeline_failed
        self._on_motion_state_changed = on_motion_state_changed
        self._callback_group = callback_group
        self._obs_nav = observation_navigator
        self._tf_buffer = tf_buffer

        self._lock = threading.RLock()
        self._cancel_event = threading.Event()
        self._motion_state = MotionExecutionState.IDLE
        self._stages: list[MoveStep] = []
        self._current_stage: MoveStep | None = None
        self._last_nav_pose: PoseStamped | None = None

        # Self-healing & grasp readiness parameters
        self._navigation_enabled = navigation_enabled
        self._chassis_settle_timeout_sec = max(0.0, float(chassis_settle_timeout_sec))
        self._bypass_pause_sec = max(0.0, float(bypass_pause_sec))
        self._grasp_readiness_provider = grasp_readiness_provider or (lambda: True)
        self._chassis_settle_provider = chassis_settle_provider or (lambda: True)
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
    def chassis_settle_timeout_sec(self) -> float:
        return self._chassis_settle_timeout_sec

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

    def start_pipeline(self, stages: list[MoveStep]) -> None:
        """Begin execution of a newly built sequential move pipeline.

        Resets cancellation flags and recovery counters before triggering the first atomic stage.

        Args:
            stages: Ordered list of MoveStep primitives (approach, clear, pick, place, verify).
        """
        self._cancel_event.clear()
        with self._lock:
            self._stages = list(stages)
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

        with self._lock:
            if not self._stages:
                self._current_stage = None
                self.transition_motion_to(MotionExecutionState.IDLE)
                completed_cb = self._on_pipeline_completed
                stage = None
            else:
                stage = self._stages.pop(0)
                self._current_stage = stage
                completed_cb = None

        if stage is None:
            if completed_cb is not None:
                completed_cb()
            return

        self._execute_stage(stage)

    def _gate_pre_grasp(self, stage: MoveStep) -> bool:
        if not stage.requires_pre_grasp_gate:
            self._node.get_logger().info(
                f"Stage '{stage.name}' is an in-place continuation. "
                "Bypassing pre-grasp readiness check."
            )
            return True

        if self._wait_for_grasp_readiness(self._chassis_settle_timeout_sec):
            return True

        if not self._cancel_event.is_set():
            self._handle_grasp_unready(stage)
        return False

    def _execute_stage(self, stage: MoveStep) -> None:
        if stage.bypass_record:
            self._node.get_logger().warn(
                f"⏭ [BYPASS QUOTA FULL] Step '{stage.name}' ({stage.instruction}) is bypassed! "
                f"Operator: please perform this move manually on the board. "
                f"Waiting {self._bypass_pause_sec:.1f}s before advancing..."
            )
            if not self._cancel_event.wait(timeout=self._bypass_pause_sec):
                self._advance_stage()
            return

        if stage.target_pose is not None:
            self._execute_nav_stage(stage)
            return

        if self._gate_pre_grasp(stage) and not self._cancel_event.is_set():
            self._execute_manip_stage(stage)

    # ==========================================================================
    # Grasp Readiness & Self-Healing (Non-blocking watchdog protection)
    # ==========================================================================

    def _reset_recovery_state(self) -> None:
        with self._lock:
            self._pending_stage = None
            if self._obs_nav is not None:
                self._obs_nav.reset_tour(ObservationIntent.RELOCALIZE)

    def _wait_for_condition(
        self, predicate: Callable[[], bool], timeout_sec: float
    ) -> bool:
        if self._cancel_event.is_set() or timeout_sec <= 0.0:
            return False
        if predicate():
            return True

        start_ros = get_node_now_sec(self._node)
        start_wall = time.monotonic()
        poll_interval = min(0.02, timeout_sec / 4.0)

        while not self._cancel_event.wait(timeout=poll_interval):
            if predicate():
                return True

            if (get_node_now_sec(self._node) - start_ros >= timeout_sec) or (
                time.monotonic() - start_wall >= timeout_sec
            ):
                break

        return (not self._cancel_event.is_set()) and predicate()

    def _wait_for_grasp_readiness(self, timeout_sec: float) -> bool:
        """Wait for grasp readiness provider until True, timeout_sec expires, or cancel requested."""
        return self._wait_for_condition(self._grasp_readiness_provider, timeout_sec)

    def _wait_for_chassis_settle(self, timeout_sec: float) -> bool:
        """Wait until chassis has fully settled (velocity within stop thresholds) or timeout expires."""
        return self._wait_for_condition(self._chassis_settle_provider, timeout_sec)

    def _handle_grasp_unready(self, stage: MoveStep) -> None:
        """Handle unready grasp state by triggering active relocalization self-healing."""
        if self._cancel_event.is_set():
            return

        if self._obs_nav is None or not self._navigation_enabled:
            self._handle_failure(
                f"Grasp readiness check failed before stage '{stage.name}' "
                f"(settle timeout {self._chassis_settle_timeout_sec:.1f}s expired, active relocalization unavailable)"
            )
            return

        if self._obs_nav.has_exhausted_viewpoints(ObservationIntent.RELOCALIZE):
            max_attempts = self._obs_nav.get_max_attempts(ObservationIntent.RELOCALIZE)
            self._handle_failure(
                f"Grasp readiness failed before stage '{stage.name}' "
                f"and exhausted all {max_attempts} relocalization viewpoints"
            )
            return

        with self._lock:
            self._pending_stage = stage

        ref_pose = stage.target_pose or self._last_nav_pose
        current_idx = self._obs_nav.get_viewpoint_index(ObservationIntent.RELOCALIZE)
        max_attempts = self._obs_nav.get_max_attempts(ObservationIntent.RELOCALIZE)

        self._node.get_logger().warn(
            f"[SELF-HEALING] Grasp readiness false before stage '{stage.name}'. "
            f"Initiating active relocalization (attempt {current_idx + 1}/{max_attempts})..."
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

    def _dispatch_return_to_manip_pose(
        self, stage: MoveStep, return_pose: PoseStamped
    ) -> None:
        self._node.get_logger().info(
            f"[SELF-HEALING] Navigating base back to manipulation stance "
            f"({return_pose.pose.position.x:.3f}, {return_pose.pose.position.y:.3f})..."
        )
        self.transition_motion_to(
            stage.nav_motion_state or MotionExecutionState.NAV_TO_PICK
        )
        self._perception.set_context(PerceptionContext.TF_TRACKING_AND_NAV)
        dispatched = self._dispatcher.send_navigation_goal(
            target_pose=return_pose,
            timeout_sec=self._action_timeout_sec,
            on_completed=lambda res: (
                self._resume_stage(stage)
                if res.success and not self._cancel_event.is_set()
                else self._handle_failure(
                    f"Failed to return to manipulation pose: {res.message}"
                )
            ),
        )
        if not dispatched:
            self._handle_failure(
                f"Failed to dispatch return navigation for '{stage.name}'"
            )

    def _resume_after_relocalization(self, stage: MoveStep) -> None:
        with self._lock:
            if self._obs_nav is not None:
                self._obs_nav.reset_tour(ObservationIntent.RELOCALIZE)
            self._pending_stage = None

        return_pose = stage.target_pose or self._last_nav_pose
        if (
            stage.target_pose is None
            and return_pose is not None
            and self._obs_nav is not None
        ):
            self._dispatch_return_to_manip_pose(stage, return_pose)
            return

        self._resume_stage(stage)

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
            f"Waiting for EKF convergence / grasp readiness ({self._chassis_settle_timeout_sec:.1f}s)..."
        )
        if not self._wait_for_grasp_readiness(self._chassis_settle_timeout_sec):
            if not self._cancel_event.is_set():
                self._node.get_logger().warn(
                    "[SELF-HEALING] Grasp readiness still false at current viewpoint. Retrying next viewpoint..."
                )
                self._handle_grasp_unready(pending)
            return

        if not self._cancel_event.is_set():
            self._node.get_logger().info(
                f"[SELF-HEALING] Relocalization succeeded! Resuming pending stage '{pending.name}'."
            )
            self._resume_after_relocalization(pending)

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

        if stage.name == StepKind.OBSERVATION:
            self._node.get_logger().info(
                f"Nav2 completed for observation standoff. Waiting for chassis to settle ({self._chassis_settle_timeout_sec:.1f}s max)..."
            )
            self._wait_for_chassis_settle(self._chassis_settle_timeout_sec)
            self._node.get_logger().info(
                "Reached primary observation standoff and chassis settled. Physical move pipeline complete."
            )
            self._advance_stage()
            return

        settle_sec = self._chassis_settle_timeout_sec
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

    def _transform_point_to_base(
        self, point: Point | None, target_frame: str
    ) -> list[float]:
        """Transforms a geometry_msgs/Point to base_footprint coordinate frame.

        Fail-Fast: Raises RuntimeError if tf_buffer is missing or transform lookup fails.
        """
        if point is None:
            return [0.0, 0.0, 0.0]

        if self._tf_buffer is None:
            raise RuntimeError(
                f"MoveSequencer cannot transform point from '{target_frame}' to 'base_footprint': "
                "tf_buffer was not provided."
            )

        try:
            tf_stamped = self._tf_buffer.lookup_transform(
                "base_footprint",
                target_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except Exception as exc:
            self._node.get_logger().error(
                f"MoveSequencer: Failed to lookup transform from '{target_frame}' to 'base_footprint': {exc}"
            )
            raise RuntimeError(
                f"Cannot build manipulation goal: TF transform from '{target_frame}' to 'base_footprint' unavailable: {exc}"
            ) from exc

        tx = tf_stamped.transform.translation.x
        ty = tf_stamped.transform.translation.y
        tz = tf_stamped.transform.translation.z
        qx = tf_stamped.transform.rotation.x
        qy = tf_stamped.transform.rotation.y
        qz = tf_stamped.transform.rotation.z
        qw = tf_stamped.transform.rotation.w

        r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
        r01 = 2.0 * (qx * qy - qz * qw)
        r02 = 2.0 * (qx * qz + qy * qw)
        r10 = 2.0 * (qx * qy + qz * qw)
        r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
        r12 = 2.0 * (qy * qz - qx * qw)
        r20 = 2.0 * (qx * qz - qy * qw)
        r21 = 2.0 * (qy * qz + qx * qw)
        r22 = 1.0 - 2.0 * (qx * qx + qy * qy)

        x = r00 * point.x + r01 * point.y + r02 * point.z + tx
        y = r10 * point.x + r11 * point.y + r12 * point.z + ty
        z = r20 * point.x + r21 * point.y + r22 * point.z + tz
        return [float(x), float(y), float(z)]

    def _build_manipulation_goal(self, stage: MoveStep) -> ExecuteChessMove.Goal:
        """Build ExecuteChessMove.Goal message with 10D environment_state from a MoveStep domain object."""
        goal = ExecuteChessMove.Goal()
        goal.target_frame = self._board_frame
        goal.pick_point = stage.pick_point
        goal.place_point = stage.place_point
        goal.instruction = stage.instruction

        if stage.name == StepKind.MOVE:
            goal.operation = ExecuteChessMove.Goal.OP_MOVE
        elif stage.name == StepKind.CLEAR:
            if stage.clear_bin_side.lower() == "left":
                goal.operation = ExecuteChessMove.Goal.OP_CLEAR_LEFT
            else:
                goal.operation = ExecuteChessMove.Goal.OP_CLEAR_RIGHT
        elif stage.name == StepKind.PICK:
            goal.operation = ExecuteChessMove.Goal.OP_PICK
        elif stage.name == StepKind.PLACE:
            goal.operation = ExecuteChessMove.Goal.OP_PLACE
        else:
            raise ValueError(f"Unsupported stage for manipulation: {stage.name}")

        # Compute 6D spatial conditioning: [src(3), tgt(3)] in base_footprint
        p_pick = self._transform_point_to_base(stage.pick_point, self._board_frame)
        p_place = self._transform_point_to_base(stage.place_point, self._board_frame)

        if goal.operation == ExecuteChessMove.Goal.OP_MOVE:
            src, tgt = p_pick, p_place
        elif goal.operation == ExecuteChessMove.Goal.OP_CLEAR_LEFT:
            src, tgt = p_pick, list(self._landmarks.clear_left)
        elif goal.operation == ExecuteChessMove.Goal.OP_CLEAR_RIGHT:
            src, tgt = p_pick, list(self._landmarks.clear_right)
        elif goal.operation == ExecuteChessMove.Goal.OP_PICK:
            src, tgt = p_pick, list(self._landmarks.stow)
        elif goal.operation == ExecuteChessMove.Goal.OP_PLACE:
            src, tgt = list(self._landmarks.stow), p_place
        else:
            src, tgt = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]

        goal.environment_state = [
            float(src[0]),
            float(src[1]),
            float(src[2]),
            float(tgt[0]),
            float(tgt[1]),
            float(tgt[2]),
        ]

        return goal

    def _execute_manip_stage(self, stage: MoveStep) -> None:
        if self._cancel_event.is_set():
            return

        self.transition_motion_to(stage.motion_state)
        self._perception.set_context(PerceptionContext.MANIPULATION_ACTOR)

        try:
            goal = self._build_manipulation_goal(stage)
        except Exception as exc:
            self._handle_failure(
                f"Failed to build manipulation goal for stage '{stage.name}': {exc}"
            )
            return

        self._node.get_logger().info(
            f"Sending ExecuteChessMove goal: operation={goal.operation} (stage='{stage.name}', '{stage.instruction}')..."
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

        if stage.name in (StepKind.PLACE, StepKind.MOVE):
            if self._motion_state == MotionExecutionState.PICKING_PIECE:
                self.transition_motion_to(MotionExecutionState.PLACING_PIECE)
        self._advance_stage()

    def _handle_failure(self, error_msg: str) -> None:
        self._node.get_logger().error(error_msg)
        with self._lock:
            self._stages.clear()
            self._current_stage = None
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
            self._last_nav_pose = None
            self._reset_recovery_state()
            self.transition_motion_to(MotionExecutionState.IDLE)

    def destroy(self) -> None:
        """Clean up active timers and cancel any ongoing goals."""
        self.cancel()


__all__ = [
    "MoveSequencer",
]
