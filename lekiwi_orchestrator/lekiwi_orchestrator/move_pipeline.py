# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""
Move Execution Pipeline Domain for LeKiwi Autonomous Chess Robot.

Encapsulates:
1. Move data models (ChessMoveGoal, ExecutionStage).
2. Stage pipeline builder (StagePipelineBuilder): pure domain mapping feasibility to stages.
3. Move pipeline executor (MovePipelineExecutor): stateful choreography engine executing
   Nav2 standoff docking, arm manipulation, and post-move verification through Level 2 FSM.
"""

from __future__ import annotations

import math
import threading
from collections.abc import Callable
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, ClassVar

from geometry_msgs.msg import Point, PoseStamped

from lekiwi_interfaces.msg import ChessMoveDetails, PerceptionContext
from lekiwi_interfaces.srv import CheckMoveFeasibility
from lekiwi_orchestrator.fsm import (
    MOTION_STATE_NAMES,
    MotionExecutionState,
    is_motion_transition_allowed,
)

if TYPE_CHECKING:
    from rclpy.node import Node

    from lekiwi_orchestrator.motion_dispatcher import (
        ActionDispatcherInterface,
        ActionResult,
    )
    from lekiwi_orchestrator.perception_manager import PerceptionContextCoordinator

try:
    from lekiwi_interfaces.action import ExecuteChessMove
except ImportError:
    ExecuteChessMove = None


@dataclass(frozen=True)
class ChessMoveGoal:
    """Lightweight mission-level move description."""

    uci: str
    from_square: str
    to_square: str
    promotion: str | None = None
    is_capture: bool = False
    captured_square: str = ""
    castling_rook_from: str = ""
    castling_rook_to: str = ""

    @classmethod
    def from_uci_or_details(
        cls,
        uci_move: str,
        move_details: ChessMoveDetails | None = None,
    ) -> ChessMoveGoal | None:
        """Parse ChessMoveGoal from UCI string or rich ChessMoveDetails message."""
        cleaned_move = uci_move.strip().lower()
        if len(cleaned_move) < 4:
            return None

        if move_details is not None and getattr(move_details, "from_square", ""):
            return cls(
                uci=getattr(move_details, "uci", "") or cleaned_move,
                from_square=move_details.from_square,
                to_square=move_details.to_square,
                promotion=(
                    move_details.promotion_piece
                    if getattr(move_details, "promotion_piece", None)
                    else None
                ),
                is_capture=bool(getattr(move_details, "is_capture", False)),
                captured_square=(
                    move_details.captured_square
                    if getattr(move_details, "captured_square", None)
                    else (
                        move_details.to_square
                        if getattr(move_details, "is_capture", False)
                        else ""
                    )
                ),
                castling_rook_from=getattr(move_details, "castling_rook_from", ""),
                castling_rook_to=getattr(move_details, "castling_rook_to", ""),
            )

        from_sq = cleaned_move[:2]
        to_sq = cleaned_move[2:4]
        promo = cleaned_move[4:] if len(cleaned_move) > 4 else None
        return cls(
            uci=cleaned_move,
            from_square=from_sq,
            to_square=to_sq,
            promotion=promo,
            is_capture=False,
        )


@dataclass
class ExecutionStage:
    """Represents a single atomic operation in a multi-stage chess move."""

    name: str  # "CLEAR", "PICK", "PLACE", "MOVE"
    target_pose: PoseStamped | None
    instruction: str
    from_square: str
    to_square: str
    is_capture: bool
    pick_point: Point
    place_point: Point
    motion_state: MotionExecutionState = MotionExecutionState.IDLE
    nav_motion_state: MotionExecutionState | None = None

    def to_ros_goal(
        self,
        board_frame: str,
        feasibility_resp: CheckMoveFeasibility.Response | None = None,
    ) -> Any:
        """Assemble ExecuteChessMove.Goal message with points and IK hints."""
        if ExecuteChessMove is None:
            raise RuntimeError("ExecuteChessMove action interface is unavailable")

        goal = ExecuteChessMove.Goal()
        goal.instruction = self.instruction
        goal.from_square = self.from_square
        goal.to_square = self.to_square
        goal.is_capture = self.is_capture
        goal.pick_point = self.pick_point
        goal.place_point = self.place_point
        goal.target_frame = board_frame

        if feasibility_resp is not None:
            if hasattr(feasibility_resp, "pick_ik_solution") and getattr(
                feasibility_resp.pick_ik_solution, "name", None
            ):
                goal.pick_ik_hint = feasibility_resp.pick_ik_solution
            if hasattr(feasibility_resp, "place_ik_solution") and getattr(
                feasibility_resp.place_ik_solution, "name", None
            ):
                goal.place_ik_hint = feasibility_resp.place_ik_solution
        return goal


StageBuilderFunc = Callable[
    [CheckMoveFeasibility.Response, ChessMoveGoal], list[ExecutionStage]
]


class StagePipelineBuilder:
    """Transforms feasibility calculation results into an ordered list of ExecutionStages."""

    _BUILDERS: ClassVar[dict[int, StageBuilderFunc]] = {}

    @classmethod
    def register_builder(cls, plan_type: int, builder: StageBuilderFunc) -> None:
        """Register or override a stage builder for a specific plan type."""
        cls._BUILDERS[plan_type] = builder

    @classmethod
    def build_stages(
        cls,
        resp: CheckMoveFeasibility.Response,
        details: ChessMoveGoal,
    ) -> list[ExecutionStage]:
        """Entry point for building stages across all feasibility plan types via Strategy Map."""
        builder = cls._BUILDERS.get(resp.plan_type, cls._build_fallback)
        return builder(resp, details)

    @staticmethod
    def _build_zero_nav(
        resp: CheckMoveFeasibility.Response,
        details: ChessMoveGoal,
    ) -> list[ExecutionStage]:
        return [
            ExecutionStage(
                name="MOVE",
                target_pose=None,
                instruction=f"Pick {details.from_square}, place {details.to_square}",
                from_square=details.from_square,
                to_square=details.to_square,
                is_capture=False,
                pick_point=resp.pick_point,
                place_point=resp.place_point,
                motion_state=MotionExecutionState.PICKING_PIECE,
                nav_motion_state=None,
            )
        ]

    @staticmethod
    def _build_single_base(
        resp: CheckMoveFeasibility.Response,
        details: ChessMoveGoal,
    ) -> list[ExecutionStage]:
        return [
            ExecutionStage(
                name="MOVE",
                target_pose=resp.pick_base_pose,
                instruction=f"Pick {details.from_square}, place {details.to_square}",
                from_square=details.from_square,
                to_square=details.to_square,
                is_capture=False,
                pick_point=resp.pick_point,
                place_point=resp.place_point,
                motion_state=MotionExecutionState.PICKING_PIECE,
                nav_motion_state=MotionExecutionState.NAV_TO_PICK,
            )
        ]

    @staticmethod
    def _build_dual_base(
        resp: CheckMoveFeasibility.Response,
        details: ChessMoveGoal,
    ) -> list[ExecutionStage]:
        return [
            ExecutionStage(
                name="PICK",
                target_pose=resp.pick_base_pose,
                instruction=f"Pick {details.from_square}",
                from_square=details.from_square,
                to_square="",
                is_capture=False,
                pick_point=resp.pick_point,
                place_point=Point(),
                motion_state=MotionExecutionState.PICKING_PIECE,
                nav_motion_state=MotionExecutionState.NAV_TO_PICK,
            ),
            ExecutionStage(
                name="PLACE",
                target_pose=resp.place_base_pose,
                instruction=f"Place {details.to_square}",
                from_square="",
                to_square=details.to_square,
                is_capture=False,
                pick_point=Point(),
                place_point=resp.place_point,
                motion_state=MotionExecutionState.PLACING_PIECE,
                nav_motion_state=MotionExecutionState.NAV_TO_PLACE,
            ),
        ]

    @staticmethod
    def _build_capture_zero_nav(
        resp: CheckMoveFeasibility.Response,
        details: ChessMoveGoal,
    ) -> list[ExecutionStage]:
        cap_sq = details.captured_square or details.to_square
        return [
            ExecutionStage(
                name="CLEAR",
                target_pose=None,
                instruction=f"Clear {cap_sq} to onboard bin",
                from_square=cap_sq,
                to_square="",
                is_capture=True,
                pick_point=resp.clear_point,
                place_point=Point(),
                motion_state=MotionExecutionState.CLEARING_PIECE,
                nav_motion_state=None,
            ),
            ExecutionStage(
                name="MOVE",
                target_pose=None,
                instruction=f"Pick {details.from_square}, place {details.to_square}",
                from_square=details.from_square,
                to_square=details.to_square,
                is_capture=False,
                pick_point=resp.pick_point,
                place_point=resp.place_point,
                motion_state=MotionExecutionState.PICKING_PIECE,
                nav_motion_state=None,
            ),
        ]

    @staticmethod
    def _build_capture_single_base(
        resp: CheckMoveFeasibility.Response,
        details: ChessMoveGoal,
    ) -> list[ExecutionStage]:
        cap_sq = details.captured_square or details.to_square
        return [
            ExecutionStage(
                name="CLEAR",
                target_pose=resp.clear_base_pose,
                instruction=f"Clear {cap_sq} to onboard bin",
                from_square=cap_sq,
                to_square="",
                is_capture=True,
                pick_point=resp.clear_point,
                place_point=Point(),
                motion_state=MotionExecutionState.CLEARING_PIECE,
                nav_motion_state=MotionExecutionState.NAV_TO_CLEAR,
            ),
            ExecutionStage(
                name="MOVE",
                target_pose=None,
                instruction=f"Pick {details.from_square}, place {details.to_square}",
                from_square=details.from_square,
                to_square=details.to_square,
                is_capture=False,
                pick_point=resp.pick_point,
                place_point=resp.place_point,
                motion_state=MotionExecutionState.PICKING_PIECE,
                nav_motion_state=None,
            ),
        ]

    @staticmethod
    def _build_capture_dual_base(
        resp: CheckMoveFeasibility.Response,
        details: ChessMoveGoal,
    ) -> list[ExecutionStage]:
        cap_sq = details.captured_square or details.to_square
        dx = resp.clear_base_pose.pose.position.x - resp.pick_base_pose.pose.position.x
        dy = resp.clear_base_pose.pose.position.y - resp.pick_base_pose.pose.position.y
        clear_same_as_pick = math.hypot(dx, dy) < 0.05

        return [
            ExecutionStage(
                name="CLEAR",
                target_pose=resp.clear_base_pose,
                instruction=f"Clear {cap_sq} to onboard bin",
                from_square=cap_sq,
                to_square="",
                is_capture=True,
                pick_point=resp.clear_point,
                place_point=Point(),
                motion_state=MotionExecutionState.CLEARING_PIECE,
                nav_motion_state=MotionExecutionState.NAV_TO_CLEAR,
            ),
            ExecutionStage(
                name="PICK",
                target_pose=None if clear_same_as_pick else resp.pick_base_pose,
                instruction=f"Pick {details.from_square}",
                from_square=details.from_square,
                to_square="",
                is_capture=False,
                pick_point=resp.pick_point,
                place_point=Point(),
                motion_state=MotionExecutionState.PICKING_PIECE,
                nav_motion_state=(
                    None if clear_same_as_pick else MotionExecutionState.NAV_TO_PICK
                ),
            ),
            ExecutionStage(
                name="PLACE",
                target_pose=resp.place_base_pose,
                instruction=f"Place {details.to_square}",
                from_square="",
                to_square=details.to_square,
                is_capture=False,
                pick_point=Point(),
                place_point=resp.place_point,
                motion_state=MotionExecutionState.PLACING_PIECE,
                nav_motion_state=MotionExecutionState.NAV_TO_PLACE,
            ),
        ]

    @staticmethod
    def _build_capture_triple_base(
        resp: CheckMoveFeasibility.Response,
        details: ChessMoveGoal,
    ) -> list[ExecutionStage]:
        cap_sq = details.captured_square or details.to_square
        return [
            ExecutionStage(
                name="CLEAR",
                target_pose=resp.clear_base_pose,
                instruction=f"Clear {cap_sq} to onboard bin",
                from_square=cap_sq,
                to_square="",
                is_capture=True,
                pick_point=resp.clear_point,
                place_point=Point(),
                motion_state=MotionExecutionState.CLEARING_PIECE,
                nav_motion_state=MotionExecutionState.NAV_TO_CLEAR,
            ),
            ExecutionStage(
                name="PICK",
                target_pose=resp.pick_base_pose,
                instruction=f"Pick {details.from_square}",
                from_square=details.from_square,
                to_square="",
                is_capture=False,
                pick_point=resp.pick_point,
                place_point=Point(),
                motion_state=MotionExecutionState.PICKING_PIECE,
                nav_motion_state=MotionExecutionState.NAV_TO_PICK,
            ),
            ExecutionStage(
                name="PLACE",
                target_pose=resp.place_base_pose,
                instruction=f"Place {details.to_square}",
                from_square="",
                to_square=details.to_square,
                is_capture=False,
                pick_point=Point(),
                place_point=resp.place_point,
                motion_state=MotionExecutionState.PLACING_PIECE,
                nav_motion_state=MotionExecutionState.NAV_TO_PLACE,
            ),
        ]

    @staticmethod
    def _build_fallback(
        resp: CheckMoveFeasibility.Response,
        details: ChessMoveGoal,
    ) -> list[ExecutionStage]:
        target_pose = getattr(resp, "pick_base_pose", None)
        return [
            ExecutionStage(
                name="MOVE",
                target_pose=target_pose,
                instruction=f"Pick {details.from_square}, place {details.to_square}",
                from_square=details.from_square,
                to_square=details.to_square,
                is_capture=details.is_capture,
                pick_point=getattr(resp, "pick_point", Point()),
                place_point=getattr(resp, "place_point", Point()),
                motion_state=MotionExecutionState.PICKING_PIECE,
                nav_motion_state=(
                    MotionExecutionState.NAV_TO_PICK if target_pose else None
                ),
            )
        ]


# Default registry mapping feasibility plan types to specialized stage builders
StagePipelineBuilder._BUILDERS = {
    CheckMoveFeasibility.Response.PLAN_ZERO_NAV: StagePipelineBuilder._build_zero_nav,
    CheckMoveFeasibility.Response.PLAN_SINGLE_BASE: StagePipelineBuilder._build_single_base,
    CheckMoveFeasibility.Response.PLAN_DUAL_BASE: StagePipelineBuilder._build_dual_base,
    CheckMoveFeasibility.Response.PLAN_CAPTURE_ZERO_NAV: StagePipelineBuilder._build_capture_zero_nav,
    CheckMoveFeasibility.Response.PLAN_CAPTURE_SINGLE_BASE: StagePipelineBuilder._build_capture_single_base,
    CheckMoveFeasibility.Response.PLAN_CAPTURE_DUAL_BASE: StagePipelineBuilder._build_capture_dual_base,
    CheckMoveFeasibility.Response.PLAN_CAPTURE_TRIPLE_BASE: StagePipelineBuilder._build_capture_triple_base,
}


class MovePipelineExecutor:
    """
    Executes a multi-stage chess move pipeline across navigation and manipulation actuators.
    Drives Level 2 MotionExecutionState transitions and post-move verification.
    """

    def __init__(
        self,
        node: Node,
        dispatcher: ActionDispatcherInterface,
        perception: PerceptionContextCoordinator,
        board_frame: str = "chessboard_frame",
        action_timeout_sec: float = 60.0,
        on_pipeline_completed: Callable[[], None] | None = None,
        on_pipeline_failed: Callable[[str], None] | None = None,
        on_motion_state_changed: Callable[[MotionExecutionState], None] | None = None,
        grasp_readiness_provider: Callable[[], bool] | None = None,
        on_observation_recovery_requested: Callable[[], bool] | None = None,
    ) -> None:
        self._node = node
        self._dispatcher = dispatcher
        self._perception = perception
        self._board_frame = board_frame
        self._action_timeout_sec = action_timeout_sec

        self._on_pipeline_completed = on_pipeline_completed
        self._on_pipeline_failed = on_pipeline_failed
        self._on_motion_state_changed = on_motion_state_changed
        self._grasp_readiness_provider = grasp_readiness_provider or (lambda: True)
        self._on_observation_recovery_requested = on_observation_recovery_requested

        self._lock = threading.RLock()
        self._motion_state = MotionExecutionState.IDLE
        self._stages: list[ExecutionStage] = []
        self._current_stage: ExecutionStage | None = None
        self._feasibility_resp: CheckMoveFeasibility.Response | None = None

    @property
    def motion_state(self) -> MotionExecutionState:
        with self._lock:
            return self._motion_state

    @property
    def current_stage(self) -> ExecutionStage | None:
        with self._lock:
            return self._current_stage

    @property
    def is_active(self) -> bool:
        with self._lock:
            return bool(self._stages or self._current_stage is not None)

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
        stages: list[ExecutionStage],
        feasibility_resp: CheckMoveFeasibility.Response,
    ) -> None:
        """Begin execution of a newly built list of ExecutionStages."""
        with self._lock:
            self._stages = list(stages)
            self._feasibility_resp = feasibility_resp
            self._current_stage = None

        self._node.get_logger().info(
            f"[PIPELINE EXECUTOR] Starting execution of {len(stages)} stages..."
        )
        self._advance_stage()

    def _advance_stage(self) -> None:
        """Advance to the next ExecutionStage or complete pipeline."""
        with self._lock:
            if not self._stages:
                self._current_stage = None
                self._feasibility_resp = None
                self.transition_motion_to(MotionExecutionState.IDLE)
                if self._on_pipeline_completed:
                    self._on_pipeline_completed()
                return

            stage = self._stages.pop(0)
            self._current_stage = stage

        if stage.target_pose is not None:
            self._execute_nav_stage(stage)
        else:
            if not self._grasp_readiness_provider():
                self._node.get_logger().warn(
                    f"[PRE-GRASP GATE] Starting stage '{stage.name}' without navigation, "
                    f"but grasp readiness is FALSE (standstill/chessboard TF missing). "
                    f"Initiating active observation recovery..."
                )
                if self._on_observation_recovery_requested:
                    triggered = self._on_observation_recovery_requested()
                    if triggered:
                        with self._lock:
                            self._stages.clear()
                            self._current_stage = None
                            self._feasibility_resp = None
                            self.transition_motion_to(MotionExecutionState.IDLE)
                        return
                self._handle_failure(
                    f"Grasp readiness check failed before stage '{stage.name}' and active observation unavailable"
                )
                return
            self._execute_manip_stage(stage)

    def _execute_nav_stage(self, stage: ExecutionStage) -> None:
        nav_state = stage.nav_motion_state or MotionExecutionState.NAV_TO_PICK
        self.transition_motion_to(nav_state)
        self._perception.set_context(PerceptionContext.TF_TRACKING_AND_NAV)

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

    def _on_nav_completed(self, result: ActionResult, stage: ExecutionStage) -> None:
        if not result.success:
            self._handle_failure(f"Nav2 navigation failed: {result.message}")
            return

        self._node.get_logger().info(
            f"Nav2 navigation for stage '{stage.name}' completed successfully! Checking grasp readiness..."
        )
        if not self._grasp_readiness_provider():
            self._node.get_logger().warn(
                f"[PRE-GRASP GATE] Standoff reached for stage '{stage.name}', "
                f"but grasp readiness is FALSE (visual tag occluded / base moving). "
                f"Initiating active observation recovery without crashing."
            )
            if self._on_observation_recovery_requested:
                triggered = self._on_observation_recovery_requested()
                if triggered:
                    with self._lock:
                        self._stages.clear()
                        self._current_stage = None
                        self._feasibility_resp = None
                        self.transition_motion_to(MotionExecutionState.IDLE)
                    return
            self._handle_failure(
                f"Grasp readiness check failed before stage '{stage.name}' and active observation unavailable"
            )
            return

        self._execute_manip_stage(stage)

    def _execute_manip_stage(self, stage: ExecutionStage) -> None:
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

    def _on_manip_completed(self, result: ActionResult, stage: ExecutionStage) -> None:
        if not result.success:
            self._handle_failure(f"Manipulation execution failed: {result.message}")
            return

        self._node.get_logger().info(
            f"Manipulation execution successful in {result.execution_time_sec:.2f}s: {result.message}"
        )

        if stage.name in ("PLACE", "MOVE"):
            if self._motion_state == MotionExecutionState.PICKING_PIECE:
                self.transition_motion_to(MotionExecutionState.PLACING_PIECE)
            self.transition_motion_to(MotionExecutionState.POST_MOVE_VERIFYING)
            self._perception.set_context(PerceptionContext.POST_MOVE_VERIFY)
            self._execute_post_move_verify()
        else:
            self._advance_stage()

    def _execute_post_move_verify(self) -> None:
        """One-shot post-move verification before concluding move."""
        self._node.get_logger().info(
            "[VERIFICATION] Post-move board state verification completed."
        )
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
        self._dispatcher.cancel_active_goal()
        with self._lock:
            self._stages.clear()
            self._current_stage = None
            self._feasibility_resp = None
            self.transition_motion_to(MotionExecutionState.IDLE)
