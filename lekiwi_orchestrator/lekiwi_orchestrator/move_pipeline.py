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
import time
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
    from rclpy.callback_groups import CallbackGroup
    from rclpy.node import Node

    from lekiwi_orchestrator.motion_dispatcher import (
        ActionDispatcherInterface,
        ActionResult,
        ActiveObservationNavigator,
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
        pre_grasp_settle_sec: float = 2.0,
        observation_pose_provider: Callable[[], PoseStamped | None] | None = None,
        navigation_enabled: bool = True,
        observation_navigator: ActiveObservationNavigator | None = None,
        observation_scan_timeout_sec: float = 10.0,
        board_verified_provider: Callable[[], bool] | None = None,
        max_observation_attempts: int = 3,
        callback_group: CallbackGroup | None = None,
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
        self._pre_grasp_settle_sec = max(0.0, float(pre_grasp_settle_sec))
        self._observation_pose_provider = observation_pose_provider
        self._navigation_enabled = navigation_enabled
        self._observation_nav = observation_navigator
        self._observation_scan_timeout_sec = max(1.0, float(observation_scan_timeout_sec))
        self._board_verified_provider = board_verified_provider
        self._max_observation_attempts = max(1, int(max_observation_attempts))
        self._callback_group = callback_group

        self._lock = threading.RLock()
        self._motion_state = MotionExecutionState.IDLE
        self._stages: list[ExecutionStage] = []
        self._current_stage: ExecutionStage | None = None
        self._feasibility_resp: CheckMoveFeasibility.Response | None = None
        self._verification_timer: Any | None = None
        self._observation_attempt: int = 0

    @property
    def observation_navigator(self) -> ActiveObservationNavigator | None:
        return self._observation_nav

    @observation_navigator.setter
    def observation_navigator(self, nav: ActiveObservationNavigator | None) -> None:
        self._observation_nav = nav

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
            self._observation_attempt = 0
            self._cancel_verification_timer()

        if self._observation_nav is not None:
            self._observation_nav.reset_viewpoint_index()

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
            if not self._wait_for_grasp_readiness(self._pre_grasp_settle_sec):
                self._handle_failure(
                    f"Grasp readiness check failed before stage '{stage.name}' "
                    f"(settle timeout {self._pre_grasp_settle_sec:.1f}s expired)"
                )
                return
            self._execute_manip_stage(stage)

    def _wait_for_grasp_readiness(self, timeout_sec: float) -> bool:
        """Poll grasp readiness provider until True or timeout_sec expires."""
        if self._grasp_readiness_provider():
            return True
        if timeout_sec <= 0.0:
            return False

        start_time = time.monotonic()
        poll_interval = 0.05
        while (time.monotonic() - start_time) < timeout_sec:
            time.sleep(poll_interval)
            if self._grasp_readiness_provider():
                return True
        return self._grasp_readiness_provider()

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
            f"Nav2 navigation for stage '{stage.name}' completed successfully! "
            f"Checking grasp readiness (settle window: {self._pre_grasp_settle_sec:.1f}s)..."
        )
        if not self._wait_for_grasp_readiness(self._pre_grasp_settle_sec):
            self._handle_failure(
                f"Grasp readiness check failed before stage '{stage.name}' "
                f"after standoff reached (settle timeout {self._pre_grasp_settle_sec:.1f}s expired)"
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
        """Post-move verification: navigate base back to observation pose to bring full board into FOV."""
        self._observation_attempt = 0
        if not self._navigation_enabled or self._observation_pose_provider is None:
            self._node.get_logger().info(
                "[VERIFICATION] Observation navigation skipped (disabled or no provider). Switching to BOARD_STATE_SCAN."
            )
            self._start_post_move_verification_window()
            return

        obs_pose = self._observation_pose_provider()
        if obs_pose is None:
            self._node.get_logger().info(
                "[VERIFICATION] No observation pose provided. Switching to BOARD_STATE_SCAN."
            )
            self._start_post_move_verification_window()
            return

        self._node.get_logger().info(
            f"[POST-MOVE VERIFICATION] Repositioning base to observation standoff "
            f"({obs_pose.pose.position.x:.3f}, {obs_pose.pose.position.y:.3f}) "
            "to bring full chessboard into FOV..."
        )
        self.transition_motion_to(MotionExecutionState.POST_MOVE_VERIFYING)
        self._perception.set_context(PerceptionContext.TF_TRACKING_AND_NAV)

        dispatched = self._dispatcher.send_navigation_goal(
            target_pose=obs_pose,
            timeout_sec=self._action_timeout_sec,
            on_completed=self._on_post_move_nav_completed,
        )
        if not dispatched:
            self._node.get_logger().warn(
                "[POST-MOVE VERIFICATION] Failed to dispatch observation navigation. "
                "Falling back to verification window."
            )
            self._start_post_move_verification_window()

    def _on_post_move_nav_completed(self, result: ActionResult) -> None:
        if not result.success:
            self._node.get_logger().warn(
                f"[POST-MOVE VERIFICATION] Observation navigation ended with warning: {result.message}"
            )
        else:
            self._node.get_logger().info(
                "[POST-MOVE VERIFICATION] Reached observation viewpoint. "
                "Full chessboard in FOV. Starting verification window."
            )
        self._start_post_move_verification_window()

    def _start_post_move_verification_window(self) -> None:
        """Begin verification window: verify board state or reposition if occluded."""
        self._perception.set_context(PerceptionContext.BOARD_STATE_SCAN)

        if self._is_board_verified():
            self._node.get_logger().info(
                "[POST-MOVE VERIFICATION] Board state verified. Completing stage."
            )
            self._cancel_verification_timer()
            self._advance_stage()
            return

        if not self._navigation_enabled or self._observation_nav is None:
            self._node.get_logger().info(
                "[POST-MOVE VERIFICATION] Active observation navigator not configured. Completing stage."
            )
            self._advance_stage()
            return

        self._start_verification_timer()

    def _is_board_verified(self) -> bool:
        if self._board_verified_provider is None:
            return True
        try:
            return bool(self._board_verified_provider())
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().warn(f"Error checking board verification: {exc}")
            return False

    def _start_verification_timer(self) -> None:
        self._cancel_verification_timer()
        timeout = max(1.0, float(self._observation_scan_timeout_sec))
        self._node.get_logger().info(
            f"[POST-MOVE VERIFICATION] Verification window active ({timeout:.1f}s)..."
        )
        if hasattr(self._node, "create_timer"):
            kwargs = {}
            if self._callback_group is not None:
                kwargs["callback_group"] = self._callback_group
            self._verification_timer = self._node.create_timer(
                timeout,
                self._on_verification_timeout,
                **kwargs,
            )

    def _cancel_verification_timer(self) -> None:
        with self._lock:
            if self._verification_timer is not None:
                try:
                    self._verification_timer.cancel()
                    if hasattr(self._node, "destroy_timer"):
                        self._node.destroy_timer(self._verification_timer)
                except Exception as exc:  # noqa: BLE001
                    self._node.get_logger().warn(
                        f"Error cancelling verification timer: {exc}"
                    )
                self._verification_timer = None

    def _on_verification_timeout(self) -> None:
        self._cancel_verification_timer()
        with self._lock:
            if self._motion_state != MotionExecutionState.POST_MOVE_VERIFYING:
                return

        if self._is_board_verified():
            self._node.get_logger().info(
                "[POST-MOVE VERIFICATION] Board verified before repositioning."
            )
            self._advance_stage()
            return

        if self._observation_nav is None or not self._navigation_enabled:
            self._node.get_logger().warn(
                "[POST-MOVE VERIFICATION] Verification timed out and active vision unavailable. Completing stage."
            )
            self._advance_stage()
            return

        self._observation_attempt += 1
        if self._observation_attempt > self._max_observation_attempts:
            self._node.get_logger().warn(
                f"[POST-MOVE VERIFICATION] Reached max observation attempts ({self._max_observation_attempts}). Completing stage."
            )
            self._advance_stage()
            return

        self._node.get_logger().warn(
            f"[ACTIVE VISION] Board verification timed out. Attempt {self._observation_attempt}/{self._max_observation_attempts}. "
            "Repositioning base to next candidate vantage point..."
        )
        self._perception.set_context(PerceptionContext.TF_TRACKING_AND_NAV)
        dispatched = self._observation_nav.reposition_to_next_viewpoint(
            board_x=0.0,
            board_y=0.0,
            board_yaw=0.0,
            timeout_sec=self._action_timeout_sec,
            on_completed=self._on_reposition_completed,
        )
        if not dispatched:
            self._node.get_logger().error(
                "[POST-MOVE VERIFICATION] Failed to dispatch reposition goal. Completing stage."
            )
            self._advance_stage()

    def _on_reposition_completed(self, result: ActionResult) -> None:
        with self._lock:
            if self._motion_state != MotionExecutionState.POST_MOVE_VERIFYING:
                return

        if result.success:
            self._node.get_logger().info(
                "[POST-MOVE VERIFICATION] Reached next observation viewpoint. Checking board..."
            )
        else:
            self._node.get_logger().warn(
                f"[POST-MOVE VERIFICATION] Failed to reach observation viewpoint: {result.message}"
            )

        self._start_post_move_verification_window()

    def notify_board_verified(self) -> None:
        """External notification that chessboard has been successfully detected and verified."""
        with self._lock:
            if self._motion_state != MotionExecutionState.POST_MOVE_VERIFYING:
                return
        self._node.get_logger().info(
            "[POST-MOVE VERIFICATION] Board verified via external notification."
        )
        self._cancel_verification_timer()
        self._advance_stage()

    def _handle_failure(self, error_msg: str) -> None:
        self._node.get_logger().error(error_msg)
        self._cancel_verification_timer()
        with self._lock:
            self._stages.clear()
            self._current_stage = None
            self._feasibility_resp = None
            self.transition_motion_to(MotionExecutionState.IDLE)
        if self._on_pipeline_failed:
            self._on_pipeline_failed(error_msg)

    def cancel(self) -> None:
        """Cancel active physical actions and reset executor state."""
        self._cancel_verification_timer()
        self._observation_attempt = 0
        self._dispatcher.cancel_active_goal()
        with self._lock:
            self._stages.clear()
            self._current_stage = None
            self._feasibility_resp = None
            self.transition_motion_to(MotionExecutionState.IDLE)

    def destroy(self) -> None:
        """Clean up active timers and cancel any ongoing goals."""
        self._cancel_verification_timer()
        self.cancel()
