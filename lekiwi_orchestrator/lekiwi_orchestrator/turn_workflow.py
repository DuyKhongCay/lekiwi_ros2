# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""
Turn workflow management for autonomous chess moves.

Encapsulates:
1. Referee signal processing and FIDE game status verification (RefereeHandler)
2. Reachability feasibility checking and move staging execution (MoveWorkflowCoordinator)
3. Post-move active perception watchdog and vantage point repositioning (PostMoveWatchdog)
"""

from __future__ import annotations

import math
import threading
from collections.abc import Callable
from typing import TYPE_CHECKING, Any, Protocol

from geometry_msgs.msg import PoseStamped
from lekiwi_interfaces.msg import ChessGameStatus, ChessMoveDetails, PerceptionContext
from lekiwi_interfaces.srv import CheckMoveFeasibility
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.timer import Timer

from lekiwi_orchestrator.board_geometry import (
    compute_radial_entry_pose,
    get_default_approach_yaw,
)
from lekiwi_orchestrator.fsm import MacroMissionState
from lekiwi_orchestrator.mission_types import ActionResult, ChessMoveGoal, ObservationIntent
from lekiwi_orchestrator.move_planner import MovePlanBuilder

if TYPE_CHECKING:
    from lekiwi_orchestrator.motion_client import MotionClient
    from lekiwi_orchestrator.move_sequencer import MoveSequencer
    from lekiwi_orchestrator.obs_navigator import ObsNavigator
    from lekiwi_orchestrator.parameters import OrchestratorParameters
    from lekiwi_orchestrator.perception_context import PerceptionContextManager


class TurnHost(Protocol):
    """Protocol defining host node capabilities required by turn workflow coordinators."""

    @property
    def mission_state(self) -> MacroMissionState: ...

    @property
    def is_nav_ready(self) -> bool: ...

    def transition_to(self, target_state: MacroMissionState) -> bool: ...

    def set_current_move(self, details: ChessMoveGoal) -> None: ...

    def clear_move_goals(self) -> None: ...

    def set_last_interaction_pose(self, pose: PoseStamped | None) -> None: ...

    def start_post_move_watchdog(self) -> None: ...




# ==============================================================================
# Referee and FIDE Game Status Handler
# ==============================================================================


class GameStatusHandler:
    """Processes ChessGameStatus referee signals and gates turn eligibility."""

    def __init__(
        self,
        node: Node,
        config: OrchestratorParameters,
        perception: PerceptionContextManager,
        state_lock: threading.RLock,
        get_mission_state: Callable[[], MacroMissionState],
        transition_to_fn: Callable[[MacroMissionState], bool],
        finalize_turn_fn: Callable[[], None],
        dispatch_workflow_fn: Callable[[str, Any], None],
        reset_obs_viewpoints_fn: Callable[[], None],
    ) -> None:
        self._node = node
        self._config = config
        self._perception = perception
        self._state_lock = state_lock
        self._get_mission_state = get_mission_state
        self._transition_to_fn = transition_to_fn
        self._finalize_turn_fn = finalize_turn_fn
        self._dispatch_workflow_fn = dispatch_workflow_fn
        self._reset_obs_viewpoints_fn = reset_obs_viewpoints_fn

        self._last_processed_fen = ""

    @property
    def last_processed_fen(self) -> str:
        return self._last_processed_fen

    @last_processed_fen.setter
    def last_processed_fen(self, fen: str) -> None:
        self._last_processed_fen = fen

    def handle_game_over_if_ended(self, msg: ChessGameStatus) -> bool:
        """Check and transition to GAME_OVER if referee reports checkmate or draw."""
        if not (msg.is_checkmate or msg.is_draw):
            return False

        with self._state_lock:
            current_state = self._get_mission_state()

        if current_state != MacroMissionState.GAME_OVER:
            self._transition_to_fn(MacroMissionState.GAME_OVER)
            reason = "CHECKMATE" if msg.is_checkmate else "DRAW"
            self._node.get_logger().info(
                f"*** GAME OVER: {reason}! FEN: {msg.full_fen} ***"
            )
            self._perception.set_context(PerceptionContext.IDLE_STANDBY)
        return True

    def verify_post_move_board(self, msg: ChessGameStatus) -> bool:
        """Handle FIDE verification while in POST_MOVE_VERIFYING state."""
        with self._state_lock:
            if self._get_mission_state() != MacroMissionState.POST_MOVE_VERIFYING:
                return False

        if msg.is_board_stable and msg.is_legal_move:
            self._node.get_logger().info(
                f">>> [POST-MOVE VERIFICATION] Board state confirmed stable & legal (FEN: {msg.full_fen}). Finalizing turn."
            )
            self._finalize_turn_fn()
        return True

    def is_eligible_robot_turn(self, msg: ChessGameStatus) -> bool:
        """Determine if referee message indicates a valid turn ready for robot execution."""
        is_robot_turn = msg.active_color == self._config.robot_color
        best_uci = bool(msg.best_move_details.uci)
        is_phase_ready = msg.game_phase in (
            ChessGameStatus.PHASE_ROBOT_READY,
            ChessGameStatus.PHASE_WAITING_PLAYER,
        )
        return msg.is_board_stable and is_robot_turn and best_uci and is_phase_ready

    def on_game_status(
        self,
        msg: ChessGameStatus,
        clear_pending_recovery_cb: Callable[[], None] | None = None,
    ) -> None:
        """Process game state updates from lekiwi_chess_master referee."""
        if self.handle_game_over_if_ended(msg):
            return

        if self.verify_post_move_board(msg):
            return

        if not self.is_eligible_robot_turn(msg):
            return

        with self._state_lock:
            if clear_pending_recovery_cb is not None:
                clear_pending_recovery_cb()

            eligible_state = self._get_mission_state() in (
                MacroMissionState.WAITING_FOR_PLAYER_MOVE,
                MacroMissionState.EVALUATING_BEST_MOVE,
                MacroMissionState.TURN_COMPLETED,
            )
            if not eligible_state or msg.full_fen == self._last_processed_fen:
                return

            self._last_processed_fen = msg.full_fen

        details = msg.best_move_details
        self._node.get_logger().info(
            f">>> [ORCHESTRATOR] New Best Move Received: '{details.uci}' "
            f"(SAN: {details.san}, Piece: {details.piece_type}, "
            f"Capture: {details.is_capture}, Eval: {msg.eval_centipawns} cp, Color: {msg.active_color})"
        )
        self._reset_obs_viewpoints_fn()
        self._dispatch_workflow_fn(details.uci, details)





# ==============================================================================
# Move Workflow Coordinator
# ==============================================================================


class MoveWorkflow:
    """Coordinates reachability feasibility checks, step plan assembly, and move dispatch."""

    def __init__(
        self,
        node: Node,
        config: OrchestratorParameters,
        host: TurnHost,
        dispatcher_provider: Callable[[], MotionClient],
        observation_nav: ObsNavigator,
        pipeline_executor: MoveSequencer,
        perception: PerceptionContextManager,
    ) -> None:
        self._node = node
        self._config = config
        self._host = host
        self._dispatcher_provider = dispatcher_provider
        self._obs_nav = observation_nav
        self._pipeline_executor = pipeline_executor
        self._perception = perception

    @staticmethod
    def extract_valid_base_pose(
        resp: CheckMoveFeasibility.Response,
    ) -> PoseStamped | None:
        """Extract populated base pose from feasibility response, or None if unpopulated."""
        for attr in ("place_base_pose", "pick_base_pose"):
            pose = getattr(resp, attr, None)
            if pose is not None:
                has_frame = bool(getattr(pose.header, "frame_id", ""))
                has_pos = (
                    abs(pose.pose.position.x) > 1e-4
                    or abs(pose.pose.position.y) > 1e-4
                    or abs(pose.pose.position.z) > 1e-4
                )
                if has_frame or has_pos:
                    return pose
        return None

    def dispatch_move_workflow(
        self,
        uci_move: str,
        move_details: ChessMoveDetails | None = None,
    ) -> None:
        """Begin autonomous execution of a single chess move."""
        if not self._host.is_nav_ready:
            self._node.get_logger().warn(
                f"Cannot dispatch move '{uci_move}': Navigation system not ready. Waiting for navigation readiness."
            )
            self._host.transition_to(MacroMissionState.WAITING_FOR_TF_READY)
            self._perception.set_context(PerceptionContext.TF_TRACKING_AND_NAV)
            return

        details = ChessMoveGoal.from_uci_or_details(uci_move, move_details)
        if details is None:
            self._node.get_logger().error(
                f"Failed to parse UCI move '{uci_move}': Expected format like 'e2e4' or 'e7e8q'."
            )
            self._host.transition_to(MacroMissionState.ERROR_FALLBACK)
            return

        self._host.set_current_move(details)

        cur_state = self._host.mission_state
        if cur_state in (
            MacroMissionState.WAITING_FOR_PLAYER_MOVE,
            MacroMissionState.TURN_COMPLETED,
            MacroMissionState.WAITING_FOR_TF_READY,
        ) and not self._host.transition_to(MacroMissionState.EVALUATING_BEST_MOVE):
            return

        if not self._host.transition_to(MacroMissionState.CHECKING_REACHABILITY):
            return

        self._node.get_logger().info(
            f"Querying reachability feasibility for {details.from_square} -> {details.to_square}..."
        )
        dispatcher = self._dispatcher_provider()
        dispatched = dispatcher.check_feasibility(
            goal=details,
            timeout_sec=self._config.feasibility_timeout_sec,
            on_success=lambda resp: self.on_feasibility_response(resp, details),
            on_error=self.on_feasibility_error,
        )
        if not dispatched:
            self._host.transition_to(MacroMissionState.ERROR_FALLBACK)

    def on_feasibility_error(self, error_msg: str) -> None:
        self._node.get_logger().error(
            f"Workspace feasibility query failed: {error_msg}"
        )
        self._host.transition_to(MacroMissionState.ERROR_FALLBACK)

    def on_feasibility_response(
        self, resp: CheckMoveFeasibility.Response, details: ChessMoveGoal
    ) -> None:
        self._host.set_current_move(details)

        if not resp.feasible:
            self._node.get_logger().error(
                f"Move {details.uci} declared NOT FEASIBLE: {resp.message}"
            )
            self._host.transition_to(MacroMissionState.ERROR_FALLBACK)
            return

        self._node.get_logger().info(
            f"Feasibility confirmed! Plan Type: {resp.plan_type} ({resp.message})"
        )

        last_base_pose = self.extract_valid_base_pose(resp)
        self._host.set_last_interaction_pose(last_base_pose)

        if self._config.navigation:
            curr_bx, curr_by = self._obs_nav.get_current_board_position(
                reference_pose=last_base_pose
            )
            fallback_yaw = get_default_approach_yaw(self._config.robot_color)
            radial_pose_board = compute_radial_entry_pose(
                curr_x=curr_bx,
                curr_y=curr_by,
                standoff_radius=self._config.observation_standoff_distance,
                board_frame=self._config.board_frame,
                fallback_yaw=fallback_yaw,
            )
            obs_pose = self._obs_nav.transform_board_pose_to_map(radial_pose_board)
        else:
            obs_pose = None

        stages = MovePlanBuilder.build_stages(
            resp, details, observation_pose=obs_pose
        )
        if not self._host.transition_to(MacroMissionState.EXECUTING_MOVE_PIPELINE):
            return

        self._pipeline_executor.start_pipeline(stages, resp)

    def on_move_pipeline_completed(self) -> None:
        """Physical move pipeline completed: transition to POST_MOVE_VERIFYING and start active verification."""
        self._host.clear_move_goals()
        self._host.transition_to(MacroMissionState.POST_MOVE_VERIFYING)
        self._perception.set_context(PerceptionContext.BOARD_STATE_SCAN)
        self._host.start_post_move_watchdog()

    def on_move_pipeline_failed(self, error_msg: str) -> None:
        self._node.get_logger().error(f"Move pipeline execution failed: {error_msg}")
        self._host.transition_to(MacroMissionState.ERROR_FALLBACK)




# ==============================================================================
# Post-Move Verification Watchdog
# ==============================================================================


class PostMoveVerifier:
    """Manages post-move verification timeout and active observation repositioning."""

    def __init__(
        self,
        node: Node,
        config: OrchestratorParameters,
        observation_nav: ObsNavigator,
        perception: PerceptionContextManager,
        state_lock: threading.RLock,
        get_mission_state: Callable[[], MacroMissionState],
        get_last_interaction_pose: Callable[[], PoseStamped | None],
        on_finalize_turn: Callable[[], None],
        callback_group: CallbackGroup | None = None,
    ) -> None:
        self._node = node
        self._config = config
        self._observation_nav = observation_nav
        self._perception = perception
        self._state_lock = state_lock
        self._get_mission_state = get_mission_state
        self._get_last_interaction_pose = get_last_interaction_pose
        self._on_finalize_turn = on_finalize_turn
        self._callback_group = callback_group

        self._timer: Timer | None = None
        self._post_move_attempts = 0

    @property
    def attempts(self) -> int:
        return self._post_move_attempts

    @property
    def is_active(self) -> bool:
        """Return True if watchdog timer is running."""
        return self._timer is not None

    def reset(self) -> None:
        self.stop()
        self._post_move_attempts = 0
        if self._observation_nav is not None:
            self._observation_nav.reset_viewpoint_index(
                ObservationIntent.POST_MOVE_VERIFY
            )

    def start(self) -> None:
        """Start verification timer while in POST_MOVE_VERIFYING state."""
        self.stop()
        timeout = max(1.0, float(self._config.observation_scan_timeout_sec))
        self._node.get_logger().info(
            f"[POST-MOVE VERIFICATION] Starting verification watchdog ({timeout:.1f}s)..."
        )
        self._timer = self._node.create_timer(
            timeout,
            self.on_timeout,
            callback_group=self._callback_group,
        )

    def stop(self) -> None:
        """Cancel and clean up verification watchdog timer."""
        if self._timer is not None:
            self._timer.cancel()
            self._node.destroy_timer(self._timer)
            self._timer = None

    def on_timeout(self) -> None:
        """Handle verification timeout: board is occluded or unconfirmed -> reposition viewpoint."""
        self.stop()
        with self._state_lock:
            if self._get_mission_state() != MacroMissionState.POST_MOVE_VERIFYING:
                return

        if not self._config.navigation:
            self._node.get_logger().warn(
                "[POST-MOVE VERIFICATION] Verification timed out and navigation disabled. Finalizing turn."
            )
            self._on_finalize_turn()
            return

        max_attempts = self._observation_nav.get_max_attempts(
            ObservationIntent.POST_MOVE_VERIFY
        )
        if self._post_move_attempts >= max_attempts:
            self._node.get_logger().warn(
                f"[POST-MOVE VERIFICATION] Reached max reposition attempts ({max_attempts}). Finalizing turn."
            )
            self._on_finalize_turn()
            return

        self._post_move_attempts += 1
        self._node.get_logger().warn(
            f"[ACTIVE VISION] Post-move board occluded/unverified ({self._config.observation_scan_timeout_sec:.1f}s). "
            f"Attempt {self._post_move_attempts}/{max_attempts}. Repositioning base to next candidate vantage point..."
        )
        self._perception.set_context(PerceptionContext.TF_TRACKING_AND_NAV)

        def _on_repositioned(res: ActionResult) -> None:
            with self._state_lock:
                if self._get_mission_state() != MacroMissionState.POST_MOVE_VERIFYING:
                    return
            if res.success:
                self._node.get_logger().info(
                    "[ACTIVE PERCEPTION] Reached observation viewpoint. Resuming board scan."
                )
            else:
                self._node.get_logger().warn(
                    f"[ACTIVE PERCEPTION] Failed to reach observation viewpoint: {res.message}"
                )
            self._perception.set_context(PerceptionContext.BOARD_STATE_SCAN)
            self.start()

        dispatched = self._observation_nav.reposition_to_next_viewpoint(
            intent=ObservationIntent.POST_MOVE_VERIFY,
            reference_pose=self._get_last_interaction_pose(),
            board_x=0.0,
            board_y=0.0,
            board_yaw=0.0,
            timeout_sec=self._config.action_timeout_sec,
            on_completed=_on_repositioned,
        )
        if not dispatched:
            self._node.get_logger().error(
                "[ACTIVE PERCEPTION] Failed to dispatch repositioning goal! Finalizing turn."
            )
            self._on_finalize_turn()


__all__ = [
    "TurnHost",
    "GameStatusHandler",
    "MoveWorkflow",
    "PostMoveVerifier",
]

