# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""
Turn workflow management for autonomous chess moves.

Encapsulates:
1. Referee signal processing and FIDE game status verification (GameStatusHandler)
2. Reachability feasibility checking and move staging execution (MoveWorkflowCoordinator)
3. Post-move active perception watchdog and vantage point repositioning (PostMoveVerifier)
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from typing import TYPE_CHECKING, Any, Protocol

from geometry_msgs.msg import PoseStamped
from lekiwi_interfaces.msg import ChessGameStatus, ChessMoveDetails, PerceptionContext
from lekiwi_interfaces.srv import CheckMoveFeasibility
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.timer import Timer

from lekiwi_orchestrator.core.fsm import MacroMissionState
from lekiwi_orchestrator.core.mission_types import ActionResult, ChessMoveGoal, ObservationIntent
from lekiwi_orchestrator.core.utils import get_node_now_sec
from lekiwi_orchestrator.planning.move_planner import MovePlanBuilder

if TYPE_CHECKING:
    from lekiwi_orchestrator.core.parameters import OrchestratorParameters
    from lekiwi_orchestrator.execution.motion_client import MotionClient
    from lekiwi_orchestrator.execution.move_sequencer import MoveSequencer
    from lekiwi_orchestrator.planning.obs_navigator import ObsNavigator
    from lekiwi_orchestrator.supervision.perception_context import PerceptionContextManager


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
        on_robot_color_updated_cb: Callable[[str], None] | None = None,
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
        self._on_robot_color_updated_cb = on_robot_color_updated_cb

        self._last_processed_fen = ""
        self._latest_full_fen = ""
        self._robot_color = ""

    @property
    def robot_color(self) -> str:
        with self._state_lock:
            return self._robot_color

    @property
    def last_processed_fen(self) -> str:
        return self._last_processed_fen

    @last_processed_fen.setter
    def last_processed_fen(self, fen: str) -> None:
        self._last_processed_fen = fen

    @property
    def latest_full_fen(self) -> str:
        with self._state_lock:
            return self._latest_full_fen

    def on_fen_imported(self, canonical_fen: str) -> None:
        """Handle newly imported FEN: invalidate last processed FEN and store latest canonical FEN."""
        with self._state_lock:
            self._last_processed_fen = ""
            self._latest_full_fen = canonical_fen

    def _handle_board_sync_if_active(self, msg: ChessGameStatus) -> bool:
        """Evaluate board synchronization progress when in AWAITING_BOARD_SYNC state.

        Returns True if event was fully handled or still waiting for physical sync,
        False if board sync just completed on robot's turn (to allow immediate turn eligibility check).
        """
        with self._state_lock:
            if self._get_mission_state() != MacroMissionState.AWAITING_BOARD_SYNC:
                return False

        # If referee is still waiting for board sync or vision is not yet stable, stay in AWAITING_BOARD_SYNC
        if msg.game_phase == ChessGameStatus.PHASE_AWAITING_BOARD_SYNC or not msg.is_board_stable:
            return True

        # Physical board layout confirmed stable and matches imported FEN!
        self._node.get_logger().info(
            f">>> [BOARD SYNC] Physical board verified stable with imported FEN: {msg.full_fen}"
        )

        if self.handle_game_over_if_ended(msg):
            return True

        with self._state_lock:
            current_color = self._robot_color

        if msg.active_color == current_color:
            self._transition_to_fn(MacroMissionState.EVALUATING_BEST_MOVE)
            return False

        self._transition_to_fn(MacroMissionState.WAITING_FOR_PLAYER_MOVE)
        return True

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
        with self._state_lock:
            current_color = self._robot_color
        is_robot_turn = msg.active_color == current_color
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
        """Process incoming referee status messages and gate robot turn eligibility.

        Evaluates game termination conditions (mate/draw), verifies post-move stabilization,
        checks turn ownership against robot color, and triggers move workflow dispatch.

        Args:
            msg: ChessGameStatus message received from chess referee engine.
            clear_pending_recovery_cb: Optional callback to clear recovery goal upon fresh move.

        Thread-safety:
            Checks and updates `_last_processed_fen` under `_state_lock`.
        """
        if msg.robot_color:
            norm_color = msg.robot_color.lower().strip()
            color_changed = False
            with self._state_lock:
                if norm_color != self._robot_color:
                    self._robot_color = norm_color
                    color_changed = True
            if color_changed and self._on_robot_color_updated_cb is not None:
                self._on_robot_color_updated_cb(norm_color)

        if msg.full_fen:
            with self._state_lock:
                self._latest_full_fen = msg.full_fen
            if clear_pending_recovery_cb is not None:
                clear_pending_recovery_cb()

        if self._handle_board_sync_if_active(msg):
            return

        if self.handle_game_over_if_ended(msg):
            return

        if self.verify_post_move_board(msg):
            return

        with self._state_lock:
            current_color = self._robot_color

        if (
            msg.game_phase == ChessGameStatus.PHASE_ROBOT_THINKING
            and current_color
            and msg.active_color == current_color
            and self._get_mission_state() == MacroMissionState.WAITING_FOR_PLAYER_MOVE
        ):
            self._transition_to_fn(MacroMissionState.EVALUATING_BEST_MOVE)
            return

        if not self.is_eligible_robot_turn(msg):
            return

        with self._state_lock:

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

        obs_pose = (
            self._obs_nav.compute_observation_pose_for_base(last_base_pose)
            if self._config.navigation
            else None
        )

        stages = MovePlanBuilder.build_steps(
            resp, details, observation_pose=obs_pose
        )
        if not self._host.transition_to(MacroMissionState.EXECUTING_MOVE_PIPELINE):
            return

        self._pipeline_executor.start_pipeline(stages)

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
        chassis_settle_provider: Callable[[], bool] | None = None,
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
        self._chassis_settle_provider = chassis_settle_provider or (lambda: True)
        self._callback_group = callback_group

        self._timer: Timer | None = None
        self._is_repositioning: bool = False

    @property
    def is_repositioning(self) -> bool:
        """True if actively repositioning the base to a candidate vantage point."""
        with self._state_lock:
            return self._is_repositioning

    @property
    def attempts(self) -> int:
        if self._observation_nav is None:
            return 0
        return self._observation_nav.get_viewpoint_index(
            ObservationIntent.POST_MOVE_VERIFY
        )

    @property
    def is_active(self) -> bool:
        """Return True if watchdog timer is running."""
        return self._timer is not None

    def reset(self) -> None:
        self.stop()
        with self._state_lock:
            self._is_repositioning = False
        if self._observation_nav is not None:
            self._observation_nav.reset_tour(
                ObservationIntent.POST_MOVE_VERIFY
            )

    def _is_post_move_verifying(self) -> bool:
        with self._state_lock:
            return self._get_mission_state() == MacroMissionState.POST_MOVE_VERIFYING

    def _wait_for_chassis_settle(self, timeout_sec: float) -> bool:
        """Wait until chassis has fully settled (velocity within stop thresholds) or timeout expires."""
        if not self._is_post_move_verifying() or timeout_sec <= 0.0:
            return False
        if self._chassis_settle_provider():
            return True

        start_ros = get_node_now_sec(self._node)
        start_wall = time.monotonic()
        poll_interval = min(0.02, timeout_sec / 4.0)

        while self._is_post_move_verifying():
            time.sleep(poll_interval)
            if self._chassis_settle_provider():
                return True
            now_ros = get_node_now_sec(self._node)
            if (now_ros - start_ros >= timeout_sec) or (
                time.monotonic() - start_wall >= timeout_sec
            ):
                break

        return self._is_post_move_verifying() and self._chassis_settle_provider()

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

    def _can_reposition(self) -> bool:
        if not self._config.navigation:
            self._node.get_logger().warn(
                "[POST-MOVE VERIFICATION] Verification timed out and navigation disabled. Finalizing turn."
            )
            return False

        if self._observation_nav.has_exhausted_viewpoints(
            ObservationIntent.POST_MOVE_VERIFY
        ):
            self._node.get_logger().warn(
                "[POST-MOVE VERIFICATION] Exhausted all candidate observation viewpoints. Finalizing turn."
            )
            return False
        return True

    def _on_repositioned(self, res: ActionResult) -> None:
        if res.success:
            self._wait_for_chassis_settle(self._config.chassis_settle_timeout_sec)

        with self._state_lock:
            self._is_repositioning = False
            if self._get_mission_state() != MacroMissionState.POST_MOVE_VERIFYING:
                return

        if res.success:
            self._node.get_logger().info(
                "[ACTIVE PERCEPTION] Reached observation viewpoint and chassis settled. Resuming board scan."
            )
        else:
            self._node.get_logger().warn(
                f"[ACTIVE PERCEPTION] Failed to reach observation viewpoint: {res.message}"
            )
        self._perception.set_context(PerceptionContext.BOARD_STATE_SCAN)
        self.start()

    def _dispatch_repositioning(self) -> None:
        idx = self._observation_nav.get_viewpoint_index(
            ObservationIntent.POST_MOVE_VERIFY
        )
        max_attempts = self._observation_nav.get_max_attempts(
            ObservationIntent.POST_MOVE_VERIFY
        )
        self._node.get_logger().warn(
            f"[ACTIVE VISION] Post-move board occluded/unverified ({self._config.observation_scan_timeout_sec:.1f}s). "
            f"Attempt {idx + 1}/{max_attempts}. Repositioning base to next candidate vantage point..."
        )
        with self._state_lock:
            self._is_repositioning = True
        self._perception.set_context(PerceptionContext.TF_TRACKING_AND_NAV)

        dispatched = self._observation_nav.reposition_to_next_viewpoint(
            intent=ObservationIntent.POST_MOVE_VERIFY,
            timeout_sec=self._config.action_timeout_sec,
            on_completed=self._on_repositioned,
        )
        if not dispatched:
            with self._state_lock:
                self._is_repositioning = False
            self._node.get_logger().warn(
                "[ACTIVE PERCEPTION] Failed to dispatch repositioning goal or viewpoints exhausted! Finalizing turn."
            )
            self._on_finalize_turn()

    def on_timeout(self) -> None:
        """Handle post-move verification timeout by repositioning base to alternative vantage point.

        Called when chessboard FEN cannot be confirmed stable/legal within observation timeout.
        Selects next viewpoint along standoff circle and cycles back to BOARD_STATE_SCAN.
        """
        self.stop()
        with self._state_lock:
            if self._get_mission_state() != MacroMissionState.POST_MOVE_VERIFYING:
                return

        if not self._can_reposition():
            self._on_finalize_turn()
            return

        self._dispatch_repositioning()


__all__ = [
    "TurnHost",
    "GameStatusHandler",
    "MoveWorkflow",
    "PostMoveVerifier",
]

