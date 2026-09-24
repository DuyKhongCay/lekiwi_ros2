# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""
Central Mission Orchestrator for LeKiwi Mobile Manipulation Chess Robot.

Acts as a Mediator and Facade coordinating:
1. TF readiness & EKF convergence (/system/tf_ready) via selective gating (NodeHealthMonitor)
2. Game status, FIDE legality & Stockfish best move (/chess/game_status) from lekiwi_chess_master
3. Reachability & base standoff queries (/workspace/check_move_feasibility) from lekiwi_motion
4. Standoff trajectory choreography via StagePipelineBuilder
5. Navigation & physical manipulation execution via ActionDispatcher
6. Dynamic operational camera modes published to latched topic (/camera_mode)
7. Self-healing & recovery mechanism (/orchestrator/recover and auto-recovery timer)
"""

from __future__ import annotations

import math
import threading

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from lekiwi_interfaces.msg import CameraMode, ChessGameStatus, ChessMoveDetails
from lekiwi_interfaces.srv import CheckMoveFeasibility
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from lekiwi_orchestrator.action_dispatcher import (
    ActionDispatcherInterface,
    RosActionDispatcher,
    SimulatedActionDispatcher,
)
from lekiwi_orchestrator.fsm import (
    CAMERA_MODE_NAMES,
    MISSION_STATE_NAMES,
    MissionState,
    is_camera_transition_allowed,
    is_mission_transition_allowed,
)
from lekiwi_orchestrator.health_monitor import NodeHealthMonitor
from lekiwi_orchestrator.stage_pipeline_builder import (
    ChessMoveGoal,
    ExecutionStage,
    StagePipelineBuilder,
)

try:
    from lekiwi_interfaces.action import ExecuteChessMove
except ImportError:
    ExecuteChessMove = None


class OrchestratorConfig:
    """Type-safe configuration container parsed from ROS 2 node parameters."""

    def __init__(self, node: Node) -> None:
        node.declare_parameter("robot_color", "b")
        node.declare_parameter("board_frame", "chessboard_frame")
        node.declare_parameter("map_frame", "map")
        node.declare_parameter("feasibility_timeout_sec", 5.0)
        node.declare_parameter("action_timeout_sec", 60.0)
        node.declare_parameter("readiness_timeout_sec", 1.0)
        node.declare_parameter("skip_navigation", False)
        node.declare_parameter("start_navigation", False)

        node.declare_parameter("recovery.auto_recovery_enabled", True)
        node.declare_parameter("recovery.auto_recovery_timeout_sec", 5.0)
        node.declare_parameter("recovery.max_recovery_attempts", 3)

        node.declare_parameter("topics.tf_ready", "/system/tf_ready")
        node.declare_parameter("topics.game_status", "/chess/game_status")
        node.declare_parameter("topics.camera_mode", "/camera_mode")
        node.declare_parameter("topics.diagnostics", "/diagnostics")

        node.declare_parameter(
            "services.check_feasibility", "/workspace/check_move_feasibility"
        )
        node.declare_parameter("services.recover", "/orchestrator/recover")

        node.declare_parameter("actions.navigate_to_pose", "/navigate_to_pose")
        node.declare_parameter(
            "actions.execute_chess_move", "/manipulation/execute_chess_move"
        )

        readiness_to = float(node.get_parameter("readiness_timeout_sec").value)
        if not math.isfinite(readiness_to) or readiness_to <= 0:
            raise ValueError("readiness_timeout_sec must be finite and positive")

        self.robot_color = str(node.get_parameter("robot_color").value).lower()
        self.board_frame = str(node.get_parameter("board_frame").value)
        self.map_frame = str(node.get_parameter("map_frame").value)
        self.feasibility_timeout_sec = float(
            node.get_parameter("feasibility_timeout_sec").value
        )
        self.action_timeout_sec = float(node.get_parameter("action_timeout_sec").value)
        self.readiness_timeout_sec = readiness_to
        self.skip_navigation = bool(node.get_parameter("skip_navigation").value)
        self.start_navigation = bool(node.get_parameter("start_navigation").value)

        self.auto_recovery_enabled = bool(
            node.get_parameter("recovery.auto_recovery_enabled").value
        )
        self.auto_recovery_timeout_sec = float(
            node.get_parameter("recovery.auto_recovery_timeout_sec").value
        )
        self.max_recovery_attempts = int(
            node.get_parameter("recovery.max_recovery_attempts").value
        )

        self.tf_ready_topic = str(node.get_parameter("topics.tf_ready").value)
        self.game_status_topic = str(node.get_parameter("topics.game_status").value)
        self.camera_mode_topic = str(node.get_parameter("topics.camera_mode").value)
        self.diagnostics_topic = str(node.get_parameter("topics.diagnostics").value)

        self.check_feasibility_srv = str(
            node.get_parameter("services.check_feasibility").value
        )
        self.recover_srv = str(node.get_parameter("services.recover").value)

        self.navigate_to_pose_action = str(
            node.get_parameter("actions.navigate_to_pose").value
        )
        self.execute_chess_move_action = str(
            node.get_parameter("actions.execute_chess_move").value
        )


class ChessMissionOrchestrator(Node):
    """High-level mission coordinator implementing Mediator and Facade design patterns."""

    def __init__(
        self,
        node_name: str = "chess_mission_orchestrator",
        action_dispatcher: ActionDispatcherInterface | None = None,
        **kwargs,
    ) -> None:
        super().__init__(node_name, **kwargs)

        self.config = OrchestratorConfig(self)

        # Thread Safety & State Machine Tracking
        self._state_lock = threading.RLock()
        self._mission_state = MissionState.BOOT_INITIALIZING
        self._camera_mode = CameraMode.STANDBY
        self._current_goal_move: str | None = None
        self._current_move_details: ChessMoveGoal | None = None
        self._last_processed_fen: str | None = None
        self._execution_stages: list[ExecutionStage] = []
        self._current_stage: ExecutionStage | None = None
        self._current_feasibility_resp: CheckMoveFeasibility.Response | None = None
        self._dual_base_phase: str | None = None

        # Watchdog Timers
        self._feasibility_timer = None
        self._action_timer = None
        self._action_watchdog_desc: str = ""

        # Callback Groups
        self._cb_group_sub = MutuallyExclusiveCallbackGroup()
        self._cb_group_client = ReentrantCallbackGroup()

        # Action Dispatcher
        if action_dispatcher is not None:
            self._dispatcher = action_dispatcher
        elif self.config.skip_navigation:
            self._dispatcher = SimulatedActionDispatcher(self)
        else:
            self._dispatcher = RosActionDispatcher(
                self,
                nav2_action_name=self.config.navigate_to_pose_action,
                manipulation_action_name=self.config.execute_chess_move_action,
                callback_group=self._cb_group_client,
            )

        # Health, Lease, Auto-Recovery & Diagnostics Manager
        self._health_monitor = NodeHealthMonitor(
            node=self,
            tf_ready_topic=self.config.tf_ready_topic,
            diagnostics_topic=self.config.diagnostics_topic,
            recover_service_name=self.config.recover_srv,
            readiness_timeout_sec=self.config.readiness_timeout_sec,
            auto_recovery_enabled=self.config.auto_recovery_enabled,
            auto_recovery_timeout_sec=self.config.auto_recovery_timeout_sec,
            max_recovery_attempts=self.config.max_recovery_attempts,
            robot_color=self.config.robot_color,
            state_lock=self._state_lock,
            cb_group_sub=self._cb_group_sub,
            on_tf_ready_transition_cb=self._on_tf_ready_transition,
            on_critical_tf_loss_cb=lambda: self.transition_to(
                MissionState.ERROR_FALLBACK
            ),
            on_auto_recovery_cb=lambda: self.trigger_recovery(
                reason="auto_recovery_timer"
            ),
            on_reset_recovery_system_cb=self.trigger_recovery,
        )

        # Subscriptions
        self._tf_ready_sub = self.create_subscription(
            Bool,
            self.config.tf_ready_topic,
            self._on_tf_ready,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
            callback_group=self._cb_group_sub,
        )
        self._game_status_sub = self.create_subscription(
            ChessGameStatus,
            self.config.game_status_topic,
            self._on_game_status,
            10,
            callback_group=self._cb_group_sub,
        )

        # Service Clients & Servers
        self._feasibility_client = self.create_client(
            CheckMoveFeasibility,
            self.config.check_feasibility_srv,
            callback_group=self._cb_group_client,
        )
        self._recover_service = self.create_service(
            Trigger,
            self.config.recover_srv,
            self._handle_recover_service,
            callback_group=self._cb_group_client,
        )

        # Camera Mode Publisher (Latched)
        mode_pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._camera_mode_pub = self.create_publisher(
            CameraMode, self.config.camera_mode_topic, mode_pub_qos
        )

        # Optional Navigation Startup Component
        self._navigation_startup = None
        if self.config.start_navigation:
            from lekiwi_orchestrator.navigation_startup import NavigationStartup

            self._navigation_startup = NavigationStartup(self)

        # Publish initial camera mode
        self._publish_camera_mode(self._camera_mode)

        # Initial State transition to WAITING_FOR_TF_READY
        self.transition_to(MissionState.WAITING_FOR_TF_READY)
        self.get_logger().info(
            f"ChessMissionOrchestrator initialized. Robot Color: '{self.config.robot_color}'. "
            f"Waiting for TF Tree Readiness on {self.config.tf_ready_topic}..."
        )

    def destroy_node(self) -> bool:
        """Clean up active timers and components upon node shutdown."""
        self._cancel_feasibility_timer()
        self._cancel_action_watchdog()
        if hasattr(self, "_health_monitor"):
            self._health_monitor.destroy()
        if hasattr(self, "_dispatcher") and self._dispatcher is not None:
            self._dispatcher.destroy()
        return super().destroy_node()

    # --- Compatibility Properties for Tests & Telemetry ---
    @property
    def mission_state(self) -> MissionState:
        with self._state_lock:
            return self._mission_state

    @property
    def camera_mode(self) -> int:
        with self._state_lock:
            return self._camera_mode

    @property
    def current_move_details(self) -> ChessMoveGoal | None:
        with self._state_lock:
            return self._current_move_details

    @property
    def is_tf_ready(self) -> bool:
        return self._health_monitor.is_tf_ready

    @property
    def _tf_ready(self) -> bool:
        return self._health_monitor.tf_ready

    @_tf_ready.setter
    def _tf_ready(self, val: bool) -> None:
        self._health_monitor.tf_ready = val

    @property
    def _last_readiness_heartbeat(self) -> float | None:
        return self._health_monitor.last_readiness_heartbeat

    @_last_readiness_heartbeat.setter
    def _last_readiness_heartbeat(self, val: float | None) -> None:
        self._health_monitor.last_readiness_heartbeat = val

    @property
    def _recovery_attempts(self) -> int:
        return self._health_monitor.recovery_attempts

    @_recovery_attempts.setter
    def _recovery_attempts(self, val: int) -> None:
        self._health_monitor.recovery_attempts = val

    @property
    def _recovery_timer(self):
        return self._health_monitor._recovery_timer

    @_recovery_timer.setter
    def _recovery_timer(self, val):
        self._health_monitor._recovery_timer = val

    def _on_auto_recovery_timer(self) -> None:
        self._health_monitor.cancel_recovery_timer()
        self.trigger_recovery(reason="auto_recovery_timer")

    def _cancel_recovery_timer(self) -> None:
        self._health_monitor.cancel_recovery_timer()

    # ================= State Machine Management =================

    def transition_to(self, target_state: MissionState) -> bool:
        """Safely transition to target_state validating against legal transition matrix."""
        with self._state_lock:
            if target_state == self._mission_state:
                return True

            if not is_mission_transition_allowed(self._mission_state, target_state):
                err_msg = (
                    f"ILLEGAL FSM TRANSITION: Cannot move from "
                    f"{MISSION_STATE_NAMES.get(self._mission_state, 'UNKNOWN')} to "
                    f"{MISSION_STATE_NAMES.get(target_state, 'UNKNOWN')}"
                )
                self.get_logger().error(err_msg)
                return False

            old_name = MISSION_STATE_NAMES.get(self._mission_state, "UNKNOWN")
            new_name = MISSION_STATE_NAMES.get(target_state, "UNKNOWN")
            self._mission_state = target_state
            self.get_logger().info(
                f"[MISSION FSM] Transitioned: {old_name} -> {new_name}"
            )

            if target_state in (
                MissionState.WAITING_FOR_PLAYER_MOVE,
                MissionState.TURN_COMPLETED,
            ):
                self._health_monitor.reset_recovery_attempts()

            if target_state == MissionState.ERROR_FALLBACK:
                self._health_monitor.schedule_auto_recovery_if_enabled()

            return True

    def set_camera_mode(self, requested_mode: int) -> bool:
        """Update system camera mode and publish directly on latched topic."""
        with self._state_lock:
            current_mode = self._camera_mode
            if requested_mode == current_mode:
                return True

            if not is_camera_transition_allowed(current_mode, requested_mode):
                self.get_logger().warn(
                    f"[CAMERA FSM] Rejected mode switch: "
                    f"{CAMERA_MODE_NAMES.get(current_mode, 'UNKNOWN')} -> "
                    f"{CAMERA_MODE_NAMES.get(requested_mode, 'UNKNOWN')}"
                )
                return False

            self._camera_mode = requested_mode

        self.get_logger().info(
            f"[CAMERA FSM] Switched CameraMode to {CAMERA_MODE_NAMES.get(requested_mode, 'UNKNOWN')}"
        )
        self._publish_camera_mode(requested_mode)
        return True

    def _publish_camera_mode(self, mode_value: int) -> None:
        if hasattr(self, "_camera_mode_pub") and self._camera_mode_pub is not None:
            msg = CameraMode()
            msg.value = mode_value
            self._camera_mode_pub.publish(msg)

    # ================= Self-Healing & Recovery =================

    def trigger_recovery(self, reason: str = "manual") -> bool:
        """Recover from ERROR_FALLBACK safely to WAITING_FOR_TF_READY."""
        with self._state_lock:
            if self._mission_state != MissionState.ERROR_FALLBACK:
                self.get_logger().warn(
                    f"Recovery requested ({reason}), but node is not in ERROR_FALLBACK "
                    f"(current: {MISSION_STATE_NAMES.get(self._mission_state, 'UNKNOWN')})"
                )
                return False

            self._cancel_feasibility_timer()
            self._cancel_action_watchdog()
            self._health_monitor.cancel_recovery_timer()
            self._dispatcher.cancel_active_goal()
            self._execution_stages.clear()
            self._current_stage = None
            self._current_feasibility_resp = None
            self._dual_base_phase = None
            self._current_goal_move = None

            self.get_logger().info(
                f"[RECOVERY] Resetting system state ({reason}). Transitioning to WAITING_FOR_TF_READY."
            )
            success = self.transition_to(MissionState.WAITING_FOR_TF_READY)
            if success:
                self.set_camera_mode(CameraMode.STANDBY)
            return success

    def _handle_recover_service(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        return self._health_monitor.handle_recover_service(
            request, response, self.mission_state
        )

    # ================= Subscription Callbacks =================

    def _expire_readiness(self) -> None:
        """Check lease expiration via health monitor."""
        self._health_monitor.expire_readiness(self.mission_state)

    def _on_tf_ready(self, msg: Bool) -> None:
        """Handle readiness notifications from gatekeeper."""
        self._health_monitor.on_tf_ready_msg(msg, self.mission_state)

    def _on_tf_ready_transition(self, is_ready: bool) -> None:
        """Callback invoked by health monitor when TF readiness goes True."""
        if is_ready and self.mission_state == MissionState.WAITING_FOR_TF_READY:
            initial_state = (
                MissionState.EVALUATING_BEST_MOVE
                if self.config.robot_color == "w"
                else MissionState.WAITING_FOR_PLAYER_MOVE
            )
            self.transition_to(initial_state)
            self.set_camera_mode(CameraMode.CHESS_THINKING)

    def _on_game_status(self, msg: ChessGameStatus) -> None:
        """Process game state updates from lekiwi_chess_master."""
        if self._handle_game_over_if_concluded(msg):
            return

        if self._is_eligible_robot_turn(msg):
            self._dispatch_if_new_board(msg)

    def _handle_game_over_if_concluded(self, msg: ChessGameStatus) -> bool:
        if not (msg.is_checkmate or msg.is_draw):
            return False

        with self._state_lock:
            current_state = self._mission_state
        if current_state != MissionState.GAME_OVER:
            self.transition_to(MissionState.GAME_OVER)
            reason = "CHECKMATE" if msg.is_checkmate else "DRAW"
            self.get_logger().info(f"*** GAME OVER: {reason}! FEN: {msg.full_fen} ***")
            self.set_camera_mode(CameraMode.STANDBY)
        return True

    def _is_eligible_robot_turn(self, msg: ChessGameStatus) -> bool:
        is_robot_turn = msg.active_color == self.config.robot_color
        best_uci = msg.best_move_details.uci
        is_ready = (
            msg.is_board_stable
            and is_robot_turn
            and bool(best_uci)
            and msg.game_phase
            in (
                ChessGameStatus.PHASE_ROBOT_READY,
                ChessGameStatus.PHASE_WAITING_PLAYER,
            )
        )
        if not is_ready:
            return False

        with self._state_lock:
            return self._mission_state in (
                MissionState.WAITING_FOR_PLAYER_MOVE,
                MissionState.EVALUATING_BEST_MOVE,
                MissionState.TURN_COMPLETED,
            )

    def _dispatch_if_new_board(self, msg: ChessGameStatus) -> None:
        with self._state_lock:
            if msg.full_fen == self._last_processed_fen:
                return
            self._last_processed_fen = msg.full_fen
            self._current_goal_move = msg.best_move_details.uci

        details = msg.best_move_details
        self.get_logger().info(
            f">>> [ORCHESTRATOR] New Best Move Received: '{details.uci}' "
            f"(SAN: {details.san}, Piece: {details.piece_type}, "
            f"Capture: {details.is_capture}, Eval: {msg.eval_centipawns} cp, Color: {msg.active_color})"
        )
        self._dispatch_move_workflow(details.uci, move_details=details)

    # ================= Workflow Orchestration =================

    def _dispatch_move_workflow(
        self,
        uci_move: str,
        move_details: ChessMoveDetails | None = None,
    ) -> None:
        """Begin autonomous execution of a single chess move."""
        if not self.is_tf_ready:
            self.get_logger().warn(
                f"Cannot dispatch move '{uci_move}': TF tree not ready. Waiting for TF readiness."
            )
            self.transition_to(MissionState.WAITING_FOR_TF_READY)
            return

        details = self._parse_move_goal(uci_move, move_details)
        if details is None:
            self.transition_to(MissionState.ERROR_FALLBACK)
            return

        with self._state_lock:
            self._current_move_details = details

        if self._mission_state in (
            MissionState.WAITING_FOR_PLAYER_MOVE,
            MissionState.TURN_COMPLETED,
        ) and not self.transition_to(MissionState.EVALUATING_BEST_MOVE):
            return

        if not self.transition_to(MissionState.CHECKING_REACHABILITY):
            return

        if not self._feasibility_client.service_is_ready():
            self.get_logger().error(
                f"Workspace Checker service '{self.config.check_feasibility_srv}' is unavailable!"
            )
            self.transition_to(MissionState.ERROR_FALLBACK)
            return

        self._send_feasibility_query(details)

    def _parse_move_goal(
        self,
        uci_move: str,
        move_details: ChessMoveDetails | None,
    ) -> ChessMoveGoal | None:
        cleaned_move = uci_move.strip().lower()
        if len(cleaned_move) < 4:
            self.get_logger().error(
                f"Failed to parse UCI move '{uci_move}': Expected format like 'e2e4' or 'e7e8q'."
            )
            return None

        if move_details is not None and move_details.from_square:
            return ChessMoveGoal(
                uci=move_details.uci or cleaned_move,
                from_square=move_details.from_square,
                to_square=move_details.to_square,
                promotion=(
                    move_details.promotion_piece
                    if move_details.promotion_piece
                    else None
                ),
                is_capture=move_details.is_capture,
                captured_square=(
                    move_details.captured_square
                    if move_details.captured_square
                    else (move_details.to_square if move_details.is_capture else "")
                ),
                castling_rook_from=move_details.castling_rook_from,
                castling_rook_to=move_details.castling_rook_to,
            )

        from_sq = cleaned_move[:2]
        to_sq = cleaned_move[2:4]
        promo = cleaned_move[4:] if len(cleaned_move) > 4 else None
        return ChessMoveGoal(
            uci=cleaned_move,
            from_square=from_sq,
            to_square=to_sq,
            promotion=promo,
            is_capture=False,
        )

    def _send_feasibility_query(self, details: ChessMoveGoal) -> None:
        req = CheckMoveFeasibility.Request()
        req.move.uci = details.uci
        req.move.from_square = details.from_square
        req.move.to_square = details.to_square
        req.move.is_capture = details.is_capture
        req.move.captured_square = details.captured_square
        if details.promotion:
            req.move.promotion_piece = details.promotion
        if details.castling_rook_from:
            req.move.castling_rook_from = details.castling_rook_from
            req.move.castling_rook_to = details.castling_rook_to
            req.move.is_castling = True

        self.get_logger().info(
            f"Querying reachability feasibility for {details.from_square} -> {details.to_square} "
            f"({'capture at ' + details.captured_square if details.is_capture else 'quiet'})..."
        )

        self._cancel_feasibility_timer()
        self._feasibility_timer = self.create_timer(
            self.config.feasibility_timeout_sec,
            self._on_feasibility_timeout,
            callback_group=self._cb_group_sub,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

        future = self._feasibility_client.call_async(req)
        future.add_done_callback(lambda f: self._on_feasibility_response(f, details))

    def _cancel_feasibility_timer(self) -> None:
        if self._feasibility_timer is not None:
            self._feasibility_timer.cancel()
            self.destroy_timer(self._feasibility_timer)
            self._feasibility_timer = None

    def _on_feasibility_timeout(self) -> None:
        self.get_logger().error(
            f"Workspace feasibility query timed out after {self.config.feasibility_timeout_sec}s!"
        )
        self._cancel_feasibility_timer()
        self.transition_to(MissionState.ERROR_FALLBACK)

    def _on_feasibility_response(self, future, details: ChessMoveGoal) -> None:
        self._cancel_feasibility_timer()
        try:
            resp: CheckMoveFeasibility.Response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Workspace feasibility query failed: {exc}")
            self.transition_to(MissionState.ERROR_FALLBACK)
            return

        if not resp.feasible:
            self.get_logger().error(
                f"Move {details.uci} declared NOT FEASIBLE: {resp.message}"
            )
            self.transition_to(MissionState.ERROR_FALLBACK)
            return

        self.get_logger().info(
            f"Feasibility confirmed! Plan Type: {resp.plan_type} ({resp.message})"
        )

        stages = StagePipelineBuilder.build_stages(resp, details)
        with self._state_lock:
            self._execution_stages = stages
            self._current_feasibility_resp = resp

        self._advance_execution_pipeline()

    # ================= Navigation & Manipulation Execution =================

    def _start_action_watchdog(self, action_name: str) -> None:
        self._cancel_action_watchdog()
        self._action_watchdog_desc = action_name
        self._action_timer = self.create_timer(
            self.config.action_timeout_sec,
            self._on_action_timeout,
            callback_group=self._cb_group_sub,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

    def _cancel_action_watchdog(self) -> None:
        if self._action_timer is not None:
            self._action_timer.cancel()
            self.destroy_timer(self._action_timer)
            self._action_timer = None
        self._action_watchdog_desc = ""

    def _on_action_timeout(self) -> None:
        desc = self._action_watchdog_desc or "Action"
        self.get_logger().error(
            f"{desc} timed out after {self.config.action_timeout_sec}s! Cancelling active goal."
        )
        self._dispatcher.cancel_active_goal()
        self._cancel_action_watchdog()
        self.transition_to(MissionState.ERROR_FALLBACK)

    def _advance_execution_pipeline(self) -> None:
        """Advance to next execution stage or finalize turn."""
        with self._state_lock:
            if not self._execution_stages:
                self._current_stage = None
                self._dual_base_phase = None
                self._finalize_turn()
                return

            stage = self._execution_stages.pop(0)
            self._current_stage = stage
            self._dual_base_phase = stage.name
            feasibility_resp = self._current_feasibility_resp
            details = self._current_move_details

        if stage.target_pose is not None:
            self._execute_navigation_step(
                stage.target_pose, details, feasibility_resp, stage
            )
        else:
            self._execute_manipulation_step(details, feasibility_resp, stage)

    def _execute_navigation_step(
        self,
        target_pose: PoseStamped,
        details: ChessMoveGoal,
        feasibility_resp: CheckMoveFeasibility.Response,
        stage: ExecutionStage | None = None,
    ) -> None:
        if not self.transition_to(MissionState.NAVIGATING_TO_STANDOFF):
            return

        self.set_camera_mode(CameraMode.NAVIGATING)
        stage_name = stage.name if stage else (self._dual_base_phase or "NAV")
        self.get_logger().info(
            f"Dispatching NavigateToPose ({stage_name} standoff) to ({target_pose.pose.position.x:.3f}, "
            f"{target_pose.pose.position.y:.3f})..."
        )

        self._start_action_watchdog(f"Nav2 navigation ({stage_name} standoff)")

        success = self._dispatcher.send_navigation_goal(
            target_pose=target_pose,
            on_accepted=lambda handle: self._on_nav2_goal_submitted(
                handle, details, feasibility_resp
            ),
            on_completed=lambda fut: self._on_nav2_completed(
                fut, details, feasibility_resp, stage
            ),
        )
        if not success:
            self._cancel_action_watchdog()
            self.transition_to(MissionState.ERROR_FALLBACK)

    def _on_nav2_goal_submitted(self, goal_handle, details, feasibility_resp) -> None:
        if goal_handle is None:
            self._cancel_action_watchdog()
            self.get_logger().error("Nav2 rejected navigation goal!")
            self.transition_to(MissionState.ERROR_FALLBACK)
            return
        self.get_logger().info("Nav2 navigation goal accepted.")

    def _on_nav2_completed(self, future, details, feasibility_resp, stage) -> None:
        self._cancel_action_watchdog()
        stage_name = stage.name if stage else "target"
        self.get_logger().info(
            f"Nav2 navigation for stage '{stage_name}' completed successfully! Commencing manipulation."
        )
        self._execute_manipulation_step(details, feasibility_resp, stage)

    def _execute_manipulation_step(
        self,
        details: ChessMoveGoal,
        feasibility_resp: CheckMoveFeasibility.Response,
        stage: ExecutionStage | None = None,
    ) -> None:
        if not self.transition_to(MissionState.EXECUTING_MANIPULATION):
            return

        self.set_camera_mode(CameraMode.MANIPULATION_LEROBOT)

        goal = self._build_manipulation_goal(details, feasibility_resp, stage)
        self._start_action_watchdog(f"Manipulation goal: '{goal.instruction}'")

        self.get_logger().info(
            f"Sending ExecuteChessMove goal: '{goal.instruction}' (capture={goal.is_capture})..."
        )
        success = self._dispatcher.send_manipulation_goal(
            goal=goal,
            on_feedback=self._on_manipulation_feedback,
            on_accepted=lambda handle: self._on_manipulation_goal_submitted(
                handle, details, feasibility_resp
            ),
            on_completed=lambda fut: self._on_manipulation_completed(
                fut, details, feasibility_resp
            ),
        )
        if not success:
            self._cancel_action_watchdog()
            self.transition_to(MissionState.ERROR_FALLBACK)

    def _build_manipulation_goal(
        self,
        details: ChessMoveGoal,
        feasibility_resp: CheckMoveFeasibility.Response,
        stage: ExecutionStage | None,
    ) -> ExecuteChessMove.Goal:
        goal = ExecuteChessMove.Goal()
        cur_stage = stage or self._current_stage
        if cur_stage is not None:
            goal.instruction = cur_stage.instruction
            goal.from_square = cur_stage.from_square
            goal.to_square = cur_stage.to_square
            goal.is_capture = cur_stage.is_capture
            goal.pick_point = cur_stage.pick_point
            goal.place_point = cur_stage.place_point
        else:
            goal.instruction = f"Pick {details.from_square}, place {details.to_square}"
            goal.from_square = details.from_square
            goal.to_square = details.to_square
            goal.is_capture = details.is_capture
            goal.pick_point = getattr(feasibility_resp, "pick_point", Point())
            goal.place_point = getattr(feasibility_resp, "place_point", Point())

        goal.target_frame = self.config.board_frame
        if hasattr(feasibility_resp, "pick_ik_solution") and getattr(
            feasibility_resp.pick_ik_solution, "name", None
        ):
            goal.pick_ik_hint = feasibility_resp.pick_ik_solution
        if hasattr(feasibility_resp, "place_ik_solution") and getattr(
            feasibility_resp.place_ik_solution, "name", None
        ):
            goal.place_ik_hint = feasibility_resp.place_ik_solution
        return goal

    def _on_manipulation_feedback(self, feedback_msg) -> None:
        fb = getattr(feedback_msg, "feedback", feedback_msg)
        phase = getattr(fb, "current_phase", "EXEC")
        progress = getattr(fb, "progress_percent", 0.0)
        self.get_logger().info(
            f"[MANIPULATION FEEDBACK] Phase: {phase} ({progress:.1f}%)"
        )

    def _on_manipulation_goal_submitted(
        self, goal_handle, details, feasibility_resp
    ) -> None:
        if goal_handle is None:
            self._cancel_action_watchdog()
            self.get_logger().error("Manipulation server rejected chess move goal!")
            self.transition_to(MissionState.ERROR_FALLBACK)
            return
        self.get_logger().info("Manipulation goal accepted.")

    def _on_manipulation_completed(self, future, details, feasibility_resp) -> None:
        self._cancel_action_watchdog()
        try:
            res_obj = future.result() if hasattr(future, "result") else future
            result = getattr(res_obj, "result", res_obj)
            if getattr(result, "success", False):
                exec_time = getattr(result, "execution_time_sec", 0.0)
                msg = getattr(result, "message", "OK")
                self.get_logger().info(
                    f"Manipulation execution successful in {exec_time:.2f}s: {msg}"
                )
                self._advance_execution_pipeline()
            else:
                msg = getattr(result, "message", "unknown failure")
                self.get_logger().error(f"Manipulation execution failed: {msg}")
                self.transition_to(MissionState.ERROR_FALLBACK)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Error reading manipulation result: {exc}")
            self.transition_to(MissionState.ERROR_FALLBACK)

    def _finalize_turn(self) -> None:
        """Conclude robot turn and return to waiting for opponent."""
        with self._state_lock:
            self._execution_stages.clear()
            self._current_stage = None
            self._current_feasibility_resp = None
            self._dual_base_phase = None
        self.transition_to(MissionState.TURN_COMPLETED)
        self.set_camera_mode(CameraMode.CHESS_THINKING)
        self.transition_to(MissionState.WAITING_FOR_PLAYER_MOVE)
        self.get_logger().info(
            ">>> Turn finalized successfully. Waiting for opponent move."
        )

    def publish_diagnostics_snapshot(self) -> None:
        """Delegate diagnostic snapshot to health monitor."""
        with self._state_lock:
            state = self._mission_state
            cam_mode = self._camera_mode
            goal = self._current_goal_move
            phase = self._dual_base_phase
        self._health_monitor.publish_diagnostics(state, cam_mode, goal, phase)


def main(args=None):
    rclpy.init(args=args)
    node = ChessMissionOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
