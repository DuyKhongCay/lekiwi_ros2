# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""
Central Orchestrator Node for the LeKiwi Autonomous Chess Playing Robot.
Coordinates game referee signals, localization gating, and specialized domain subsystems.

Implements Mediator & Facade Design Patterns:
- Delegates move staging & execution to MovePipelineExecutor (move_pipeline.py)
- Delegates vision hardware gating & geometry to PerceptionContextCoordinator (perception_manager.py)
- Delegates action dispatching & active observation to MotionDispatcher (motion_dispatcher.py)
- Delegates TF readiness lease & self-healing to NodeHealthMonitor (health_monitor.py)
"""

from __future__ import annotations

import math
import threading
from typing import Any

import rclpy
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from lekiwi_interfaces.msg import (
    ChessGameStatus,
    ChessMoveDetails,
    PerceptionContext,
)
from lekiwi_interfaces.srv import CheckMoveFeasibility
from lekiwi_orchestrator.fsm import (
    MISSION_STATE_NAMES,
    MacroMissionState,
    MotionExecutionState,
    is_mission_transition_allowed,
)
from lekiwi_orchestrator.health_monitor import (
    HealthMonitorConfig,
    NodeHealthMonitor,
    OrchestratorStateSnapshot,
)
from lekiwi_orchestrator.motion_dispatcher import (
    ActionDispatcherInterface,
    ActionResult,
    ActiveObservationNavigator,
    RosActionDispatcher,
    SimulatedActionDispatcher,
)
from lekiwi_orchestrator.move_pipeline import (
    ChessMoveGoal,
    MovePipelineExecutor,
    StagePipelineBuilder,
)
from lekiwi_orchestrator.perception_manager import (
    PerceptionContextCoordinator,
)
from lekiwi_orchestrator.visualizer import VisualizerConfig


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

        node.declare_parameter("recovery.auto_recovery_enabled", True)
        node.declare_parameter("recovery.auto_recovery_timeout_sec", 5.0)
        node.declare_parameter("recovery.max_recovery_attempts", 3)

        node.declare_parameter("topics.nav_ready", "/system/nav_ready")
        node.declare_parameter("topics.grasp_ready", "/system/grasp_ready")
        node.declare_parameter("topics.game_status", "/chess/game_status")
        node.declare_parameter("topics.perception_context", "/perception_context")
        node.declare_parameter("topics.diagnostics", "/diagnostics")

        node.declare_parameter(
            "services.check_feasibility", "/workspace/check_move_feasibility"
        )
        node.declare_parameter("services.recover", "/orchestrator/recover")
        node.declare_parameter(
            "services.set_perception_context",
            "/orchestrator/set_perception_context",
        )

        node.declare_parameter("actions.navigate_to_pose", "/navigate_to_pose")
        node.declare_parameter(
            "actions.execute_chess_move", "/manipulation/execute_chess_move"
        )

        node.declare_parameter("observation.standoff_distance", 0.65)
        node.declare_parameter("observation.scan_timeout_sec", 6.0)

        node.declare_parameter("visualization.enabled", True)
        node.declare_parameter("visualization.topic", "~/status_markers")
        node.declare_parameter("visualization.hud_z_offset", 0.35)
        node.declare_parameter("visualization.robot_frame", "base_footprint")

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

        self.auto_recovery_enabled = bool(
            node.get_parameter("recovery.auto_recovery_enabled").value
        )
        self.auto_recovery_timeout_sec = float(
            node.get_parameter("recovery.auto_recovery_timeout_sec").value
        )
        self.max_recovery_attempts = int(
            node.get_parameter("recovery.max_recovery_attempts").value
        )

        self.nav_ready_topic = str(node.get_parameter("topics.nav_ready").value)
        self.grasp_ready_topic = str(node.get_parameter("topics.grasp_ready").value)
        self.game_status_topic = str(node.get_parameter("topics.game_status").value)
        self.perception_context_topic = str(
            node.get_parameter("topics.perception_context").value
        )
        self.diagnostics_topic = str(node.get_parameter("topics.diagnostics").value)

        self.check_feasibility_srv = str(
            node.get_parameter("services.check_feasibility").value
        )
        self.recover_srv = str(node.get_parameter("services.recover").value)
        self.set_perception_srv = str(
            node.get_parameter("services.set_perception_context").value
        )

        self.navigate_to_pose_action = str(
            node.get_parameter("actions.navigate_to_pose").value
        )
        self.execute_chess_move_action = str(
            node.get_parameter("actions.execute_chess_move").value
        )

        self.observation_standoff_distance = float(
            node.get_parameter("observation.standoff_distance").value
        )
        self.observation_scan_timeout_sec = float(
            node.get_parameter("observation.scan_timeout_sec").value
        )

        self.visualization_enabled = bool(
            node.get_parameter("visualization.enabled").value
        )
        self.visualization_topic = str(node.get_parameter("visualization.topic").value)
        self.visualization_hud_z_offset = float(
            node.get_parameter("visualization.hud_z_offset").value
        )
        self.visualization_robot_frame = str(
            node.get_parameter("visualization.robot_frame").value
        )


class ChessMissionOrchestrator(Node):
    """High-level mission coordinator implementing Mediator and Facade design patterns."""

    def __init__(
        self,
        node_name: str = "chess_mission_orchestrator",
        action_dispatcher: ActionDispatcherInterface | None = None,
        **kwargs: Any,
    ) -> None:
        super().__init__(node_name, **kwargs)

        self.config = OrchestratorConfig(self)

        # Thread Safety & Level 1 Macro FSM Tracking
        self._state_lock = threading.RLock()
        self._mission_state = MacroMissionState.BOOT_INITIALIZING
        self._current_goal_move: str | None = None
        self._current_move_details: ChessMoveGoal | None = None
        self._last_processed_fen: str | None = None

        # Callback Groups
        self._cb_group_sub = MutuallyExclusiveCallbackGroup()
        self._cb_group_client = ReentrantCallbackGroup()

        # Pillar 1: Perception Subsystem (Context Coordinator & Latched Topics & Service)
        self._perception = PerceptionContextCoordinator(
            node=self,
            perception_context_topic=self.config.perception_context_topic,
            set_perception_service_name=self.config.set_perception_srv,
            callback_group=self._cb_group_client,
        )

        # Pillar 2: Motion Dispatcher & Active Observation Subsystem
        if action_dispatcher is not None:
            self._dispatcher = action_dispatcher
        elif self.config.skip_navigation:
            self._dispatcher = SimulatedActionDispatcher(self)
        else:
            self._dispatcher = RosActionDispatcher(
                self,
                nav2_action_name=self.config.navigate_to_pose_action,
                manipulation_action_name=self.config.execute_chess_move_action,
                check_feasibility_service_name=self.config.check_feasibility_srv,
                callback_group=self._cb_group_client,
            )

        self._observation_nav = ActiveObservationNavigator(
            node=self,
            dispatcher=self._dispatcher,
            map_frame=self.config.map_frame,
            standoff_distance=self.config.observation_standoff_distance,
        )

        # Pillar 3: Move Pipeline Execution Engine (Drives Stages & Level 2 FSM)
        self._pipeline_executor = MovePipelineExecutor(
            node=self,
            dispatcher=self._dispatcher,
            perception=self._perception,
            board_frame=self.config.board_frame,
            action_timeout_sec=self.config.action_timeout_sec,
            on_pipeline_completed=self._on_move_pipeline_completed,
            on_pipeline_failed=self._on_move_pipeline_failed,
            grasp_readiness_provider=lambda: self.is_grasp_ready,
            on_observation_recovery_requested=self.reposition_to_next_observation_viewpoint,
        )

        # Health, Lease, Auto-Recovery & Diagnostics Manager
        health_config = HealthMonitorConfig(
            nav_ready_topic=self.config.nav_ready_topic,
            grasp_ready_topic=self.config.grasp_ready_topic,
            diagnostics_topic=self.config.diagnostics_topic,
            recover_service_name=self.config.recover_srv,
            readiness_timeout_sec=self.config.readiness_timeout_sec,
            auto_recovery_enabled=self.config.auto_recovery_enabled,
            auto_recovery_timeout_sec=self.config.auto_recovery_timeout_sec,
            max_recovery_attempts=self.config.max_recovery_attempts,
            robot_color=self.config.robot_color,
            enable_visualizer=self.config.visualization_enabled,
            visualizer_topic=self.config.visualization_topic,
            visualizer_config=VisualizerConfig(
                robot_frame=self.config.visualization_robot_frame,
                hud_z_offset=self.config.visualization_hud_z_offset,
            ),
        )
        self._health_monitor = NodeHealthMonitor(
            node=self,
            config=health_config,
            state_provider=self._get_orchestrator_snapshot,
            state_lock=self._state_lock,
            cb_group_sub=self._cb_group_sub,
            on_nav_ready_transition_cb=self._on_nav_ready_confirmed,
            on_critical_tf_loss_cb=lambda: self.transition_to(
                MacroMissionState.ERROR_FALLBACK
            ),
            on_auto_recovery_cb=lambda: self.trigger_recovery(
                reason="auto_recovery_timer"
            ),
            on_reset_recovery_system_cb=self.trigger_recovery,
        )

        # ROS 2 Subscriptions
        self._nav_ready_sub = self.create_subscription(
            Bool,
            self.config.nav_ready_topic,
            self._on_nav_ready,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
            callback_group=self._cb_group_sub,
        )
        self._grasp_ready_sub = self.create_subscription(
            Bool,
            self.config.grasp_ready_topic,
            self._on_grasp_ready,
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

        # Recovery Service Server
        self._recover_service = self.create_service(
            Trigger,
            self.config.recover_srv,
            self._handle_recover_service,
            callback_group=self._cb_group_client,
        )

        # Initial State: WAITING_FOR_TF_READY with TF_TRACKING_AND_NAV perception
        self.transition_to(MacroMissionState.WAITING_FOR_TF_READY)
        self._perception.set_context(PerceptionContext.TF_TRACKING_AND_NAV)

        self.get_logger().info(
            f"ChessMissionOrchestrator initialized. Robot Color: '{self.config.robot_color}'. "
            f"Waiting for Navigation Readiness on {self.config.nav_ready_topic} and Grasp Readiness on {self.config.grasp_ready_topic}..."
        )

    def destroy_node(self) -> bool:
        """Clean up active components upon node shutdown."""
        if hasattr(self, "_health_monitor"):
            self._health_monitor.destroy()
        if hasattr(self, "_pipeline_executor"):
            self._pipeline_executor.cancel()
        if hasattr(self, "_perception"):
            self._perception.destroy()
        if hasattr(self, "_dispatcher") and self._dispatcher is not None:
            self._dispatcher.destroy()
        return super().destroy_node()

    def _get_orchestrator_snapshot(self) -> OrchestratorStateSnapshot:
        with self._state_lock:
            stage = (
                self._pipeline_executor.current_stage
                if hasattr(self, "_pipeline_executor")
                else None
            )
            stage_name = stage.name if stage else None
            return OrchestratorStateSnapshot(
                mission_state=self._mission_state,
                perception_context=(
                    self._perception.context if hasattr(self, "_perception") else 0
                ),
                last_goal_move=self._current_goal_move,
                execution_stage=stage_name,
            )

    def _get_diagnostics_state(
        self,
    ) -> tuple[MacroMissionState, int, str | None, str | None]:
        snap = self._get_orchestrator_snapshot()
        return (
            snap.mission_state,
            snap.perception_context,
            snap.last_goal_move,
            snap.execution_stage,
        )

    # ================= Public Properties & Delegations =================

    @property
    def mission_state(self) -> MacroMissionState:
        with self._state_lock:
            return self._mission_state

    @property
    def motion_state(self) -> MotionExecutionState:
        return self._pipeline_executor.motion_state

    @property
    def perception_context(self) -> int:
        return self._perception.context

    @property
    def current_move_details(self) -> ChessMoveGoal | None:
        with self._state_lock:
            return self._current_move_details

    @property
    def is_nav_ready(self) -> bool:
        return self._health_monitor.is_nav_ready

    @property
    def is_grasp_ready(self) -> bool:
        return self._health_monitor.is_grasp_ready

    @property
    def health_monitor(self) -> NodeHealthMonitor:
        return self._health_monitor

    @property
    def dispatcher(self) -> ActionDispatcherInterface:
        return self._dispatcher

    @dispatcher.setter
    def dispatcher(self, d: ActionDispatcherInterface) -> None:
        self._dispatcher = d
        if hasattr(self, "_pipeline_executor") and self._pipeline_executor is not None:
            self._pipeline_executor._dispatcher = d
        if hasattr(self, "_observation_nav") and self._observation_nav is not None:
            self._observation_nav._dispatcher = d

    @property
    def pipeline_executor(self) -> MovePipelineExecutor:
        return self._pipeline_executor

    @property
    def perception_coordinator(self) -> PerceptionContextCoordinator:
        return self._perception

    @property
    def observation_navigator(self) -> ActiveObservationNavigator:
        return self._observation_nav

    # ================= State Machine Management =================

    def transition_to(self, target_state: MacroMissionState) -> bool:
        """Safely transition to Level 1 MacroMissionState validating against legal transition matrix."""
        with self._state_lock:
            if target_state == self._mission_state:
                return True

            if not is_mission_transition_allowed(self._mission_state, target_state):
                old_name = MISSION_STATE_NAMES.get(self._mission_state, "UNKNOWN")
                new_name = MISSION_STATE_NAMES.get(target_state, "UNKNOWN")
                self.get_logger().error(
                    f"ILLEGAL MISSION FSM TRANSITION: Cannot move from {old_name} to {new_name}"
                )
                return False

            old_name = MISSION_STATE_NAMES.get(self._mission_state, "UNKNOWN")
            new_name = MISSION_STATE_NAMES.get(target_state, "UNKNOWN")
            self._mission_state = target_state
            self.get_logger().info(
                f"[MISSION FSM] Transitioned: {old_name} -> {new_name}"
            )

            if target_state in (
                MacroMissionState.WAITING_FOR_PLAYER_MOVE,
                MacroMissionState.TURN_COMPLETED,
            ):
                self._health_monitor.reset_recovery_attempts()

            if target_state == MacroMissionState.ERROR_FALLBACK:
                self._pipeline_executor.cancel()
                self._health_monitor.schedule_auto_recovery_if_enabled()

            return True

    def transition_motion_to(self, target_state: MotionExecutionState) -> bool:
        """Delegate Level 2 MotionExecutionState transition to MovePipelineExecutor."""
        return self._pipeline_executor.transition_motion_to(target_state)

    def set_perception_context(self, requested_context: int) -> bool:
        """Delegate perception context transition to PerceptionContextCoordinator."""
        return self._perception.set_context(requested_context)

    # ================= Self-Healing & Recovery =================

    def trigger_recovery(self, reason: str = "manual") -> bool:
        """Recover from ERROR_FALLBACK safely to WAITING_FOR_TF_READY."""
        with self._state_lock:
            if self._mission_state != MacroMissionState.ERROR_FALLBACK:
                self.get_logger().warn(
                    f"Recovery requested ({reason}), but node is not in ERROR_FALLBACK "
                    f"(current: {MISSION_STATE_NAMES.get(self._mission_state, 'UNKNOWN')})"
                )
                return False

            self._health_monitor.cancel_recovery_timer()
            if hasattr(self, "_dispatcher") and self._dispatcher is not None:
                self._dispatcher.cancel_active_goal()
            self._pipeline_executor.cancel()
            self._current_goal_move = None
            self._last_processed_fen = None

            self.get_logger().info(
                f"[RECOVERY] Resetting system state ({reason}). Transitioning to WAITING_FOR_TF_READY."
            )
            success = self.transition_to(MacroMissionState.WAITING_FOR_TF_READY)
            if success:
                self._perception.set_context(PerceptionContext.TF_TRACKING_AND_NAV)
            return success

    def _handle_recover_service(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        return self._health_monitor.handle_recover_service(request, response)

    # ================= Active Observation / Viewpoint Repositioning =================

    def reposition_to_next_observation_viewpoint(
        self,
        board_x: float = 0.5,
        board_y: float = 0.5,
        board_yaw: float = 0.0,
    ) -> bool:
        """Reposition robot base to next candidate observation standoff when board scan is occluded."""
        self._perception.set_context(PerceptionContext.TF_TRACKING_AND_NAV)

        def _on_repositioned(res: ActionResult) -> None:
            if res.success:
                self.get_logger().info(
                    "[ACTIVE PERCEPTION] Reached observation viewpoint. Resuming board scan."
                )
                self._perception.set_context(PerceptionContext.BOARD_STATE_SCAN)
            else:
                self.get_logger().warn(
                    f"[ACTIVE PERCEPTION] Failed to reach observation viewpoint: {res.message}"
                )

        return self._observation_nav.reposition_to_next_viewpoint(
            board_x=board_x,
            board_y=board_y,
            board_yaw=board_yaw,
            timeout_sec=self.config.action_timeout_sec,
            on_completed=_on_repositioned,
        )

    # ================= Subscription Callbacks =================

    def _on_nav_ready(self, msg: Bool) -> None:
        """Handle navigation readiness notifications."""
        self._health_monitor.on_nav_ready_msg(msg)

    def _on_grasp_ready(self, msg: Bool) -> None:
        """Handle grasp precision readiness notifications."""
        self._health_monitor.on_grasp_ready_msg(msg)

    def _on_nav_ready_confirmed(self, is_ready: bool = True) -> None:
        """Callback invoked by health monitor when Navigation readiness goes True."""
        if is_ready and self.mission_state == MacroMissionState.WAITING_FOR_TF_READY:
            initial_state = (
                MacroMissionState.EVALUATING_BEST_MOVE
                if self.config.robot_color == "w"
                else MacroMissionState.WAITING_FOR_PLAYER_MOVE
            )
            self.transition_to(initial_state)
            self._perception.set_context(PerceptionContext.BOARD_STATE_SCAN)

    def _on_game_status(self, msg: ChessGameStatus) -> None:
        """Process game state updates from lekiwi_chess_master referee."""
        if msg.is_checkmate or msg.is_draw:
            with self._state_lock:
                current_state = self._mission_state
            if current_state != MacroMissionState.GAME_OVER:
                self.transition_to(MacroMissionState.GAME_OVER)
                reason = "CHECKMATE" if msg.is_checkmate else "DRAW"
                self.get_logger().info(
                    f"*** GAME OVER: {reason}! FEN: {msg.full_fen} ***"
                )
                self._perception.set_context(PerceptionContext.IDLE_STANDBY)
            return

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
            return

        with self._state_lock:
            eligible_state = self._mission_state in (
                MacroMissionState.WAITING_FOR_PLAYER_MOVE,
                MacroMissionState.EVALUATING_BEST_MOVE,
                MacroMissionState.TURN_COMPLETED,
            )
            if not eligible_state or msg.full_fen == self._last_processed_fen:
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
        if not self.is_nav_ready:
            self.get_logger().warn(
                f"Cannot dispatch move '{uci_move}': Navigation system not ready. Waiting for navigation readiness."
            )
            self.transition_to(MacroMissionState.WAITING_FOR_TF_READY)
            self._perception.set_context(PerceptionContext.TF_TRACKING_AND_NAV)
            return

        details = ChessMoveGoal.from_uci_or_details(uci_move, move_details)
        if details is None:
            self.get_logger().error(
                f"Failed to parse UCI move '{uci_move}': Expected format like 'e2e4' or 'e7e8q'."
            )
            self.transition_to(MacroMissionState.ERROR_FALLBACK)
            return

        with self._state_lock:
            self._current_move_details = details

        if self._mission_state in (
            MacroMissionState.WAITING_FOR_PLAYER_MOVE,
            MacroMissionState.TURN_COMPLETED,
        ) and not self.transition_to(MacroMissionState.EVALUATING_BEST_MOVE):
            return

        if not self.transition_to(MacroMissionState.CHECKING_REACHABILITY):
            return

        self.get_logger().info(
            f"Querying reachability feasibility for {details.from_square} -> {details.to_square}..."
        )
        dispatched = self._dispatcher.check_feasibility(
            goal=details,
            timeout_sec=self.config.feasibility_timeout_sec,
            on_success=lambda resp: self._on_feasibility_response(resp, details),
            on_error=self._on_feasibility_error,
        )
        if not dispatched:
            self.transition_to(MacroMissionState.ERROR_FALLBACK)

    def _on_feasibility_error(self, error_msg: str) -> None:
        self.get_logger().error(f"Workspace feasibility query failed: {error_msg}")
        self.transition_to(MacroMissionState.ERROR_FALLBACK)

    def _on_feasibility_response(
        self, resp: CheckMoveFeasibility.Response, details: ChessMoveGoal
    ) -> None:
        if not resp.feasible:
            self.get_logger().error(
                f"Move {details.uci} declared NOT FEASIBLE: {resp.message}"
            )
            self.transition_to(MacroMissionState.ERROR_FALLBACK)
            return

        self.get_logger().info(
            f"Feasibility confirmed! Plan Type: {resp.plan_type} ({resp.message})"
        )

        stages = StagePipelineBuilder.build_stages(resp, details)
        if not self.transition_to(MacroMissionState.EXECUTING_MOVE_PIPELINE):
            return

        self._pipeline_executor.start_pipeline(stages, resp)

    def _on_move_pipeline_completed(self) -> None:
        """Conclude robot turn and return to waiting for opponent."""
        self.transition_to(MacroMissionState.TURN_COMPLETED)
        self._perception.set_context(PerceptionContext.BOARD_STATE_SCAN)
        self.transition_to(MacroMissionState.WAITING_FOR_PLAYER_MOVE)
        self.get_logger().info(
            ">>> Turn finalized successfully. Waiting for opponent move."
        )

    def _on_move_pipeline_failed(self, error_msg: str) -> None:
        self.get_logger().error(f"Move pipeline execution failed: {error_msg}")
        self.transition_to(MacroMissionState.ERROR_FALLBACK)

    def publish_diagnostics_snapshot(self) -> None:
        """Delegate diagnostic snapshot to health monitor."""
        snapshot = self._get_orchestrator_snapshot()
        self._health_monitor.publish_diagnostics(snapshot)


def main(args: Any = None) -> None:
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
