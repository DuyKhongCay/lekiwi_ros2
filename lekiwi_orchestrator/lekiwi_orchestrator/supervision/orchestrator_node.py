# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""
Central Orchestrator Node for the LeKiwi Autonomous Chess Playing Robot.
Coordinates game referee signals, localization gating, and specialized domain subsystems.

Implements Mediator & Facade Design Patterns:
- Delegates move staging & execution to MovePipelineExecutor (executor.py)
- Delegates vision hardware gating & geometry to PerceptionContextCoordinator (perception/)
- Delegates action dispatching & active observation to MotionDispatcher (motion/)
- Delegates TF readiness lease & self-healing to NodeHealthMonitor (monitoring/)
- Delegates post-move verification, referee signals, and workflow execution to turn_workflow.py
"""

from __future__ import annotations

import json
import threading
from dataclasses import dataclass
from typing import Any

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from lekiwi_interfaces.msg import (
    ChessGameStatus,
    ChessMoveDetails,
    PerceptionContext,
)
from lekiwi_interfaces.srv import (
    CheckMoveFeasibility,
    ExecuteCollectionTask,
    SetFen,
)
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Empty, Trigger

from lekiwi_orchestrator.core.fsm import (
    IMPORT_FEN_ALLOWED_STATES,
    MISSION_STATE_NAMES,
    MacroMissionState,
    MotionExecutionState,
    is_mission_transition_allowed,
)
from lekiwi_orchestrator.core.mission_types import ActionResult, ChessMoveGoal, ObservationIntent
from lekiwi_orchestrator.core.parameters import OrchestratorParameters
from lekiwi_orchestrator.execution.engine_client import EngineClient
from lekiwi_orchestrator.execution.motion_client import (
    MotionClient,
    RosMotionClient,
)
from lekiwi_orchestrator.execution.move_sequencer import MoveSequencer
from lekiwi_orchestrator.execution.turn_workflow import (
    GameStatusHandler,
    MoveWorkflow,
    PostMoveVerifier,
    TurnHost,
)
from lekiwi_orchestrator.planning.obs_navigator import ObsNavigator
from lekiwi_orchestrator.supervision.health_supervisor import (
    HealthSupervisor,
    HealthSupervisorConfig,
    OrchestratorStateSnapshot,
    VisualizerConfig,
)
from lekiwi_orchestrator.supervision.perception_context import PerceptionContextManager


@dataclass(frozen=True)
class OrchestratorSubsystems:
    """Container holding wired orchestrator subsystems."""

    perception: PerceptionContextManager
    motion_client: MotionClient
    obs_navigator: ObsNavigator
    move_sequencer: MoveSequencer
    health_supervisor: HealthSupervisor
    post_move_verifier: PostMoveVerifier
    game_status_handler: GameStatusHandler
    move_workflow: MoveWorkflow
    engine_client: EngineClient


class ChessMissionOrchestrator(Node):
    """High-level mission coordinator implementing Mediator and Facade design patterns."""

    def __init__(
        self,
        node_name: str = "chess_mission_orchestrator",
        motion_client: MotionClient | None = None,
        tf_buffer: tf2_ros.Buffer | None = None,
        **kwargs: Any,
    ) -> None:
        """Initialize ChessMissionOrchestrator node, subsystems, and ROS communications."""
        super().__init__(node_name, **kwargs)

        self.parameters = OrchestratorParameters(self)

        # Thread Safety & Level 1 Macro FSM Tracking
        self._state_lock = threading.RLock()
        self._mission_state = MacroMissionState.BOOT_INITIALIZING
        self._current_goal_move: str | None = None
        self._current_move_details: ChessMoveGoal | None = None
        self._pending_recovery_move: ChessMoveGoal | None = None
        self._last_interaction_pose: PoseStamped | None = None

        # TF2 Buffer and TransformListener
        if tf_buffer is not None:
            self._tf_buffer = tf_buffer
            self._tf_listener = None
        else:
            self._tf_buffer = tf2_ros.Buffer()
            if motion_client is None or isinstance(motion_client, RosMotionClient):
                self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
            else:
                self._tf_listener = None
                self._seed_mock_transforms(self._tf_buffer)

        # Callback Groups
        self._cb_group_sub = MutuallyExclusiveCallbackGroup()
        self._cb_group_client = ReentrantCallbackGroup()

        # Wire Subsystems via Dependency Injection
        self._subsystems = self._wire_subsystems(motion_client)
        self._perception = self._subsystems.perception
        self._motion_client = self._subsystems.motion_client
        self._obs_navigator = self._subsystems.obs_navigator
        self._move_sequencer = self._subsystems.move_sequencer
        self._health_supervisor = self._subsystems.health_supervisor
        self._post_move_verifier = self._subsystems.post_move_verifier
        self._game_status_handler = self._subsystems.game_status_handler
        self._move_workflow = self._subsystems.move_workflow
        self._engine_client = self._subsystems.engine_client

        # ROS 2 Subscriptions & Services
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        game_status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._nav_ready_sub = self.create_subscription(
            Bool,
            self.parameters.nav_ready_topic,
            self._on_nav_ready,
            latched_qos,
            callback_group=self._cb_group_sub,
        )
        self._grasp_ready_sub = self.create_subscription(
            Bool,
            self.parameters.grasp_ready_topic,
            self._on_grasp_ready,
            latched_qos,
            callback_group=self._cb_group_sub,
        )
        self._game_status_sub = self.create_subscription(
            ChessGameStatus,
            self.parameters.game_status_topic,
            self._on_game_status,
            game_status_qos,
            callback_group=self._cb_group_sub,
        )
        self._candidate_move_pub = self.create_publisher(
            ChessMoveDetails,
            self.parameters.candidate_move_topic,
            latched_qos,
        )
        self._recover_service = self.create_service(
            Trigger,
            self.parameters.recover_srv,
            self._handle_recover_service,
            callback_group=self._cb_group_client,
        )
        self._import_fen_service = self.create_service(
            SetFen,
            self.parameters.import_fen_srv,
            self._handle_import_fen_service,
            callback_group=self._cb_group_client,
        )
        self._export_fen_service = self.create_service(
            Trigger,
            self.parameters.export_fen_srv,
            self._handle_export_fen_service,
            callback_group=self._cb_group_client,
        )
        self._execute_collection_task_service = self.create_service(
            ExecuteCollectionTask,
            self.parameters.execute_collection_task_srv,
            self._handle_execute_collection_task_service,
            callback_group=self._cb_group_client,
        )

        # Service clients for system, EKF reset, and chess engine FEN configuration
        self._system_reset_client = self.create_client(
            Trigger,
            self.parameters.system_reset_readiness_srv,
            callback_group=self._cb_group_client,
        )
        self._ekf_reset_client = self.create_client(
            Empty,
            self.parameters.ekf_reset_srv,
            callback_group=self._cb_group_client,
        )
        self._chess_set_fen_client = self.create_client(
            SetFen,
            self.parameters.chess_set_fen_srv,
            callback_group=self._cb_group_client,
        )

        # Initial State: WAITING_FOR_TF_READY with TF_TRACKING_AND_NAV perception
        self.transition_to(MacroMissionState.WAITING_FOR_TF_READY)
        self._perception.set_context(PerceptionContext.TF_TRACKING_AND_NAV)

        self.get_logger().info(
            "ChessMissionOrchestrator initialized. "
            f"Waiting for Navigation Readiness on {self.parameters.nav_ready_topic} and Grasp Readiness on {self.parameters.grasp_ready_topic}..."
        )

    def _seed_mock_transforms(self, buffer: tf2_ros.Buffer) -> None:
        """Seed mock TF buffer with canonical baseline transforms for offline testing."""
        from geometry_msgs.msg import TransformStamped

        t_map_board = TransformStamped()
        t_map_board.header.frame_id = self.parameters.map_frame
        t_map_board.child_frame_id = self.parameters.board_frame
        t_map_board.transform.rotation.w = 1.0

        t_base_board = TransformStamped()
        t_base_board.header.frame_id = "base_footprint"
        t_base_board.child_frame_id = self.parameters.board_frame
        t_base_board.transform.translation.x = 0.40
        t_base_board.transform.translation.y = 0.18
        t_base_board.transform.translation.z = 0.01
        t_base_board.transform.rotation.w = 1.0

        t_map_base = TransformStamped()
        t_map_base.header.frame_id = self.parameters.map_frame
        t_map_base.child_frame_id = "base_footprint"
        t_map_base.transform.translation.x = -0.40
        t_map_base.transform.rotation.w = 1.0

        buffer.set_transform_static(t_map_board, "mock_authority")
        buffer.set_transform_static(t_base_board, "mock_authority")
        buffer.set_transform_static(t_map_base, "mock_authority")

    def _wire_subsystems(
        self, motion_client: MotionClient | None = None
    ) -> OrchestratorSubsystems:
        """Compose and wire orchestrator subsystems via dependency injection.

        Instantiates Perception, Motion, Observation, Sequencer, Health, and Workflow coordinators.
        Assigns asynchronous clients to `ReentrantCallbackGroup` to prevent deadlock during execution.

        Args:
            motion_client: Optional pre-configured motion client instance (e.g. for unit testing).

        Returns:
            OrchestratorSubsystems container holding wired subsystem references.
        """
        config = self.parameters

        # 1. Perception: Wire camera context gating and latched context publisher
        perception = PerceptionContextManager(
            node=self,
            perception_context_topic=config.perception_context_topic,
            set_perception_service_name=config.set_perception_srv,
            callback_group=self._cb_group_client,
        )

        # 2. Motion & Observation
        motion_client = motion_client or RosMotionClient(
            self,
            nav2_action_name=config.navigate_to_pose_action,
            manipulation_action_name=config.execute_chess_move_action,
            check_feasibility_service_name=config.check_feasibility_srv,
            callback_group=self._cb_group_client,
            mock_nav2=not config.navigation,
        )

        angle_offsets_map = {
            ObservationIntent.RELOCALIZE: config.observation_angle_offsets_relocalize,
            ObservationIntent.POST_MOVE_VERIFY: config.observation_angle_offsets_post_move_verify,
        }
        obs_navigator = ObsNavigator(
            node=self,
            dispatcher=motion_client,
            map_frame=config.map_frame,
            board_frame=config.board_frame,
            standoff_distance=config.observation_standoff_distance,
            angle_offsets_map=angle_offsets_map,
            tf_buffer=getattr(self, "_tf_buffer", None),
        )

        # 3. Health Supervisor
        health_config = HealthSupervisorConfig(
            nav_ready_topic=config.nav_ready_topic,
            grasp_ready_topic=config.grasp_ready_topic,
            diagnostics_topic=config.diagnostics_topic,
            recover_service_name=config.recover_srv,
            readiness_timeout_sec=config.readiness_timeout_sec,
            auto_recovery_enabled=config.auto_recovery_enabled,
            auto_recovery_timeout_sec=config.auto_recovery_timeout_sec,
            max_recovery_attempts=config.max_recovery_attempts,
            enable_visualizer=config.visualization_enabled,
            visualizer_topic=config.visualization_topic,
            visualizer_config=VisualizerConfig(
                robot_frame=config.visualization_robot_frame,
                hud_z_offset=config.visualization_hud_z_offset,
            ),
            local_odom_topic=config.disturbance_local_odom_topic,
            global_odom_topic=config.disturbance_global_odom_topic,
            uncommanded_linear_thresh=config.disturbance_uncommanded_linear_thresh,
            uncommanded_angular_thresh=config.disturbance_uncommanded_angular_thresh,
            uncommanded_confirm_duration_sec=config.disturbance_uncommanded_confirm_duration_sec,
        )
        health_supervisor = HealthSupervisor(
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
            on_disturbance_detected_cb=self._on_disturbance_detected,
        )

        # 4. Move Sequencer
        move_sequencer = MoveSequencer(
            node=self,
            dispatcher=motion_client,
            perception=perception,
            board_frame=config.board_frame,
            action_timeout_sec=config.action_timeout_sec,
            on_pipeline_completed=self._on_move_pipeline_completed,
            on_pipeline_failed=self._on_move_pipeline_failed,
            grasp_readiness_provider=lambda: health_supervisor.is_grasp_ready,
            chassis_settle_timeout_sec=config.chassis_settle_timeout_sec,
            chassis_settle_provider=lambda: health_supervisor.is_chassis_stationary,
            navigation_enabled=config.navigation,
            callback_group=self._cb_group_client,
            observation_navigator=obs_navigator,
            tf_buffer=self._tf_buffer,
        )

        # 5. Post-Move Verifier
        post_move_verifier = PostMoveVerifier(
            node=self,
            config=config,
            observation_nav=obs_navigator,
            perception=perception,
            state_lock=self._state_lock,
            get_mission_state=lambda: self.mission_state,
            get_last_interaction_pose=lambda: self._last_interaction_pose,
            on_finalize_turn=self._finalize_turn_completed,
            chassis_settle_provider=lambda: health_supervisor.is_chassis_stationary,
            callback_group=self._cb_group_client,
        )

        # 6. Engine Client
        engine_client = EngineClient(
            self,
            action_name=config.chess_compute_best_move_action,
            callback_group=self._cb_group_client,
        )

        # 7. Game Status Handler
        game_status_handler = GameStatusHandler(
            node=self,
            config=config,
            perception=perception,
            state_lock=self._state_lock,
            get_mission_state=lambda: self.mission_state,
            transition_to_fn=self.transition_to,
            finalize_turn_fn=self._finalize_turn_completed,
            dispatch_workflow_fn=self._dispatch_move_workflow,
            reset_obs_viewpoints_fn=obs_navigator.reset_tour,
            engine_client=engine_client,
            on_robot_color_updated_cb=health_supervisor.set_robot_color,
        )

        # 8. Move Workflow
        move_workflow = MoveWorkflow(
            node=self,
            config=config,
            host=self,
            dispatcher_provider=lambda: self._motion_client,
            observation_nav=obs_navigator,
            pipeline_executor=move_sequencer,
            perception=perception,
        )

        return OrchestratorSubsystems(
            perception=perception,
            motion_client=motion_client,
            obs_navigator=obs_navigator,
            move_sequencer=move_sequencer,
            health_supervisor=health_supervisor,
            post_move_verifier=post_move_verifier,
            game_status_handler=game_status_handler,
            move_workflow=move_workflow,
            engine_client=engine_client,
        )


    def destroy_node(self) -> bool:
        """Clean up active components upon node shutdown."""
        self._post_move_verifier.stop()
        self._health_supervisor.destroy()
        self._move_sequencer.destroy()
        self._perception.destroy()
        if self._engine_client is not None:
            self._engine_client.destroy()
            self._engine_client = None
        if self._system_reset_client is not None:
            self.destroy_client(self._system_reset_client)
            self._system_reset_client = None
        if self._ekf_reset_client is not None:
            self.destroy_client(self._ekf_reset_client)
            self._ekf_reset_client = None
        if self._chess_set_fen_client is not None:
            self.destroy_client(self._chess_set_fen_client)
            self._chess_set_fen_client = None
        if self._candidate_move_pub is not None:
            self.destroy_publisher(self._candidate_move_pub)
            self._candidate_move_pub = None
        if self._motion_client is not None:
            self._motion_client.destroy()
        return super().destroy_node()

    def _get_orchestrator_snapshot(self) -> OrchestratorStateSnapshot:
        """Capture thread-safe snapshot of orchestrator states for telemetry and health."""
        with self._state_lock:
            stage = self._move_sequencer.current_stage
            stage_name = stage.name if stage else None
            motion_st = self._move_sequencer.motion_state
            is_nav = (
                self._motion_client.is_navigating
                if self._motion_client is not None
                else False
            )

            if (
                hasattr(self, "_post_move_verifier")
                and self._post_move_verifier.is_repositioning
            ):
                motion_st = MotionExecutionState.NAV_TO_OBS
                is_nav = True

            last_goal = self._current_goal_move or (
                self._pending_recovery_move.uci if self._pending_recovery_move else None
            )
            return OrchestratorStateSnapshot(
                mission_state=self._mission_state,
                perception_context=self._perception.context,
                last_goal_move=last_goal,
                execution_stage=stage_name,
                motion_state=motion_st,
                is_navigating=is_nav,
            )

    # ================= Public Properties & Delegations =================

    @property
    def mission_state(self) -> MacroMissionState:
        """Current MacroMissionState of the orchestrator."""
        with self._state_lock:
            return self._mission_state

    @property
    def motion_state(self) -> MotionExecutionState:
        """Current MotionExecutionState of the move sequencer."""
        return self._move_sequencer.motion_state

    @property
    def perception_context(self) -> int:
        """Current active camera perception context."""
        return self._perception.context

    @property
    def current_move_details(self) -> ChessMoveGoal | None:
        """Active chess move goal details if currently executing."""
        with self._state_lock:
            return self._current_move_details

    @property
    def pending_recovery_move(self) -> ChessMoveGoal | None:
        """Interrupted chess move goal pending recovery resumption."""
        with self._state_lock:
            return self._pending_recovery_move

    @property
    def is_nav_ready(self) -> bool:
        """True if navigation subsystem reports ready."""
        return self._health_supervisor.is_nav_ready

    @property
    def is_grasp_ready(self) -> bool:
        """True if grasping subsystem reports ready."""
        return self._health_supervisor.is_grasp_ready

    @property
    def health_supervisor(self) -> HealthSupervisor:
        """Reference to HealthSupervisor subsystem."""
        return self._health_supervisor

    @property
    def motion_client(self) -> MotionClient:
        """Reference to MotionClient subsystem."""
        return self._motion_client

    @property
    def move_sequencer(self) -> MoveSequencer:
        """Reference to MoveSequencer subsystem."""
        return self._move_sequencer

    @property
    def perception_manager(self) -> PerceptionContextManager:
        """Reference to PerceptionContextManager subsystem."""
        return self._perception

    @property
    def obs_navigator(self) -> ObsNavigator:
        """Reference to ObsNavigator subsystem."""
        return self._obs_navigator

    @property
    def game_status_handler(self) -> GameStatusHandler:
        """Reference to GameStatusHandler subsystem."""
        return self._game_status_handler

    @property
    def move_workflow(self) -> MoveWorkflow:
        """Reference to MoveWorkflow subsystem."""
        return self._move_workflow

    @property
    def post_move_verifier(self) -> PostMoveVerifier:
        """Reference to PostMoveVerifier subsystem."""
        return self._post_move_verifier

    @property
    def engine_client(self) -> EngineClient:
        """Reference to EngineClient subsystem."""
        return self._engine_client


    # ================= State Machine Management =================

    def transition_to(self, target_state: MacroMissionState) -> bool:
        """Safely transition to Level 1 MacroMissionState validating against legal transition matrix.

        Enforces legal transition rules, manages verification watchdogs, and coordinates
        error pipeline cancellations and auto-recovery triggers.

        Args:
            target_state: Target MacroMissionState enum value to enter.

        Returns:
            True if transition was permitted and applied, False if rejected by matrix.

        Thread-safety:
            Acquires `_state_lock` during FSM evaluation. Pipeline cancellation and
            recovery timer scheduling are executed outside the lock to prevent deadlocks.
        """
        cancel_pipeline = False
        schedule_recovery = False
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

            if target_state != MacroMissionState.POST_MOVE_VERIFYING:
                self._post_move_verifier.stop()

            if target_state in (
                MacroMissionState.WAITING_FOR_PLAYER_MOVE,
                MacroMissionState.TURN_COMPLETED,
            ):
                self._health_supervisor.reset_recovery_attempts()

            if target_state == MacroMissionState.ERROR_FALLBACK:
                cancel_pipeline = True
                schedule_recovery = True

        # Defer cancellation and recovery scheduling outside lock to avoid deadlocks
        if cancel_pipeline:
            self._move_sequencer.cancel()
        if schedule_recovery:
            self._health_supervisor.schedule_auto_recovery_if_enabled()

        return True

    def transition_motion_to(self, target_state: MotionExecutionState) -> bool:
        """Delegate Level 2 MotionExecutionState transition to MoveSequencer."""
        return self._move_sequencer.transition_motion_to(target_state)

    def set_perception_context(self, requested_context: int) -> bool:
        """Delegate perception context transition to PerceptionContextManager."""
        return self._perception.set_context(requested_context)

    # ================= Self-Healing & Recovery =================

    def _preserve_interrupted_move_locked(self, log_tag: str, log_suffix: str) -> None:
        """Preserve active move goal under state lock for recovery resumption."""
        if self._pending_recovery_move is None:
            if self._current_move_details is not None:
                self._pending_recovery_move = self._current_move_details
            elif self._current_goal_move:
                self._pending_recovery_move = ChessMoveGoal.from_uci_or_details(
                    self._current_goal_move
                )
        if self._pending_recovery_move is not None:
            self.get_logger().info(
                f"[{log_tag}] Preserved interrupted move '{self._pending_recovery_move.uci}' {log_suffix}"
            )

    def _on_disturbance_detected(self, reason: str) -> None:
        """Handle physical chassis disturbance detected by HealthSupervisor."""
        self.get_logger().warn(
            f"[DISTURBANCE INTERRUPT] Detected disturbance ({reason})! Aborting active action goals."
        )
        self._health_supervisor.set_operator_latched(True)
        self._health_supervisor.cancel_recovery_timer()

        with self._state_lock:
            self._preserve_interrupted_move_locked(
                "DISTURBANCE", "for resumption after manual recovery."
            )

        if self._motion_client is not None:
            self._motion_client.cancel_active_goal()
        self._move_sequencer.cancel()
        self.transition_to(MacroMissionState.ERROR_FALLBACK)

    def _call_reset_services(self) -> None:
        """Invoke /system/reset_readiness and /ekf_filter_node_map/reset asynchronously."""
        if self._system_reset_client is not None:
            if self._system_reset_client.service_is_ready():
                req = Trigger.Request()
                self._system_reset_client.call_async(req)
            else:
                self.get_logger().warn(
                    f"[RECOVERY] Reset readiness service '{self.parameters.system_reset_readiness_srv}' not available yet"
                )

        if self._ekf_reset_client is not None:
            if self._ekf_reset_client.service_is_ready():
                req = Empty.Request()
                self._ekf_reset_client.call_async(req)
            else:
                self.get_logger().warn(
                    f"[RECOVERY] EKF reset service '{self.parameters.ekf_reset_srv}' not available yet"
                )

    def trigger_recovery(self, reason: str = "manual") -> bool:
        """Recover from any state safely to WAITING_FOR_TF_READY.

        Preserves any interrupted move goal for automatic resumption once navigation recovers.
        Resets motion client goals, pipeline sequencers, and camera context.
        Calls system readiness and EKF filter reset services asynchronously.

        Args:
            reason: Diagnostic string explaining why recovery was triggered.

        Returns:
            True if system successfully transitioned to WAITING_FOR_TF_READY, False otherwise.
        """
        with self._state_lock:
            self._preserve_interrupted_move_locked(
                "RECOVERY", "for automatic resumption upon nav ready."
            )

        self._health_supervisor.cancel_recovery_timer()
        self._health_supervisor.set_operator_latched(False)
        if self._motion_client is not None:
            self._motion_client.cancel_active_goal()
        self._move_sequencer.cancel()

        self.get_logger().info(
            f"[RECOVERY] Resetting system state ({reason}). Transitioning to WAITING_FOR_TF_READY."
        )
        self._call_reset_services()
        self._perception.set_context(PerceptionContext.TF_TRACKING_AND_NAV)
        success = self.transition_to(MacroMissionState.WAITING_FOR_TF_READY)
        return success

    def _handle_recover_service(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Delegate recovery service request to HealthSupervisor."""
        return self._health_supervisor.handle_recover_service(request, response)

    def _handle_export_fen_service(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Export current 6-field canonical FEN string and robot color as JSON."""
        fen = self._game_status_handler.latest_full_fen
        if not fen:
            response.success = False
            response.message = "No FEN available (no game status received yet)"
            return response
        robot_color = self._game_status_handler.robot_color or "unknown"
        response.success = True
        response.message = json.dumps({"fen": fen, "robot_color": robot_color})
        return response

    def publish_candidate_move(
        self, details: ChessMoveGoal | ChessMoveDetails | None
    ) -> None:
        """Publish candidate move goal to /chess/candidate_move (latched). Clears candidate if details is None."""
        if details is None:
            self._candidate_move_pub.publish(ChessMoveDetails())
            return
        if isinstance(details, ChessMoveGoal):
            msg = details.to_ros_move_details()
        else:
            msg = details
        self._candidate_move_pub.publish(msg)

    def _handle_execute_collection_task_service(
        self,
        request: ExecuteCollectionTask.Request,
        response: ExecuteCollectionTask.Response,
    ) -> ExecuteCollectionTask.Response:
        """Handle collection task dispatch from episode recorder."""
        mv = request.move
        details = ChessMoveGoal.from_uci_or_details(
            uci_move=mv.uci,
            move_details=mv,
            force_pick_place=request.force_pick_place,
            clear_bin_side=request.clear_bin_side,
            bypass_steps=request.bypass_steps,
        )
        if details is None:
            response.success = False
            response.message = f"Invalid move details: uci='{mv.uci}'"
            self.get_logger().error(response.message)
            return response

        # Mark current FEN as processed to prevent re-dispatching from referee latched state
        self._game_status_handler.last_processed_fen = self._game_status_handler.latest_full_fen

        success, msg = self._move_workflow.dispatch_collection_task(details)
        response.success = success
        response.message = msg
        return response

    def _handle_import_fen_service(
        self, request: SetFen.Request, response: SetFen.Response
    ) -> SetFen.Response:
        """Import a FEN string, validate robot stationary state, and coordinate board sync."""
        target_fen = request.fen.strip()
        target_color = request.robot_color.strip()

        # Check if request.fen is a JSON payload (e.g. from an export_fen call)
        if target_fen.startswith("{") and target_fen.endswith("}"):
            try:
                data = json.loads(target_fen)
                if isinstance(data, dict):
                    if "fen" in data and isinstance(data["fen"], str):
                        target_fen = data["fen"].strip()
                    if not target_color and "robot_color" in data and isinstance(data["robot_color"], str):
                        target_color = data["robot_color"].strip()
            except Exception as ex:
                self.get_logger().warn(f"Failed to parse FEN JSON payload: {ex}")

        if not target_fen:
            response.success = False
            response.message = "Missing required FEN string"
            self.get_logger().warn(f"Rejected import_fen: {response.message}")
            return response

        # Auto-deduce robot_color from FEN active color if not specified
        if not target_color:
            parts = target_fen.split()
            if len(parts) >= 2 and parts[1].lower() in ("w", "b"):
                target_color = "black" if parts[1].lower() == "b" else "white"
            else:
                target_color = "white"

        with self._state_lock:
            current_state = self.mission_state

        if current_state not in IMPORT_FEN_ALLOWED_STATES:
            response.success = False
            response.message = (
                f"Cannot import FEN while robot is in active state {current_state.name}. "
                f"Allowed states: {[s.name for s in IMPORT_FEN_ALLOWED_STATES]}"
            )
            self.get_logger().warn(f"Rejected import_fen: {response.message}")
            return response

        if not self._chess_set_fen_client.service_is_ready():
            response.success = False
            response.message = (
                f"Chess set_fen service '{self.parameters.chess_set_fen_srv}' is not available"
            )
            self.get_logger().warn(response.message)
            return response

        chess_req = SetFen.Request(fen=target_fen, robot_color=target_color)
        future = self._chess_set_fen_client.call_async(chess_req)

        done_event = threading.Event()
        chess_resp: list[SetFen.Response | None] = [None]
        err_box: list[str | None] = [None]

        def _on_done(fut: Any) -> None:
            try:
                chess_resp[0] = fut.result()
            except Exception as ex:
                err_box[0] = str(ex)
            finally:
                done_event.set()

        future.add_done_callback(_on_done)
        timeout_sec = self.parameters.action_timeout_sec
        if not done_event.wait(timeout=timeout_sec):
            response.success = False
            response.message = f"Timed out waiting for chess set_fen service after {timeout_sec:.1f}s"
            self.get_logger().warn(response.message)
            return response

        if err_box[0] is not None:
            response.success = False
            response.message = f"Error calling chess set_fen service: {err_box[0]}"
            self.get_logger().warn(response.message)
            return response

        res = chess_resp[0]
        if res is None or not res.success:
            response.success = False
            response.message = res.message if res else "Chess service returned empty response"
            self.get_logger().warn(f"Chess set_fen rejected request: {response.message}")
            return response

        # Cancel active recovery timers or latches if we were in ERROR_FALLBACK
        self._health_supervisor.cancel_recovery_timer()
        self._health_supervisor.set_operator_latched(False)

        # Update robot color immediately in HealthSupervisor and GameStatusHandler
        if res.robot_color:
            self._health_supervisor.set_robot_color(res.robot_color)
            self._game_status_handler.set_robot_color(res.robot_color)

        self.clear_move_goals()
        self._game_status_handler.on_fen_imported(res.canonical_fen)
        if current_state != MacroMissionState.WAITING_FOR_TF_READY:
            self.transition_to(MacroMissionState.AWAITING_BOARD_SYNC)
            self._perception.set_context(PerceptionContext.BOARD_STATE_SCAN)
        else:
            self.get_logger().info(
                f"[IMPORT FEN] FEN '{res.canonical_fen}' imported during WAITING_FOR_TF_READY. "
                "Will transition to WAITING_FOR_PLAYER_MOVE once Navigation is confirmed ready."
            )

        response.success = True
        response.canonical_fen = res.canonical_fen
        response.active_color = res.active_color
        response.robot_color = res.robot_color
        response.message = res.message
        return response

    def _on_post_move_watchdog_timeout(self) -> None:
        """Handle post-move verification watchdog expiry."""
        self._post_move_verifier.on_timeout()

    def _finalize_turn_completed(self) -> None:
        """Reset verification state and transition FSM to await opponent move."""
        self._post_move_verifier.reset()
        self._last_interaction_pose = None
        self.clear_move_goals()
        self.transition_to(MacroMissionState.TURN_COMPLETED)
        self._perception.set_context(PerceptionContext.BOARD_STATE_SCAN)
        self.transition_to(MacroMissionState.WAITING_FOR_PLAYER_MOVE)
        self.get_logger().info(
            ">>> Turn finalized successfully. Waiting for opponent move."
        )

    def _on_nav_ready(self, msg: Bool) -> None:
        """Forward navigation readiness message to HealthSupervisor."""
        self._health_supervisor.on_nav_ready_msg(msg)

    def _on_grasp_ready(self, msg: Bool) -> None:
        """Forward grasp readiness message to HealthSupervisor."""
        self._health_supervisor.on_grasp_ready_msg(msg)

    def _on_nav_ready_confirmed(self, is_ready: bool = True) -> None:
        """Handle confirmed navigation readiness transition.

        Resumes pending interrupted move if recovering from fault, otherwise transitions
        to initial game state based on assigned robot color.

        Args:
            is_ready: Readiness status flag confirmed by HealthSupervisor.
        """
        if not is_ready:
            return

        with self._state_lock:
            if self._mission_state != MacroMissionState.WAITING_FOR_TF_READY:
                return
            pending_goal = self._pending_recovery_move

        if pending_goal is not None:
            self.get_logger().info(
                f"[RECOVERY RESUME] Resuming interrupted move '{pending_goal.uci}' "
                "after navigation/TF confirmed ready."
            )
            self._perception.set_context(PerceptionContext.TF_TRACKING_AND_NAV)
            if self.transition_to(MacroMissionState.EVALUATING_BEST_MOVE):
                with self._state_lock:
                    self._pending_recovery_move = None
                self._dispatch_move_workflow(
                    pending_goal.uci, move_details=pending_goal
                )
            return

        initial_state = MacroMissionState.WAITING_FOR_PLAYER_MOVE
        self.transition_to(initial_state)
        self._perception.set_context(PerceptionContext.BOARD_STATE_SCAN)

    def _on_game_status(self, msg: ChessGameStatus) -> None:
        """Forward incoming game status message to workflow handler."""
        def _clear_pending_if_new_fen():
            if (
                self._pending_recovery_move is not None
                and msg.full_fen != self._game_status_handler.last_processed_fen
            ):
                self.get_logger().info(
                    f"[ORCHESTRATOR] Discarding stale recovery move '{self._pending_recovery_move.uci}' "
                    f"due to new board FEN: {msg.full_fen}"
                )
                self._pending_recovery_move = None

        self._game_status_handler.on_game_status(
            msg, clear_pending_recovery_cb=_clear_pending_if_new_fen
        )

    # ================= Workflow Coordination =================

    def set_current_move(self, details: ChessMoveGoal) -> None:
        """Store active move details under state lock."""
        with self._state_lock:
            self._current_move_details = details
            self._current_goal_move = details.uci

    def clear_move_goals(self) -> None:
        """Clear all active and pending recovery move goals under state lock."""
        with self._state_lock:
            self._pending_recovery_move = None
            self._current_goal_move = None
            self._current_move_details = None
        self.publish_candidate_move(None)

    def set_last_interaction_pose(self, pose: PoseStamped | None) -> None:
        """Record base pose where last board manipulation occurred."""
        self._last_interaction_pose = pose

    def start_post_move_watchdog(self) -> None:
        """Start post-move verification timeout watchdog."""
        self._post_move_verifier.start()

    def _dispatch_move_workflow(
        self,
        uci_move: str,
        move_details: ChessMoveDetails | ChessMoveGoal | None = None,
    ) -> None:
        """Dispatch move feasibility check and execution pipeline."""
        self._move_workflow.dispatch_move_workflow(uci_move, move_details)

    def _on_feasibility_error(self, error_msg: str) -> None:
        """Handle move feasibility service failure."""
        self._move_workflow.on_feasibility_error(error_msg)

    def _on_feasibility_response(
        self, resp: CheckMoveFeasibility.Response, details: ChessMoveGoal
    ) -> None:
        """Handle move feasibility service response."""
        self._move_workflow.on_feasibility_response(resp, details)

    def _on_move_pipeline_completed(self) -> None:
        """Handle successful completion of move execution pipeline."""
        self._move_workflow.on_move_pipeline_completed()

    def _on_move_pipeline_failed(self, error_msg: str) -> None:
        """Handle failure in move execution pipeline."""
        self._move_workflow.on_move_pipeline_failed(error_msg)

    def publish_diagnostics_snapshot(self) -> None:
        """Delegate diagnostic snapshot to health supervisor."""
        snapshot = self._get_orchestrator_snapshot()
        self._health_supervisor.publish_diagnostics(snapshot)


def main(args: Any = None) -> None:
    """Initialize ROS 2 and spin multithreaded orchestrator node."""
    rclpy.init(args=args)
    node = ChessMissionOrchestrator()
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
