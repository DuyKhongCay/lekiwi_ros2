# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""
Health, readiness lease supervision, auto-recovery, and 3D HUD visualizer.

Encapsulates:
1. Navigation & Grasp Precision heartbeat leases (/system/nav_ready, /system/grasp_ready).
2. Auto-recovery backoff state machine and /orchestrator/recover service.
3. Diagnostic telemetry publisher (/diagnostics).
4. 3D Status HUD MarkerArray publisher (~/status_markers).
"""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from lekiwi_orchestrator.fsm import (
    MISSION_STATE_NAMES,
    MOTION_STATE_NAMES,
    PERCEPTION_CONTEXT_NAMES,
    MacroMissionState,
    MotionExecutionState,
)


# ==============================================================================
# Snapshot & Visualizer Configuration
# ==============================================================================


@dataclass(frozen=True)
class OrchestratorStateSnapshot:
    """Read-only snapshot of high-level orchestrator state for diagnostics and HUD visualization."""

    mission_state: MacroMissionState
    perception_context: int
    last_goal_move: str | None = None
    execution_stage: str | None = None
    motion_state: MotionExecutionState = MotionExecutionState.IDLE


@dataclass(frozen=True)
class VisualizerConfig:
    """Configuration for 3D Robot Status HUD Marker."""

    robot_frame: str = "base_footprint"
    hud_z_offset: float = 0.35
    font_scale: float = 0.025
    ns: str = "mission/robot_hud"


class StatusMarkerBuilder:
    """
    Pure builder producing standardized ROS 2 MarkerArray for Robot 3D Status HUD.
    Decoupled from ROS node context for deterministic testing.
    """

    def __init__(self, config: VisualizerConfig | None = None) -> None:
        self._config = config or VisualizerConfig()

    @property
    def config(self) -> VisualizerConfig:
        return self._config

    @staticmethod
    def _format_hud_content(
        snapshot: OrchestratorStateSnapshot,
        nav_ready: bool,
        robot_color: str,
    ) -> tuple[tuple[float, float, float, float], str]:
        state = snapshot.mission_state
        state_str = MISSION_STATE_NAMES.get(state, "UNKNOWN")
        color_name = "White" if robot_color.lower().startswith("w") else "Black"

        if state in (MacroMissionState.BOOT_INITIALIZING, MacroMissionState.WAITING_FOR_TF_READY):
            return (
                1.0,
                0.8,
                0.0,
                1.0,
            ), f"[{state_str}]\nWaiting for Nav Ready | Robot: {color_name}"

        if state == MacroMissionState.WAITING_FOR_PLAYER_MOVE:
            nav_status = "Ready" if nav_ready else "Unready"
            return (
                (0.2, 0.8, 1.0, 1.0),
                f"[IDLE] Waiting for Opponent\nRobot: {color_name} | Nav: {nav_status}",
            )

        if state in (
            MacroMissionState.EVALUATING_BEST_MOVE,
            MacroMissionState.CHECKING_REACHABILITY,
        ):
            target_str = snapshot.last_goal_move or "Evaluating"
            return (
                0.95,
                0.9,
                0.2,
                1.0,
            ), f"[{state_str}]\nTarget: {target_str} ({color_name})"

        if state == MacroMissionState.EXECUTING_MOVE_PIPELINE:
            stage_str = snapshot.execution_stage or "Executing"
            move_str = snapshot.last_goal_move or "Unknown"
            return (
                0.1,
                1.0,
                0.3,
                1.0,
            ), f"[EXECUTING] Move: {move_str} ({color_name})\nStage: {stage_str}"

        if state == MacroMissionState.TURN_COMPLETED:
            return (
                0.0,
                1.0,
                0.8,
                1.0,
            ), f"[COMPLETED] Turn Finalized\nRobot: {color_name}"

        if state == MacroMissionState.ERROR_FALLBACK:
            nav_status = "Ready" if nav_ready else "Lost"
            return (
                1.0,
                0.1,
                0.1,
                1.0,
            ), f"[ERROR] System in Error Fallback\nNav: {nav_status}"

        return (0.85, 0.85, 0.85, 1.0), f"[{state_str}]\nRobot: {color_name}"

    def build(
        self,
        snapshot: OrchestratorStateSnapshot,
        nav_ready: bool,
        robot_color: str,
        stamp: Time,
    ) -> MarkerArray:
        """
        Assemble MarkerArray with DELETEALL hygiene followed by a 3D text billboard.
        """
        markers = MarkerArray()

        # 0. Always start with DELETEALL to prevent stale remnants across state changes
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        clear_marker.header.frame_id = self._config.robot_frame
        clear_marker.header.stamp = stamp
        markers.markers.append(clear_marker)

        # 1. Layer 1: Robot 3D Status HUD Billboard
        hud = Marker()
        hud.header.frame_id = self._config.robot_frame
        hud.header.stamp = stamp
        hud.ns = self._config.ns
        hud.id = 0
        hud.type = Marker.TEXT_VIEW_FACING
        hud.action = Marker.ADD

        hud.pose.position.x = 0.0
        hud.pose.position.y = 0.0
        hud.pose.position.z = self._config.hud_z_offset
        hud.pose.orientation.w = 1.0

        hud.scale.z = self._config.font_scale

        (hud.color.r, hud.color.g, hud.color.b, hud.color.a), hud.text = (
            self._format_hud_content(snapshot, nav_ready, robot_color)
        )

        markers.markers.append(hud)
        return markers


# ==============================================================================
# Node Health Supervisor
# ==============================================================================


@dataclass
class HealthSupervisorConfig:
    """Configuration container for HealthSupervisor."""

    nav_ready_topic: str = "/system/nav_ready"
    grasp_ready_topic: str = "/system/grasp_ready"
    diagnostics_topic: str = "/diagnostics"
    recover_service_name: str = "/orchestrator/recover"
    readiness_timeout_sec: float = 1.0
    auto_recovery_enabled: bool = True
    auto_recovery_timeout_sec: float = 5.0
    max_recovery_attempts: int = 3
    robot_color: str = "w"
    enable_visualizer: bool = True
    visualizer_topic: str = "~/status_markers"
    visualizer_config: VisualizerConfig = field(default_factory=VisualizerConfig)




class HealthSupervisor:
    """Manages Navigation and Grasp readiness leases, auto-recovery, and diagnostics."""

    def __init__(
        self,
        node: Node,
        config: HealthSupervisorConfig | None = None,
        state_provider: Callable[[], OrchestratorStateSnapshot] | None = None,
        state_lock: threading.RLock | None = None,
        cb_group_sub: MutuallyExclusiveCallbackGroup | None = None,
        on_nav_ready_transition_cb: Callable[[bool], None] | None = None,
        on_critical_tf_loss_cb: Callable[[], None] | None = None,
        on_auto_recovery_cb: Callable[[], None] | None = None,
        on_reset_recovery_system_cb: Callable[[str], bool] | None = None,
    ) -> None:
        self._node = node
        self._config = config or HealthSupervisorConfig()
        self._state_lock = state_lock or threading.RLock()
        self._cb_group_sub = cb_group_sub

        self._on_nav_ready_transition_cb = on_nav_ready_transition_cb or (
            lambda _: None
        )
        self._on_critical_tf_loss_cb = on_critical_tf_loss_cb or (lambda: None)
        self._on_auto_recovery_cb = on_auto_recovery_cb or (lambda: None)
        self._on_reset_recovery_system_cb = on_reset_recovery_system_cb or (
            lambda _: True
        )
        self._state_provider = state_provider or (
            lambda: OrchestratorStateSnapshot(
                mission_state=MacroMissionState.BOOT_INITIALIZING, perception_context=0
            )
        )

        # Visualizer Setup
        self._enable_visualizer = self._config.enable_visualizer
        self._visualizer_topic = self._config.visualizer_topic
        self._visualizer_config = self._config.visualizer_config
        self._marker_builder = (
            StatusMarkerBuilder(self._visualizer_config)
            if self._enable_visualizer
            else None
        )

        # State tracking: independent Navigation and Grasp readiness
        self.nav_ready = False
        self.grasp_ready = False
        self.last_nav_heartbeat: float | None = None
        self.last_grasp_heartbeat: float | None = None
        self.recovery_attempts = 0

        # Timers
        self._recovery_timer = None
        self._readiness_timer = self._node.create_timer(
            min(0.2, self._config.readiness_timeout_sec / 2),
            self.expire_readiness,
            callback_group=self._cb_group_sub,
        )

        # Publishers
        self._diag_pub = self._node.create_publisher(
            DiagnosticArray, self._config.diagnostics_topic, 10
        )
        if self._enable_visualizer:
            pub_qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            self._marker_pub = self._node.create_publisher(
                MarkerArray, self._visualizer_topic, pub_qos
            )
        else:
            self._marker_pub = None

        self._diag_timer = self._node.create_timer(1.0, self._on_diag_timer)

    def _get_now_sec(self) -> float:
        """Return current timestamp in seconds using node clock, fallback to monotonic."""
        try:
            if hasattr(self._node, "get_clock"):
                clk = self._node.get_clock()
                if clk is not None:
                    now = clk.now()
                    if hasattr(now, "nanoseconds") and isinstance(now.nanoseconds, (int, float)):
                        return float(now.nanoseconds) * 1e-9
                    if hasattr(now, "seconds_nanoseconds") and callable(now.seconds_nanoseconds):
                        res = now.seconds_nanoseconds()
                        if isinstance(res, tuple) and len(res) == 2 and isinstance(res[0], (int, float)):
                            sec, nsec = res
                            return float(sec) + float(nsec) * 1e-9
        except Exception:
            pass
        return time.monotonic()


    @property
    def is_nav_ready(self) -> bool:
        """Check if navigation readiness lease is valid within TTL."""
        with self._state_lock:
            if not self.nav_ready or self.last_nav_heartbeat is None:
                return False
            now = self._get_now_sec()
            return (now - self.last_nav_heartbeat) <= self._config.readiness_timeout_sec

    @property
    def is_grasp_ready(self) -> bool:
        """Check if grasp precision readiness lease is valid within TTL."""
        with self._state_lock:
            if not self.grasp_ready or self.last_grasp_heartbeat is None:
                return False
            now = self._get_now_sec()
            return (now - self.last_grasp_heartbeat) <= self._config.readiness_timeout_sec

    def on_nav_ready_msg(
        self,
        msg: Bool,
        current_state: MacroMissionState | None = None,
    ) -> None:
        """Handle incoming Navigation Ready message (/system/nav_ready)."""
        transition_cb: Callable[[bool], None] | None = None
        critical_loss_cb: Callable[[], None] | None = None

        with self._state_lock:
            state = (
                current_state
                if current_state is not None
                else self._state_provider().mission_state
            )
            previous = self.is_nav_ready
            self.last_nav_heartbeat = self._get_now_sec()
            self.nav_ready = bool(msg.data)

            if self.nav_ready and (
                not previous or state == MacroMissionState.WAITING_FOR_TF_READY
            ):
                self._node.get_logger().info(
                    ">>> [ORCHESTRATOR] Navigation Ready confirmed! Base odometry/TF active."
                )
                transition_cb = self._on_nav_ready_transition_cb
            elif not self.nav_ready and previous:
                critical_loss_cb = self._handle_nav_unready_in_state(
                    state, "topic unready"
                )

        # Open Call Pattern: invoke callbacks outside lock to prevent deadlock
        if transition_cb is not None:
            transition_cb(True)
        if critical_loss_cb is not None:
            critical_loss_cb()

    def on_grasp_ready_msg(
        self,
        msg: Bool,
    ) -> None:
        """Handle incoming Grasp Precision Ready message (/system/grasp_ready)."""
        with self._state_lock:
            self.last_grasp_heartbeat = self._get_now_sec()
            self.grasp_ready = bool(msg.data)

    def _handle_nav_unready_in_state(
        self, state: MacroMissionState, reason: str
    ) -> Callable[[], None] | None:
        """Handle unready navigation transition/expiry and return critical loss callback if triggered."""
        state_name = MISSION_STATE_NAMES.get(state, "UNKNOWN")
        if state in (
            MacroMissionState.BOOT_INITIALIZING,
            MacroMissionState.WAITING_FOR_TF_READY,
            MacroMissionState.CHECKING_REACHABILITY,
        ):
            self._node.get_logger().warn(
                f"<<< [ORCHESTRATOR] Navigation unready ({reason}) in critical state: {state_name}"
            )
            if state == MacroMissionState.CHECKING_REACHABILITY:
                return self._on_critical_tf_loss_cb
            return None

        self._node.get_logger().debug(
            f"<<< [ORCHESTRATOR] Navigation unready ({reason}) in un-gated state: {state_name} (ignored)"
        )
        return None

    def expire_readiness(self, current_state: MacroMissionState | None = None) -> None:
        """Heartbeat lease expiration check for Navigation odometry."""
        critical_loss_cb: Callable[[], None] | None = None

        with self._state_lock:
            was_ready = self.nav_ready
            is_ready = self.is_nav_ready
            if was_ready and not is_ready:
                self.nav_ready = False
                state = (
                    current_state
                    if current_state is not None
                    else self._state_provider().mission_state
                )
                critical_loss_cb = self._handle_nav_unready_in_state(
                    state, "heartbeat expired"
                )

        # Open Call Pattern
        if critical_loss_cb is not None:
            critical_loss_cb()

    def reset_recovery_attempts(self) -> None:
        """Reset auto-recovery attempt counter upon healthy state transitions."""
        with self._state_lock:
            self.recovery_attempts = 0

    def schedule_auto_recovery_if_enabled(self) -> None:
        """Schedule automatic recovery timer when entering ERROR_FALLBACK."""
        if not self._config.auto_recovery_enabled:
            return

        with self._state_lock:
            if self.recovery_attempts >= self._config.max_recovery_attempts:
                self._node.get_logger().error(
                    f"Max auto-recovery attempts ({self._config.max_recovery_attempts}) exceeded! "
                    "Halting automatic recovery. Manual intervention (/orchestrator/recover) required."
                )
                return

            self.recovery_attempts += 1
            delay = self._config.auto_recovery_timeout_sec
            self._node.get_logger().warn(
                f"[RECOVERY] Scheduling auto-recovery attempt {self.recovery_attempts}/"
                f"{self._config.max_recovery_attempts} in {delay:.1f}s..."
            )

        self.cancel_recovery_timer()

        self._recovery_timer = self._node.create_timer(
            delay,
            self._on_auto_recovery_timer_fired,
            callback_group=self._cb_group_sub,
        )

    def _on_auto_recovery_timer_fired(self) -> None:
        """Execute scheduled auto-recovery."""
        self.cancel_recovery_timer()
        self._node.get_logger().info(
            f">>> [RECOVERY] Executing auto-recovery attempt {self.recovery_attempts}/"
            f"{self._config.max_recovery_attempts}..."
        )
        self._on_auto_recovery_cb()

    def cancel_recovery_timer(self) -> None:
        """Cancel any running recovery timer and destroy it from node."""
        if self._recovery_timer is not None:
            self._recovery_timer.cancel()
            self._node.destroy_timer(self._recovery_timer)
            self._recovery_timer = None

    def handle_recover_service(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
        current_state: MacroMissionState | None = None,
    ) -> Trigger.Response:
        """Handle /orchestrator/recover manual operator reset service."""
        state = (
            current_state
            if current_state is not None
            else self._state_provider().mission_state
        )
        self._node.get_logger().info(
            f"[RECOVERY] Manual recovery service called while in state: {MISSION_STATE_NAMES.get(state, 'UNKNOWN')}"
        )
        success = self._on_reset_recovery_system_cb("operator service request")
        response.success = success
        response.message = (
            "System recovered to WAITING_FOR_TF_READY"
            if success
            else f"Recovery rejected: node is not in ERROR_FALLBACK (current: {MISSION_STATE_NAMES.get(state, 'UNKNOWN')})"
        )
        return response

    def _on_diag_timer(self) -> None:
        """Periodic diagnostic publisher callback."""
        if self._state_provider is not None:
            snapshot = self._state_provider()
            self.publish_diagnostics(snapshot)

    def publish_diagnostics(
        self,
        snapshot: OrchestratorStateSnapshot,
    ) -> None:
        """Assemble and publish diagnostic array and RViz visualizer status markers."""
        state = snapshot.mission_state
        context_val = snapshot.perception_context
        last_goal = snapshot.last_goal_move
        stage_name = snapshot.execution_stage

        with self._state_lock:
            nav_ready_val = self.nav_ready
            grasp_ready_val = self.grasp_ready
            robot_col = self._config.robot_color
            attempts = self.recovery_attempts

        diag = DiagnosticStatus()
        diag.name = "Chess Mission Orchestrator"
        diag.hardware_id = "LeKiwi_Brain"

        if state == MacroMissionState.ERROR_FALLBACK:
            diag.level = DiagnosticStatus.ERROR
            diag.message = "System in Error Fallback state"
        elif (
            state in (MacroMissionState.BOOT_INITIALIZING, MacroMissionState.WAITING_FOR_TF_READY)
            and not nav_ready_val
        ):
            diag.level = DiagnosticStatus.WARN
            diag.message = "Waiting for Navigation readiness"
        else:
            diag.level = DiagnosticStatus.OK
            diag.message = f"Active ({MISSION_STATE_NAMES.get(state, 'UNKNOWN')})"

        diag.values = [
            KeyValue(
                key="mission_state", value=MISSION_STATE_NAMES.get(state, "UNKNOWN")
            ),
            KeyValue(
                key="micro_state",
                value=MOTION_STATE_NAMES.get(snapshot.motion_state, "IDLE"),
            ),
            KeyValue(
                key="perception_context",
                value=PERCEPTION_CONTEXT_NAMES.get(context_val, "UNKNOWN"),
            ),
            KeyValue(key="nav_ready", value=str(nav_ready_val)),
            KeyValue(key="grasp_ready", value=str(grasp_ready_val)),
            KeyValue(key="robot_color", value=robot_col),
            KeyValue(key="last_goal_move", value=str(last_goal)),
            KeyValue(key="execution_stage", value=str(stage_name)),
            KeyValue(key="recovery_attempts", value=str(attempts)),
        ]

        diag_array = DiagnosticArray()
        stamp = self._node.get_clock().now().to_msg()
        diag_array.header.stamp = stamp
        diag_array.status.append(diag)
        self._diag_pub.publish(diag_array)

        if self._marker_pub is not None and self._marker_builder is not None:
            markers = self._marker_builder.build(
                snapshot=snapshot,
                nav_ready=nav_ready_val,
                robot_color=self._config.robot_color,
                stamp=stamp,
            )
            self._marker_pub.publish(markers)

    def destroy(self) -> None:
        """Clean up active timers and publishers."""
        self.cancel_recovery_timer()
        if self._readiness_timer is not None:
            self._node.destroy_timer(self._readiness_timer)
            self._readiness_timer = None
        if self._diag_timer is not None:
            self._node.destroy_timer(self._diag_timer)
            self._diag_timer = None
        if self._diag_pub is not None:
            self._node.destroy_publisher(self._diag_pub)
            self._diag_pub = None
        if self._marker_pub is not None:
            self._node.destroy_publisher(self._marker_pub)
            self._marker_pub = None


__all__ = [
    "OrchestratorStateSnapshot",
    "VisualizerConfig",
    "StatusMarkerBuilder",
    "HealthSupervisorConfig",
    "HealthSupervisor",
]
