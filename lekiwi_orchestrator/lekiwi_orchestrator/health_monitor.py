# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""
Health, Lease Monitoring, Self-Healing and Telemetry Manager for LeKiwi Orchestrator.
Consolidates Navigation & Grasp readiness TTL lease, auto-recovery timers, and diagnostics publishing.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray

from lekiwi_orchestrator.fsm import (
    MISSION_STATE_NAMES,
    PERCEPTION_CONTEXT_NAMES,
    MissionState,
)
from lekiwi_orchestrator.visualizer import MissionStatusMarkerBuilder, VisualizerConfig


@dataclass(frozen=True)
class OrchestratorStateSnapshot:
    """Immutable state snapshot provided by the host orchestrator to avoid circular coupling."""

    mission_state: MissionState
    perception_context: int
    last_goal_move: str | None = None
    execution_stage: str | None = None


@dataclass(frozen=True)
class HealthMonitorConfig:
    """Immutable configuration for NodeHealthMonitor."""

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


class NodeHealthMonitor:
    """Manages Navigation and Grasp readiness leases, auto-recovery, and diagnostics."""

    def __init__(
        self,
        node: Node,
        config: HealthMonitorConfig | None = None,
        state_provider: Callable[[], OrchestratorStateSnapshot] | None = None,
        state_lock: threading.RLock | None = None,
        cb_group_sub: MutuallyExclusiveCallbackGroup | None = None,
        on_nav_ready_transition_cb: Callable[[bool], None] | None = None,
        on_critical_tf_loss_cb: Callable[[], None] | None = None,
        on_auto_recovery_cb: Callable[[], None] | None = None,
        on_reset_recovery_system_cb: Callable[[str], bool] | None = None,
    ) -> None:
        self._node = node
        self._config = config or HealthMonitorConfig()
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
                mission_state=MissionState.BOOT_INITIALIZING, perception_context=0
            )
        )

        # Visualizer Setup
        self._enable_visualizer = self._config.enable_visualizer
        self._visualizer_topic = self._config.visualizer_topic
        self._visualizer_config = self._config.visualizer_config
        self._marker_builder = (
            MissionStatusMarkerBuilder(self._visualizer_config)
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
            clock=Clock(clock_type=ClockType.STEADY_TIME),
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

    @property
    def config(self) -> HealthMonitorConfig:
        return self._config

    @property
    def is_nav_ready(self) -> bool:
        """Evaluate if the navigation readiness lease is valid within the timeout window."""
        with self._state_lock:
            return (
                self.nav_ready
                and self.last_nav_heartbeat is not None
                and time.monotonic() - self.last_nav_heartbeat
                <= self._config.readiness_timeout_sec
            )

    @property
    def is_grasp_ready(self) -> bool:
        """Evaluate if the grasp precision readiness lease is valid within the timeout window."""
        with self._state_lock:
            return (
                self.grasp_ready
                and self.last_grasp_heartbeat is not None
                and time.monotonic() - self.last_grasp_heartbeat
                <= self._config.readiness_timeout_sec
            )

    def on_nav_ready_msg(
        self,
        msg: Bool,
        current_state: MissionState | None = None,
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
            self.last_nav_heartbeat = time.monotonic()
            self.nav_ready = bool(msg.data)

            if self.nav_ready and not previous:
                self._node.get_logger().info(
                    ">>> [ORCHESTRATOR] Navigation Ready confirmed! Base odometry/TF active."
                )
                transition_cb = self._on_nav_ready_transition_cb
            elif not self.nav_ready and previous:
                if state in (
                    MissionState.BOOT_INITIALIZING,
                    MissionState.WAITING_FOR_TF_READY,
                    MissionState.CHECKING_REACHABILITY,
                ):
                    self._node.get_logger().warn(
                        f"<<< [ORCHESTRATOR] Navigation Unready in critical state: "
                        f"{MISSION_STATE_NAMES.get(state, 'UNKNOWN')}"
                    )
                    if state == MissionState.CHECKING_REACHABILITY:
                        critical_loss_cb = self._on_critical_tf_loss_cb
                else:
                    self._node.get_logger().debug(
                        f"<<< [ORCHESTRATOR] Navigation Unready in un-gated state: "
                        f"{MISSION_STATE_NAMES.get(state, 'UNKNOWN')} (continuing)"
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
            self.last_grasp_heartbeat = time.monotonic()
            self.grasp_ready = bool(msg.data)

    def expire_readiness(self, current_state: MissionState | None = None) -> None:
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
                if state in (
                    MissionState.BOOT_INITIALIZING,
                    MissionState.WAITING_FOR_TF_READY,
                    MissionState.CHECKING_REACHABILITY,
                ):
                    self._node.get_logger().warning(
                        f"Navigation readiness heartbeat expired (odometry/TF lost): "
                        f"{MISSION_STATE_NAMES.get(state, 'UNKNOWN')}"
                    )
                    if state == MissionState.CHECKING_REACHABILITY:
                        critical_loss_cb = self._on_critical_tf_loss_cb
                else:
                    self._node.get_logger().debug(
                        f"Navigation readiness heartbeat expired in un-gated state: "
                        f"{MISSION_STATE_NAMES.get(state, 'UNKNOWN')} (ignored)"
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

        if self._recovery_timer is not None:
            self._recovery_timer.cancel()

        self._recovery_timer = self._node.create_timer(
            delay,
            self._on_auto_recovery_timer_fired,
            callback_group=self._cb_group_sub,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
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
        """Cancel any running recovery timer."""
        if self._recovery_timer is not None:
            self._recovery_timer.cancel()
            self._recovery_timer = None

    def handle_recover_service(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
        current_state: MissionState | None = None,
    ) -> Trigger.Response:
        """Handle manual recovery request on /orchestrator/recover."""
        with self._state_lock:
            self.recovery_attempts = 0
        state = (
            current_state
            if current_state is not None
            else self._state_provider().mission_state
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

        if state == MissionState.ERROR_FALLBACK:
            diag.level = DiagnosticStatus.ERROR
            diag.message = "System in Error Fallback state"
        elif (
            state in (MissionState.BOOT_INITIALIZING, MissionState.WAITING_FOR_TF_READY)
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
