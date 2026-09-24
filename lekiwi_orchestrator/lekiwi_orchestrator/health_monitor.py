# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""
Health, Lease Monitoring, Self-Healing and Telemetry Manager for LeKiwi Orchestrator.
Consolidates TF readiness TTL lease, auto-recovery timers, and diagnostics publishing.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from lekiwi_orchestrator.fsm import (
    CAMERA_MODE_NAMES,
    MISSION_STATE_NAMES,
    MissionState,
)


class NodeHealthMonitor:
    """Manages TF lease freshness, auto-recovery, and diagnostics."""

    def __init__(
        self,
        node: Node,
        tf_ready_topic: str,
        diagnostics_topic: str,
        recover_service_name: str,
        readiness_timeout_sec: float,
        auto_recovery_enabled: bool,
        auto_recovery_timeout_sec: float,
        max_recovery_attempts: int,
        robot_color: str,
        state_lock: threading.RLock,
        cb_group_sub: MutuallyExclusiveCallbackGroup,
        on_tf_ready_transition_cb: Callable[[bool], None],
        on_critical_tf_loss_cb: Callable[[], None],
        on_auto_recovery_cb: Callable[[], None],
        on_reset_recovery_system_cb: Callable[[str], bool],
    ) -> None:
        self._node = node
        self._tf_ready_topic = tf_ready_topic
        self._diagnostics_topic = diagnostics_topic
        self._recover_service_name = recover_service_name
        self._readiness_timeout_sec = readiness_timeout_sec
        self._auto_recovery_enabled = auto_recovery_enabled
        self._auto_recovery_timeout_sec = auto_recovery_timeout_sec
        self._max_recovery_attempts = max_recovery_attempts
        self._robot_color = robot_color
        self._state_lock = state_lock
        self._cb_group_sub = cb_group_sub

        self._on_tf_ready_transition_cb = on_tf_ready_transition_cb
        self._on_critical_tf_loss_cb = on_critical_tf_loss_cb
        self._on_auto_recovery_cb = on_auto_recovery_cb
        self._on_reset_recovery_system_cb = on_reset_recovery_system_cb

        # State tracking
        self.tf_ready = False
        self.last_readiness_heartbeat: float | None = None
        self.recovery_attempts = 0

        # Timers
        self._recovery_timer = None
        self._readiness_timer = self._node.create_timer(
            min(0.2, self._readiness_timeout_sec / 2),
            self.expire_readiness,
            callback_group=self._cb_group_sub,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

        # Publisher
        self._diag_pub = self._node.create_publisher(
            DiagnosticArray, self._diagnostics_topic, 10
        )
        self._diag_timer = self._node.create_timer(1.0, self._on_diag_timer)

    @property
    def is_tf_ready(self) -> bool:
        """Evaluate if the TF lease is valid within the readiness timeout window."""
        with self._state_lock:
            return (
                self.tf_ready
                and self.last_readiness_heartbeat is not None
                and time.monotonic() - self.last_readiness_heartbeat
                <= self._readiness_timeout_sec
            )

    def on_tf_ready_msg(self, msg: Bool, current_state: MissionState) -> None:
        """Handle incoming TF ready message."""
        with self._state_lock:
            previous = self.is_tf_ready
            self.last_readiness_heartbeat = time.monotonic()
            self.tf_ready = bool(msg.data)

            if self.tf_ready and not previous:
                self._node.get_logger().info(
                    ">>> [ORCHESTRATOR] TF Ready confirmed! System is localized & stationary."
                )
                self._on_tf_ready_transition_cb(True)
            elif not self.tf_ready and previous:
                if current_state in (
                    MissionState.BOOT_INITIALIZING,
                    MissionState.WAITING_FOR_TF_READY,
                    MissionState.CHECKING_REACHABILITY,
                ):
                    self._node.get_logger().warn(
                        f"<<< [ORCHESTRATOR] TF Unready in critical state: "
                        f"{MISSION_STATE_NAMES.get(current_state, 'UNKNOWN')}"
                    )
                    if current_state == MissionState.CHECKING_REACHABILITY:
                        self._on_critical_tf_loss_cb()
                else:
                    self._node.get_logger().debug(
                        f"<<< [ORCHESTRATOR] TF Unready in un-gated state: "
                        f"{MISSION_STATE_NAMES.get(current_state, 'UNKNOWN')} (continuing)"
                    )

    def expire_readiness(self, current_state: MissionState | None = None) -> None:
        """Heartbeat lease expiration check."""
        with self._state_lock:
            was_ready = self.tf_ready
            is_ready = self.is_tf_ready
            if was_ready and not is_ready:
                self.tf_ready = False
                state = (
                    current_state
                    if current_state is not None
                    else getattr(self._node, "mission_state", None)
                )
                if state in (
                    MissionState.BOOT_INITIALIZING,
                    MissionState.WAITING_FOR_TF_READY,
                    MissionState.CHECKING_REACHABILITY,
                ):
                    self._node.get_logger().warning(
                        f"TF readiness heartbeat expired in critical state: "
                        f"{MISSION_STATE_NAMES.get(state, 'UNKNOWN')}"
                    )
                    if state == MissionState.CHECKING_REACHABILITY:
                        self._on_critical_tf_loss_cb()
                else:
                    self._node.get_logger().debug(
                        f"TF readiness heartbeat expired in un-gated state: "
                        f"{MISSION_STATE_NAMES.get(state, 'UNKNOWN')} (ignored)"
                    )

    def reset_recovery_attempts(self) -> None:
        """Reset auto-recovery attempt counter upon healthy state transitions."""
        with self._state_lock:
            self.recovery_attempts = 0

    def schedule_auto_recovery_if_enabled(self) -> None:
        """Schedule automatic recovery timer when entering ERROR_FALLBACK."""
        if not self._auto_recovery_enabled:
            return

        with self._state_lock:
            if self.recovery_attempts >= self._max_recovery_attempts:
                self._node.get_logger().error(
                    f"Max auto-recovery attempts ({self._max_recovery_attempts}) exceeded! "
                    f"Awaiting manual intervention via {self._recover_service_name}."
                )
                return

            self.recovery_attempts += 1
            attempts = self.recovery_attempts
            max_att = self._max_recovery_attempts
            delay = self._auto_recovery_timeout_sec

        self._node.get_logger().warn(
            f"[SELF-HEALING] Scheduling auto-recovery attempt {attempts}/{max_att} in {delay:.1f}s..."
        )
        self.cancel_recovery_timer()
        self._recovery_timer = self._node.create_timer(
            delay,
            self._on_auto_recovery_timer_fired,
            callback_group=self._cb_group_sub,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

    def _on_auto_recovery_timer_fired(self) -> None:
        self.cancel_recovery_timer()
        self._on_auto_recovery_cb()

    def cancel_recovery_timer(self) -> None:
        """Cancel and release active recovery timer."""
        if self._recovery_timer is not None:
            self._recovery_timer.cancel()
            self._node.destroy_timer(self._recovery_timer)
            self._recovery_timer = None

    def handle_recover_service(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
        current_state: MissionState,
    ) -> Trigger.Response:
        """Handle manual recovery request on /orchestrator/recover."""
        with self._state_lock:
            self.recovery_attempts = 0
        success = self._on_reset_recovery_system_cb("operator service request")
        response.success = success
        response.message = (
            "System recovered to WAITING_FOR_TF_READY"
            if success
            else f"Recovery rejected: node is not in ERROR_FALLBACK (current: {MISSION_STATE_NAMES.get(current_state, 'UNKNOWN')})"
        )
        return response

    def _on_diag_timer(self) -> None:
        """Periodic diagnostic publisher callback."""
        if hasattr(self._node, "publish_diagnostics_snapshot"):
            self._node.publish_diagnostics_snapshot()

    def publish_diagnostics(
        self,
        mission_state: MissionState,
        camera_mode: int,
        last_goal_move: str | None,
        dual_base_phase: str | None,
    ) -> None:
        """Assemble and publish diagnostic array."""
        with self._state_lock:
            state = mission_state
            cam_mode = camera_mode
            tf_ready_val = self.tf_ready
            robot_col = self._robot_color
            last_goal = last_goal_move
            phase = dual_base_phase
            attempts = self.recovery_attempts

        diag = DiagnosticStatus()
        diag.name = "Chess Mission Orchestrator"
        diag.hardware_id = "LeKiwi_Brain"

        if state == MissionState.ERROR_FALLBACK:
            diag.level = DiagnosticStatus.ERROR
            diag.message = "System in Error Fallback state"
        elif (
            state in (MissionState.BOOT_INITIALIZING, MissionState.WAITING_FOR_TF_READY)
            and not tf_ready_val
        ):
            diag.level = DiagnosticStatus.WARN
            diag.message = "Waiting for TF readiness"
        else:
            diag.level = DiagnosticStatus.OK
            diag.message = f"Active ({MISSION_STATE_NAMES.get(state, 'UNKNOWN')})"

        diag.values = [
            KeyValue(
                key="mission_state", value=MISSION_STATE_NAMES.get(state, "UNKNOWN")
            ),
            KeyValue(
                key="camera_mode", value=CAMERA_MODE_NAMES.get(cam_mode, "UNKNOWN")
            ),
            KeyValue(key="tf_ready", value=str(tf_ready_val)),
            KeyValue(key="robot_color", value=robot_col),
            KeyValue(key="last_goal_move", value=str(last_goal)),
            KeyValue(key="dual_base_phase", value=str(phase)),
            KeyValue(key="recovery_attempts", value=str(attempts)),
        ]

        diag_array = DiagnosticArray()
        diag_array.header.stamp = self._node.get_clock().now().to_msg()
        diag_array.status.append(diag)
        self._diag_pub.publish(diag_array)

    def destroy(self) -> None:
        """Clean up active timers."""
        self.cancel_recovery_timer()
        if self._readiness_timer is not None:
            self._readiness_timer.cancel()
            self._node.destroy_timer(self._readiness_timer)
            self._readiness_timer = None
        if self._diag_timer is not None:
            self._diag_timer.cancel()
            self._node.destroy_timer(self._diag_timer)
            self._diag_timer = None
