# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""
Unit Tests for HealthSupervisor and HealthSupervisorConfig.
Verifies lease TTL expiration, Open Call thread safety, auto-recovery policies, and diagnostics.
"""

from __future__ import annotations

import threading
import time
from unittest.mock import MagicMock

import pytest
from diagnostic_msgs.msg import DiagnosticStatus
from nav_msgs.msg import Odometry
from lekiwi_orchestrator.core.fsm import MacroMissionState, MotionExecutionState
from lekiwi_orchestrator.supervision.health_supervisor import (
    HealthSupervisor,
    HealthSupervisorConfig,
    OrchestratorStateSnapshot,
)
from rclpy.time import Time
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


@pytest.fixture
def dummy_snapshot() -> OrchestratorStateSnapshot:
    return OrchestratorStateSnapshot(
        mission_state=MacroMissionState.WAITING_FOR_NEXT_MOVE,
        perception_context=0,
        last_goal_move="e2e4",
        execution_stage=None,
    )


@pytest.fixture
def mock_node():
    node = MagicMock()
    clock = MagicMock()
    start_mono = time.monotonic()

    def _now():
        now_mock = MagicMock()
        elapsed = time.monotonic() - start_mono
        now_mock.nanoseconds = int((100.0 + elapsed) * 1e9)
        time_msg = Time()
        time_msg.sec = int(100.0 + elapsed)
        now_mock.to_msg.return_value = time_msg
        return now_mock

    clock.now.side_effect = _now
    node.get_clock.return_value = clock
    return node


def test_readiness_leases_and_expiration(mock_node, dummy_snapshot):
    """Verify initial readiness state, lease publishing, timeouts, and critical state expiration."""
    transition_cb = MagicMock()
    critical_loss_cb = MagicMock()
    monitor = HealthSupervisor(
        node=mock_node,
        config=HealthSupervisorConfig(readiness_timeout_sec=0.1),
        state_provider=lambda: dummy_snapshot,
        on_nav_ready_transition_cb=transition_cb,
        on_critical_tf_loss_cb=critical_loss_cb,
    )
    try:
        # 1. Initial state
        assert not monitor.is_nav_ready
        assert not monitor.is_grasp_ready
        assert not monitor.nav_ready
        assert not monitor.grasp_ready

        # 2. Nav ready message and expiration
        monitor.on_nav_ready_msg(Bool(data=True))
        assert monitor.is_nav_ready
        assert monitor.nav_ready
        assert transition_cb.called

        time.sleep(0.15)
        assert not monitor.is_nav_ready

        critical_snap = OrchestratorStateSnapshot(
            mission_state=MacroMissionState.CHECKING_REACHABILITY,
            perception_context=0,
        )
        monitor.expire_readiness(current_state=critical_snap.mission_state)
        assert not monitor.nav_ready
        assert critical_loss_cb.called

        # 3. Grasp ready message and expiration
        assert not monitor.is_grasp_ready
        monitor.on_grasp_ready_msg(Bool(data=True))
        assert monitor.is_grasp_ready
        assert monitor.grasp_ready

        time.sleep(0.15)
        assert not monitor.is_grasp_ready
    finally:
        monitor.destroy()


def test_open_call_thread_safety(mock_node):
    """Verify external callbacks are executed outside the supervisor's state lock."""
    lock = threading.RLock()
    was_locked_during_callback = None

    def sample_transition_cb(is_ready: bool):
        nonlocal was_locked_during_callback
        was_locked_during_callback = lock._is_owned()

    snapshot = OrchestratorStateSnapshot(
        mission_state=MacroMissionState.WAITING_FOR_TF_READY,
        perception_context=0,
    )

    monitor = HealthSupervisor(
        node=mock_node,
        config=HealthSupervisorConfig(),
        state_lock=lock,
        state_provider=lambda: snapshot,
        on_nav_ready_transition_cb=sample_transition_cb,
    )
    try:
        monitor.on_nav_ready_msg(Bool(data=True))
        assert (
            was_locked_during_callback is False
        ), "Callback was called while holding lock!"
    finally:
        monitor.destroy()


def test_auto_recovery_lifecycle_and_max_attempts(mock_node, dummy_snapshot):
    recovery_cb = MagicMock()
    cfg = HealthSupervisorConfig(
        auto_recovery_enabled=True,
        auto_recovery_timeout_sec=0.01,
        max_recovery_attempts=2,
    )
    monitor = HealthSupervisor(
        node=mock_node,
        config=cfg,
        state_provider=lambda: dummy_snapshot,
        on_auto_recovery_cb=recovery_cb,
    )
    try:
        # Attempt 1
        monitor.schedule_auto_recovery_if_enabled()
        assert monitor.recovery_attempts == 1

        # Attempt 2
        monitor.schedule_auto_recovery_if_enabled()
        assert monitor.recovery_attempts == 2

        # Attempt 3 (exceeded)
        monitor.schedule_auto_recovery_if_enabled()
        assert monitor.recovery_attempts == 2  # not incremented

        # Fire recovery timer callback
        monitor._on_auto_recovery_timer_fired()
        assert recovery_cb.called

        # Reset
        monitor.reset_recovery_attempts()
        assert monitor.recovery_attempts == 0
    finally:
        monitor.destroy()


def test_manual_recover_service_handler(mock_node, dummy_snapshot):
    """Verify manual recovery service handler unlatches operator, invokes reset callback, and resets counter."""
    reset_cb = MagicMock(return_value=True)
    monitor = HealthSupervisor(
        node=mock_node,
        config=HealthSupervisorConfig(),
        state_provider=lambda: dummy_snapshot,
        on_reset_recovery_system_cb=reset_cb,
    )
    try:
        monitor.set_operator_latched(True)
        assert monitor.is_operator_latched

        req = Trigger.Request()
        resp = Trigger.Response()
        result = monitor.handle_recover_service(req, resp, current_state=MacroMissionState.EXECUTING_MOVE_PIPELINE)

        assert result.success is True
        assert reset_cb.called
        assert not monitor.is_operator_latched
        assert monitor.recovery_attempts == 0
    finally:
        monitor.destroy()


def test_diagnostics_and_visualizer_emission(mock_node, dummy_snapshot):
    diag_pub = MagicMock()
    marker_pub = MagicMock()
    mock_node.create_publisher.side_effect = [diag_pub, marker_pub]

    cfg = HealthSupervisorConfig(enable_visualizer=True)
    monitor = HealthSupervisor(
        node=mock_node,
        config=cfg,
        state_provider=lambda: dummy_snapshot,
    )
    try:
        monitor.set_robot_color("b")
        monitor.on_nav_ready_msg(Bool(data=True))
        monitor.on_grasp_ready_msg(Bool(data=True))
        monitor.publish_diagnostics(dummy_snapshot)

        assert diag_pub.publish.called
        assert marker_pub.publish.called

        diag_array = diag_pub.publish.call_args[0][0]
        status = diag_array.status[0]
        assert status.level == DiagnosticStatus.OK

        keys = {kv.key: kv.value for kv in status.values}
        assert keys["nav_ready"] == "True"
        assert keys["grasp_ready"] == "True"
        assert "tf_ready" not in keys
        assert keys["robot_color"] == "b"
        assert keys["last_goal_move"] == "e2e4"

        # Diagnostic ERROR state
        err_snapshot = OrchestratorStateSnapshot(
            mission_state=MacroMissionState.ERROR_FALLBACK,
            perception_context=0,
            last_goal_move="e2e4",
            execution_stage="MOVE",
            motion_state=0,
        )
        monitor.publish_diagnostics(err_snapshot)
        assert diag_pub.publish.call_args[0][0].status[0].level == DiagnosticStatus.ERROR

        # Diagnostic WARN state (unready navigation)
        monitor.nav_ready = False
        warn_snapshot = OrchestratorStateSnapshot(
            mission_state=MacroMissionState.WAITING_FOR_TF_READY,
            perception_context=0,
            last_goal_move="e2e4",
            execution_stage="MOVE",
            motion_state=0,
        )
        monitor.publish_diagnostics(warn_snapshot)
        assert diag_pub.publish.call_args[0][0].status[0].level == DiagnosticStatus.WARN
    finally:
        monitor.destroy()


def test_parameters_invalid_readiness_timeout():
    """Verify that non-positive readiness_timeout_sec raises ValueError."""
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from lekiwi_orchestrator.core.parameters import OrchestratorParameters

    if not rclpy.ok():
        rclpy.init()
    test_node = Node(
        "test_params_node",
        parameter_overrides=[Parameter("readiness_timeout_sec", value=-1.0)],
    )
    try:
        with pytest.raises(
            ValueError, match="readiness_timeout_sec must be finite and positive"
        ):
            OrchestratorParameters(test_node)
    finally:
        test_node.destroy_node()


def _make_odom(
    vx: float = 0.0,
    vy: float = 0.0,
    wz: float = 0.0,
    stamp_sec: float = 100.0,
) -> Odometry:
    msg = Odometry()
    msg.header.stamp.sec = int(stamp_sec)
    msg.header.stamp.nanosec = int((stamp_sec - int(stamp_sec)) * 1e9)
    msg.twist.twist.linear.x = float(vx)
    msg.twist.twist.linear.y = float(vy)
    msg.twist.twist.angular.z = float(wz)
    return msg


def test_uncommanded_motion_disturbance_and_inhibit_recovery(mock_node):
    """Verify non-zero chassis velocity while stationary triggers disturbance and inhibits auto-recovery."""
    disturbance_cb = MagicMock()
    idle_snapshot = OrchestratorStateSnapshot(
        mission_state=MacroMissionState.CHECKING_REACHABILITY,
        perception_context=0,
        motion_state=MotionExecutionState.IDLE,
    )
    monitor = HealthSupervisor(
        node=mock_node,
        config=HealthSupervisorConfig(
            uncommanded_linear_thresh=0.03,
            uncommanded_confirm_duration_sec=0.20,
        ),
        state_provider=lambda: idle_snapshot,
        on_disturbance_detected_cb=disturbance_cb,
    )
    try:
        # 1. Startup phase: before global odom, motion is ignored (startup gate)
        assert not monitor.is_startup_ekf_received
        odom_moving = _make_odom(vx=0.5, stamp_sec=100.0)
        monitor.on_local_odom(odom_moving)
        assert not disturbance_cb.called
        assert not monitor.is_operator_latched

        # 2. Global odom received: unlatches startup gate and destroys subscription
        assert monitor._sub_global_odom is not None
        monitor.on_global_odom(Odometry())
        assert monitor.is_startup_ekf_received
        assert monitor._sub_global_odom is None

        # 3. Zero velocity odometry does not trigger disturbance and reports stationary
        odom_zero = _make_odom(vx=0.0, wz=0.0, stamp_sec=100.0)
        monitor.on_local_odom(odom_zero)
        assert not disturbance_cb.called
        assert not monitor.is_operator_latched
        assert monitor.is_chassis_stationary

        # 4. Instantaneous spike (< 0.20s) is absorbed without latching
        monitor.on_local_odom(_make_odom(vx=0.5, stamp_sec=100.05))
        assert not monitor.is_operator_latched
        assert not disturbance_cb.called

        # 5. Sustained speed exceeding threshold (>= 0.20s) triggers disturbance
        monitor.on_local_odom(_make_odom(vx=0.5, stamp_sec=100.25))
        assert not monitor.is_chassis_stationary
        assert disturbance_cb.called
        assert disturbance_cb.call_args[0][0] == "uncommanded_motion"
        assert monitor.is_operator_latched

        # 6. Auto-recovery timer scheduling is strictly inhibited when latched
        monitor.schedule_auto_recovery_if_enabled()
        assert monitor._recovery_timer is None
    finally:
        monitor.destroy()


def test_valid_navigation_motion_exempt_from_disturbance(mock_node):
    """Verify that commanded chassis motion during navigation (is_navigating=True or NAV motion state) is exempt from disturbance."""
    disturbance_cb = MagicMock()
    current_snap = {"snap": OrchestratorStateSnapshot(
        mission_state=MacroMissionState.POST_MOVE_VERIFYING,
        perception_context=0,
        motion_state=MotionExecutionState.IDLE,
        is_navigating=False,
    )}
    monitor = HealthSupervisor(
        node=mock_node,
        config=HealthSupervisorConfig(
            uncommanded_linear_thresh=0.03,
            uncommanded_confirm_duration_sec=0.20,
        ),
        state_provider=lambda: current_snap["snap"],
        on_disturbance_detected_cb=disturbance_cb,
    )
    try:
        # Arm monitor with global odom
        monitor.on_global_odom(Odometry())
        assert monitor.is_startup_ekf_received

        # 1. When is_navigating=True, moving odom does not trigger disturbance
        current_snap["snap"] = OrchestratorStateSnapshot(
            mission_state=MacroMissionState.POST_MOVE_VERIFYING,
            perception_context=0,
            motion_state=MotionExecutionState.IDLE,
            is_navigating=True,
        )
        monitor.on_local_odom(_make_odom(vx=0.25, stamp_sec=10.0))
        monitor.on_local_odom(_make_odom(vx=0.25, stamp_sec=10.25))
        assert not disturbance_cb.called
        assert not monitor.is_operator_latched

        # 2. When motion_state is NAV_TO_OBS, moving odom does not trigger disturbance
        current_snap["snap"] = OrchestratorStateSnapshot(
            mission_state=MacroMissionState.POST_MOVE_VERIFYING,
            perception_context=0,
            motion_state=MotionExecutionState.NAV_TO_OBS,
            is_navigating=False,
        )
        monitor.on_local_odom(_make_odom(vx=0.25, stamp_sec=10.5))
        monitor.on_local_odom(_make_odom(vx=0.25, stamp_sec=10.75))
        assert not disturbance_cb.called
        assert not monitor.is_operator_latched

        # 3. When IDLE and not navigating, sustained moving odom DOES trigger disturbance
        current_snap["snap"] = OrchestratorStateSnapshot(
            mission_state=MacroMissionState.POST_MOVE_VERIFYING,
            perception_context=0,
            motion_state=MotionExecutionState.IDLE,
            is_navigating=False,
        )
        monitor.on_local_odom(_make_odom(vx=0.25, stamp_sec=11.0))
        assert not disturbance_cb.called
        assert not monitor.is_operator_latched

        monitor.on_local_odom(_make_odom(vx=0.25, stamp_sec=11.25))
        assert disturbance_cb.called
        assert monitor.is_operator_latched
    finally:
        monitor.destroy()


def test_motion_disturbance_spike_vs_sustained(mock_node):
    """Verify transient arm vibration (<0.2s) is ignored while sustained motion (>=0.2s) triggers disturbance."""
    disturbance_cb = MagicMock()
    snapshot = OrchestratorStateSnapshot(
        mission_state=MacroMissionState.EXECUTING_MOVE_PIPELINE,
        perception_context=0,
        motion_state=MotionExecutionState.IDLE,
        execution_stage="PICKING_PIECE",
        is_navigating=False,
    )
    monitor = HealthSupervisor(
        node=mock_node,
        config=HealthSupervisorConfig(
            uncommanded_angular_thresh=0.08,
            uncommanded_confirm_duration_sec=0.20,
        ),
        state_provider=lambda: snapshot,
        on_disturbance_detected_cb=disturbance_cb,
    )
    try:
        monitor.on_global_odom(Odometry())
        assert monitor.is_startup_ekf_received

        # Phase 1: Transient spike (60ms) and dampening
        monitor.on_local_odom(_make_odom(wz=-0.30, stamp_sec=10.00))
        monitor.on_local_odom(_make_odom(wz=-0.30, stamp_sec=10.06))
        assert not disturbance_cb.called
        assert not monitor.is_operator_latched

        monitor.on_local_odom(_make_odom(wz=0.01, stamp_sec=10.10))
        assert not disturbance_cb.called

        # Phase 2: Sustained displacement lasting 250ms triggers disturbance
        monitor.on_local_odom(_make_odom(wz=-0.30, stamp_sec=20.00))
        assert not disturbance_cb.called
        monitor.on_local_odom(_make_odom(wz=-0.30, stamp_sec=20.25))
        assert disturbance_cb.called
        assert disturbance_cb.call_args[0][0] == "uncommanded_motion"
        assert monitor.is_operator_latched
    finally:
        monitor.destroy()

