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
from lekiwi_orchestrator.fsm import MacroMissionState
from lekiwi_orchestrator.health_supervisor import (
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
        mission_state=MacroMissionState.WAITING_FOR_PLAYER_MOVE,
        perception_context=0,
        last_goal_move="e2e4",
        execution_stage=None,
    )


@pytest.fixture
def mock_node():
    node = MagicMock()
    clock = MagicMock()
    time_msg = Time()
    time_msg.sec = 100
    clock.now.return_value.to_msg.return_value = time_msg
    node.get_clock.return_value = clock
    return node


def test_initial_state_not_ready(mock_node, dummy_snapshot):
    monitor = HealthSupervisor(
        node=mock_node,
        config=HealthSupervisorConfig(),
        state_provider=lambda: dummy_snapshot,
    )
    try:
        assert not monitor.is_nav_ready
        assert not monitor.is_grasp_ready
        assert not monitor.nav_ready
        assert not monitor.grasp_ready
    finally:
        monitor.destroy()


def test_nav_ready_lease_and_expiration(mock_node, dummy_snapshot):
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
        # 1. Emit nav ready
        monitor.on_nav_ready_msg(Bool(data=True))
        assert monitor.is_nav_ready
        assert monitor.nav_ready
        assert transition_cb.called

        # 2. Simulate timeout
        time.sleep(0.15)
        assert not monitor.is_nav_ready

        # 3. Expire lease
        critical_snap = OrchestratorStateSnapshot(
            mission_state=MacroMissionState.CHECKING_REACHABILITY,
            perception_context=0,
        )
        monitor.expire_readiness(current_state=critical_snap.mission_state)
        assert not monitor.nav_ready
        assert critical_loss_cb.called
    finally:
        monitor.destroy()


def test_grasp_ready_lease_and_expiration(mock_node, dummy_snapshot):
    monitor = HealthSupervisor(
        node=mock_node,
        config=HealthSupervisorConfig(readiness_timeout_sec=0.1),
        state_provider=lambda: dummy_snapshot,
    )
    try:
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
    reset_cb = MagicMock(return_value=True)
    monitor = HealthSupervisor(
        node=mock_node,
        config=HealthSupervisorConfig(),
        state_provider=lambda: dummy_snapshot,
        on_reset_recovery_system_cb=reset_cb,
    )
    try:
        req = Trigger.Request()
        resp = Trigger.Response()
        result = monitor.handle_recover_service(req, resp)

        assert result.success is True
        assert reset_cb.called
        assert monitor.recovery_attempts == 0
    finally:
        monitor.destroy()


def test_diagnostics_and_visualizer_emission(mock_node, dummy_snapshot):
    diag_pub = MagicMock()
    marker_pub = MagicMock()
    mock_node.create_publisher.side_effect = [diag_pub, marker_pub]

    cfg = HealthSupervisorConfig(robot_color="b", enable_visualizer=True)
    monitor = HealthSupervisor(
        node=mock_node,
        config=cfg,
        state_provider=lambda: dummy_snapshot,
    )
    try:
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
    from lekiwi_orchestrator.parameters import OrchestratorParameters

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

