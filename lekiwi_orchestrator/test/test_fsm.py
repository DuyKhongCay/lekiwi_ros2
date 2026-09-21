# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for LeKiwi State Pattern and FSM rules."""

from lekiwi_interfaces.msg import CameraMode
from lekiwi_orchestrator.fsm import (
    ALLOWED_CAMERA_TRANSITIONS,
    ALLOWED_MISSION_TRANSITIONS,
    MissionState,
    is_camera_transition_allowed,
    is_mission_transition_allowed,
    is_transition_allowed,
)


def test_canonical_camera_sequence_is_allowed():
    sequence = [
        CameraMode.STANDBY,
        CameraMode.NAVIGATING,
        CameraMode.CHESS_THINKING,
        CameraMode.MANIPULATION_LEROBOT,
        CameraMode.CHESS_THINKING,
        CameraMode.STANDBY,
    ]
    assert all(
        is_camera_transition_allowed(current, requested)
        for current, requested in zip(sequence, sequence[1:])
    )


def test_invalid_camera_shortcut_is_rejected():
    assert not is_camera_transition_allowed(
        CameraMode.STANDBY, CameraMode.MANIPULATION_LEROBOT
    )


def test_backward_compatibility_alias():
    assert is_transition_allowed(CameraMode.STANDBY, CameraMode.NAVIGATING)
    assert not is_transition_allowed(
        CameraMode.STANDBY, CameraMode.MANIPULATION_LEROBOT
    )


def test_mission_fsm_canonical_workflow():
    sequence = [
        MissionState.BOOT_INITIALIZING,
        MissionState.WAITING_FOR_TF_READY,
        MissionState.WAITING_FOR_PLAYER_MOVE,
        MissionState.EVALUATING_BEST_MOVE,
        MissionState.CHECKING_REACHABILITY,
        MissionState.NAVIGATING_TO_STANDOFF,
        MissionState.EXECUTING_MANIPULATION,
        MissionState.TURN_COMPLETED,
        MissionState.WAITING_FOR_PLAYER_MOVE,
    ]
    assert all(
        is_mission_transition_allowed(curr, next_st)
        for curr, next_st in zip(sequence, sequence[1:])
    )


def test_mission_fsm_zero_nav_shortcut():
    # PLAN_ZERO_NAV allows moving directly from CHECKING_REACHABILITY to EXECUTING_MANIPULATION
    assert is_mission_transition_allowed(
        MissionState.CHECKING_REACHABILITY, MissionState.EXECUTING_MANIPULATION
    )


def test_mission_fsm_illegal_transitions():
    # Cannot jump straight from BOOT_INITIALIZING to EXECUTING_MANIPULATION
    assert not is_mission_transition_allowed(
        MissionState.BOOT_INITIALIZING, MissionState.EXECUTING_MANIPULATION
    )
    # Cannot jump from WAITING_FOR_TF_READY to NAVIGATING_TO_STANDOFF
    assert not is_mission_transition_allowed(
        MissionState.WAITING_FOR_TF_READY, MissionState.NAVIGATING_TO_STANDOFF
    )
