# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""
State Pattern & Finite State Machine (FSM) Definitions for LeKiwi.

Encapsulates legal transitions for:
1. High-level Autonomous Chess Mission (`MissionState`).
2. Vision Pipeline Operational Modes (`CameraMode`).
"""

from __future__ import annotations

from enum import IntEnum
from typing import Dict, Set

from lekiwi_interfaces.msg import CameraMode


class MissionState(IntEnum):
    """Lifecycle states of the autonomous chess-playing robot mission."""

    BOOT_INITIALIZING = 0
    WAITING_FOR_TF_READY = 1
    WAITING_FOR_PLAYER_MOVE = 2
    EVALUATING_BEST_MOVE = 3
    CHECKING_REACHABILITY = 4
    NAVIGATING_TO_STANDOFF = 5
    EXECUTING_MANIPULATION = 6
    TURN_COMPLETED = 7
    GAME_OVER = 8
    ERROR_FALLBACK = 9


MISSION_STATE_NAMES: Dict[MissionState, str] = {
    MissionState.BOOT_INITIALIZING: "BOOT_INITIALIZING",
    MissionState.WAITING_FOR_TF_READY: "WAITING_FOR_TF_READY",
    MissionState.WAITING_FOR_PLAYER_MOVE: "WAITING_FOR_PLAYER_MOVE",
    MissionState.EVALUATING_BEST_MOVE: "EVALUATING_BEST_MOVE",
    MissionState.CHECKING_REACHABILITY: "CHECKING_REACHABILITY",
    MissionState.NAVIGATING_TO_STANDOFF: "NAVIGATING_TO_STANDOFF",
    MissionState.EXECUTING_MANIPULATION: "EXECUTING_MANIPULATION",
    MissionState.TURN_COMPLETED: "TURN_COMPLETED",
    MissionState.GAME_OVER: "GAME_OVER",
    MissionState.ERROR_FALLBACK: "ERROR_FALLBACK",
}

# Permitted state transitions for MissionState
ALLOWED_MISSION_TRANSITIONS: Dict[MissionState, Set[MissionState]] = {
    MissionState.BOOT_INITIALIZING: {
        MissionState.BOOT_INITIALIZING,
        MissionState.WAITING_FOR_TF_READY,
        MissionState.ERROR_FALLBACK,
    },
    MissionState.WAITING_FOR_TF_READY: {
        MissionState.WAITING_FOR_TF_READY,
        MissionState.WAITING_FOR_PLAYER_MOVE,
        MissionState.EVALUATING_BEST_MOVE,  # In case robot moves first (White)
        MissionState.ERROR_FALLBACK,
    },
    MissionState.WAITING_FOR_PLAYER_MOVE: {
        MissionState.WAITING_FOR_PLAYER_MOVE,
        MissionState.EVALUATING_BEST_MOVE,
        MissionState.GAME_OVER,
        MissionState.ERROR_FALLBACK,
    },
    MissionState.EVALUATING_BEST_MOVE: {
        MissionState.EVALUATING_BEST_MOVE,
        MissionState.CHECKING_REACHABILITY,
        MissionState.GAME_OVER,
        MissionState.ERROR_FALLBACK,
    },
    MissionState.CHECKING_REACHABILITY: {
        MissionState.CHECKING_REACHABILITY,
        MissionState.NAVIGATING_TO_STANDOFF,
        MissionState.EXECUTING_MANIPULATION,  # If PLAN_ZERO_NAV
        MissionState.ERROR_FALLBACK,
    },
    MissionState.NAVIGATING_TO_STANDOFF: {
        MissionState.NAVIGATING_TO_STANDOFF,
        MissionState.EXECUTING_MANIPULATION,
        MissionState.ERROR_FALLBACK,
    },
    MissionState.EXECUTING_MANIPULATION: {
        MissionState.EXECUTING_MANIPULATION,
        MissionState.NAVIGATING_TO_STANDOFF,  # If dual-base: nav to place standoff
        MissionState.TURN_COMPLETED,
        MissionState.ERROR_FALLBACK,
    },
    MissionState.TURN_COMPLETED: {
        MissionState.TURN_COMPLETED,
        MissionState.WAITING_FOR_PLAYER_MOVE,
        MissionState.GAME_OVER,
        MissionState.ERROR_FALLBACK,
    },
    MissionState.GAME_OVER: {
        MissionState.GAME_OVER,
        MissionState.BOOT_INITIALIZING,  # Reset game
    },
    MissionState.ERROR_FALLBACK: {
        MissionState.ERROR_FALLBACK,
        MissionState.BOOT_INITIALIZING,
        MissionState.WAITING_FOR_TF_READY,
    },
}

# Permitted state transitions for CameraMode
ALLOWED_CAMERA_TRANSITIONS: Dict[int, Set[int]] = {
    CameraMode.STANDBY: {
        CameraMode.STANDBY,
        CameraMode.NAVIGATING,
        CameraMode.CHESS_THINKING,
    },
    CameraMode.NAVIGATING: {
        CameraMode.STANDBY,
        CameraMode.NAVIGATING,
        CameraMode.CHESS_THINKING,
        CameraMode.MANIPULATION_LEROBOT,
    },
    CameraMode.CHESS_THINKING: {
        CameraMode.STANDBY,
        CameraMode.NAVIGATING,
        CameraMode.CHESS_THINKING,
        CameraMode.MANIPULATION_LEROBOT,
    },
    CameraMode.MANIPULATION_LEROBOT: {
        CameraMode.STANDBY,
        CameraMode.NAVIGATING,
        CameraMode.CHESS_THINKING,
        CameraMode.MANIPULATION_LEROBOT,
    },
}

CAMERA_MODE_NAMES: Dict[int, str] = {
    CameraMode.STANDBY: "STANDBY",
    CameraMode.NAVIGATING: "NAVIGATING",
    CameraMode.CHESS_THINKING: "CHESS_THINKING",
    CameraMode.MANIPULATION_LEROBOT: "MANIPULATION_LEROBOT",
}

# Backward compatibility aliases for existing lekiwi_control tests
ALLOWED_TRANSITIONS = ALLOWED_CAMERA_TRANSITIONS
MODE_NAMES = CAMERA_MODE_NAMES


def is_mission_transition_allowed(
    current_state: MissionState, requested_state: MissionState
) -> bool:
    """Return True if the mission FSM permits transition from current_state to requested_state."""
    return requested_state in ALLOWED_MISSION_TRANSITIONS.get(current_state, set())


def is_camera_transition_allowed(current_mode: int, requested_mode: int) -> bool:
    """Return True if the camera mode FSM permits transition from current_mode to requested_mode."""
    return requested_mode in ALLOWED_CAMERA_TRANSITIONS.get(current_mode, set())


# Backward compatibility alias
def is_transition_allowed(current_mode: int, requested_mode: int) -> bool:
    """Alias for is_camera_transition_allowed."""
    return is_camera_transition_allowed(current_mode, requested_mode)
