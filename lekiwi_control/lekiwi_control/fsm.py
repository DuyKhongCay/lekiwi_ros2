# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

from lekiwi_interfaces.msg import CameraMode


ALLOWED_TRANSITIONS = {
    CameraMode.STANDBY: {CameraMode.STANDBY, CameraMode.NAVIGATING},
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

MODE_NAMES = {
    CameraMode.STANDBY: 'STANDBY',
    CameraMode.NAVIGATING: 'NAVIGATING',
    CameraMode.CHESS_THINKING: 'CHESS_THINKING',
    CameraMode.MANIPULATION_LEROBOT: 'MANIPULATION_LEROBOT',
}


def is_transition_allowed(current_mode, requested_mode):
    """Return whether the orchestrator FSM permits a mode transition."""
    return requested_mode in ALLOWED_TRANSITIONS.get(current_mode, set())
