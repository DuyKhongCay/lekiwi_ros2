# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

from lekiwi_control.fsm import is_transition_allowed
from lekiwi_interfaces.msg import CameraMode


def test_canonical_sequence_is_allowed():
    sequence = [
        CameraMode.STANDBY,
        CameraMode.NAVIGATING,
        CameraMode.CHESS_THINKING,
        CameraMode.MANIPULATION_LEROBOT,
        CameraMode.CHESS_THINKING,
        CameraMode.STANDBY,
    ]
    assert all(
        is_transition_allowed(current, requested)
        for current, requested in zip(sequence, sequence[1:])
    )


def test_invalid_shortcut_is_rejected():
    assert not is_transition_allowed(
        CameraMode.STANDBY, CameraMode.MANIPULATION_LEROBOT
    )
