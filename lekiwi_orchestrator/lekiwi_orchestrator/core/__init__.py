# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Core layer containing domain models, state machines, parameter schemas, and math/time utilities."""

from lekiwi_orchestrator.core.fsm import (
    ALLOWED_MISSION_TRANSITIONS,
    ALLOWED_MOTION_TRANSITIONS,
    ALLOWED_PERCEPTION_TRANSITIONS,
    MISSION_STATE_NAMES,
    MOTION_STATE_NAMES,
    PERCEPTION_CONTEXT_NAMES,
    MacroMissionState,
    MotionExecutionState,
    PerceptionContext,
    is_mission_transition_allowed,
    is_motion_transition_allowed,
    is_perception_transition_allowed,
)
from lekiwi_orchestrator.core.mission_types import (
    ActionResult,
    ChessMoveGoal,
    ObservationIntent,
    parse_piece_from_fen,
)
from lekiwi_orchestrator.core.parameters import OrchestratorParameters
from lekiwi_orchestrator.core.utils import (
    create_pose2d_stamped,
    get_header_stamp_sec,
    get_node_now_sec,
    normalize_angle,
    yaw_to_quaternion,
)

__all__ = [
    # FSM
    "ALLOWED_MISSION_TRANSITIONS",
    "ALLOWED_MOTION_TRANSITIONS",
    "ALLOWED_PERCEPTION_TRANSITIONS",
    "MISSION_STATE_NAMES",
    "MOTION_STATE_NAMES",
    "PERCEPTION_CONTEXT_NAMES",
    "MacroMissionState",
    "MotionExecutionState",
    "PerceptionContext",
    "is_mission_transition_allowed",
    "is_motion_transition_allowed",
    "is_perception_transition_allowed",
    # Mission Types
    "ActionResult",
    "ChessMoveGoal",
    "ObservationIntent",
    "parse_piece_from_fen",
    # Parameters
    "OrchestratorParameters",
    # Utilities
    "create_pose2d_stamped",
    "get_header_stamp_sec",
    "get_node_now_sec",
    "normalize_angle",
    "yaw_to_quaternion",
]

