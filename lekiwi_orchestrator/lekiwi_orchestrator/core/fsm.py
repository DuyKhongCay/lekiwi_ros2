# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""
Hierarchical State Pattern & Finite State Machine (HFSM) Definitions for LeKiwi.

Architecture:
1. Level 1: Macro Mission FSM (`MacroMissionState`)
   Manages game turn ownership, localization safety, move planning, and game outcome.
2. Level 2: Motion Execution Sub-FSM (`MotionExecutionState`)
   Semantic micro-steps decomposing the move (Clear -> Pick -> Place -> Verify).
3. Perception Context (`PerceptionContext`)
   Hardware-aware perception gating for GStreamer valves, NPU Hailo-8, and LeRobot VLA.
"""

from __future__ import annotations

from enum import IntEnum

from lekiwi_interfaces.msg import PerceptionContext


class MacroMissionState(IntEnum):
    """Level 1: High-level autonomous chess mission lifecycle states.

    Tracks high-level turn progression, safety leases, and recovery states.
    """

    BOOT_INITIALIZING = 0
    WAITING_FOR_TF_READY = 1
    WAITING_FOR_PLAYER_MOVE = 2
    EVALUATING_BEST_MOVE = 3
    CHECKING_REACHABILITY = 4
    EXECUTING_MOVE_PIPELINE = 5
    POST_MOVE_VERIFYING = 6
    TURN_COMPLETED = 7
    GAME_OVER = 8
    ERROR_FALLBACK = 9


MISSION_STATE_NAMES: dict[MacroMissionState, str] = {
    MacroMissionState.BOOT_INITIALIZING: "BOOT_INITIALIZING",
    MacroMissionState.WAITING_FOR_TF_READY: "WAITING_FOR_TF_READY",
    MacroMissionState.WAITING_FOR_PLAYER_MOVE: "WAITING_FOR_PLAYER_MOVE",
    MacroMissionState.EVALUATING_BEST_MOVE: "EVALUATING_BEST_MOVE",
    MacroMissionState.CHECKING_REACHABILITY: "CHECKING_REACHABILITY",
    MacroMissionState.EXECUTING_MOVE_PIPELINE: "EXECUTING_MOVE_PIPELINE",
    MacroMissionState.POST_MOVE_VERIFYING: "POST_MOVE_VERIFYING",
    MacroMissionState.TURN_COMPLETED: "TURN_COMPLETED",
    MacroMissionState.GAME_OVER: "GAME_OVER",
    MacroMissionState.ERROR_FALLBACK: "ERROR_FALLBACK",
}

# Permitted state transitions for Level 1 MacroMissionState
ALLOWED_MISSION_TRANSITIONS: dict[MacroMissionState, set[MacroMissionState]] = {
    MacroMissionState.BOOT_INITIALIZING: {
        MacroMissionState.BOOT_INITIALIZING,
        MacroMissionState.WAITING_FOR_TF_READY,
        MacroMissionState.ERROR_FALLBACK,
    },
    MacroMissionState.WAITING_FOR_TF_READY: {
        MacroMissionState.WAITING_FOR_TF_READY,
        MacroMissionState.WAITING_FOR_PLAYER_MOVE,
        MacroMissionState.EVALUATING_BEST_MOVE,  # Robot White plays first or recovering move
        MacroMissionState.CHECKING_REACHABILITY,  # Direct move dispatch upon recovery
        MacroMissionState.ERROR_FALLBACK,
    },
    MacroMissionState.WAITING_FOR_PLAYER_MOVE: {
        MacroMissionState.WAITING_FOR_PLAYER_MOVE,
        MacroMissionState.EVALUATING_BEST_MOVE,
        MacroMissionState.GAME_OVER,
        MacroMissionState.ERROR_FALLBACK,
    },
    MacroMissionState.EVALUATING_BEST_MOVE: {
        MacroMissionState.EVALUATING_BEST_MOVE,
        MacroMissionState.CHECKING_REACHABILITY,
        MacroMissionState.GAME_OVER,
        MacroMissionState.ERROR_FALLBACK,
    },
    MacroMissionState.CHECKING_REACHABILITY: {
        MacroMissionState.CHECKING_REACHABILITY,
        MacroMissionState.EXECUTING_MOVE_PIPELINE,
        MacroMissionState.ERROR_FALLBACK,
    },
    MacroMissionState.EXECUTING_MOVE_PIPELINE: {
        MacroMissionState.EXECUTING_MOVE_PIPELINE,
        MacroMissionState.POST_MOVE_VERIFYING,
        MacroMissionState.TURN_COMPLETED,
        MacroMissionState.WAITING_FOR_TF_READY,
        MacroMissionState.ERROR_FALLBACK,
    },
    MacroMissionState.POST_MOVE_VERIFYING: {
        MacroMissionState.POST_MOVE_VERIFYING,
        MacroMissionState.TURN_COMPLETED,
        MacroMissionState.ERROR_FALLBACK,
    },
    MacroMissionState.TURN_COMPLETED: {
        MacroMissionState.TURN_COMPLETED,
        MacroMissionState.WAITING_FOR_PLAYER_MOVE,
        MacroMissionState.GAME_OVER,
        MacroMissionState.ERROR_FALLBACK,
    },
    MacroMissionState.GAME_OVER: {
        MacroMissionState.GAME_OVER,
        MacroMissionState.BOOT_INITIALIZING,  # Reset game upon new match
    },
    MacroMissionState.ERROR_FALLBACK: {
        MacroMissionState.ERROR_FALLBACK,
        MacroMissionState.BOOT_INITIALIZING,
        MacroMissionState.WAITING_FOR_TF_READY,
    },
}


class MotionExecutionState(IntEnum):
    """Level 2: Semantic micro-steps inside EXECUTING_MOVE_PIPELINE.

    Sequences approach navigation, piece clearance, grasp, placement, and retreat.
    """

    IDLE = 0
    NAV_TO_CLEAR = 1
    CLEARING_PIECE = 2
    NAV_TO_PICK = 3
    PICKING_PIECE = 4
    NAV_TO_PLACE = 5
    PLACING_PIECE = 6
    NAV_TO_OBS = 7


MOTION_STATE_NAMES: dict[MotionExecutionState, str] = {
    MotionExecutionState.IDLE: "IDLE",
    MotionExecutionState.NAV_TO_CLEAR: "NAV_TO_CLEAR",
    MotionExecutionState.CLEARING_PIECE: "CLEARING_PIECE",
    MotionExecutionState.NAV_TO_PICK: "NAV_TO_PICK",
    MotionExecutionState.PICKING_PIECE: "PICKING_PIECE",
    MotionExecutionState.NAV_TO_PLACE: "NAV_TO_PLACE",
    MotionExecutionState.PLACING_PIECE: "PLACING_PIECE",
    MotionExecutionState.NAV_TO_OBS: "NAV_TO_OBS",
}

# Permitted state transitions for Level 2 MotionExecutionState
ALLOWED_MOTION_TRANSITIONS: dict[MotionExecutionState, set[MotionExecutionState]] = {
    MotionExecutionState.IDLE: {
        MotionExecutionState.IDLE,
        MotionExecutionState.NAV_TO_CLEAR,
        MotionExecutionState.CLEARING_PIECE,
        MotionExecutionState.NAV_TO_PICK,
        MotionExecutionState.PICKING_PIECE,
        MotionExecutionState.NAV_TO_OBS,
    },
    MotionExecutionState.NAV_TO_CLEAR: {
        MotionExecutionState.NAV_TO_CLEAR,
        MotionExecutionState.CLEARING_PIECE,
        MotionExecutionState.NAV_TO_OBS,
        MotionExecutionState.IDLE,
    },
    MotionExecutionState.CLEARING_PIECE: {
        MotionExecutionState.CLEARING_PIECE,
        MotionExecutionState.NAV_TO_PICK,
        MotionExecutionState.PICKING_PIECE,
        MotionExecutionState.NAV_TO_OBS,
        MotionExecutionState.IDLE,
    },
    MotionExecutionState.NAV_TO_PICK: {
        MotionExecutionState.NAV_TO_PICK,
        MotionExecutionState.PICKING_PIECE,
        MotionExecutionState.NAV_TO_OBS,
        MotionExecutionState.IDLE,
    },
    MotionExecutionState.PICKING_PIECE: {
        MotionExecutionState.PICKING_PIECE,
        MotionExecutionState.NAV_TO_PLACE,
        MotionExecutionState.PLACING_PIECE,
        MotionExecutionState.NAV_TO_OBS,
        MotionExecutionState.IDLE,
    },
    MotionExecutionState.NAV_TO_PLACE: {
        MotionExecutionState.NAV_TO_PLACE,
        MotionExecutionState.PLACING_PIECE,
        MotionExecutionState.NAV_TO_OBS,
        MotionExecutionState.IDLE,
    },
    MotionExecutionState.PLACING_PIECE: {
        MotionExecutionState.PLACING_PIECE,
        MotionExecutionState.NAV_TO_OBS,
        MotionExecutionState.IDLE,
    },
    MotionExecutionState.NAV_TO_OBS: {
        MotionExecutionState.NAV_TO_OBS,
        MotionExecutionState.NAV_TO_CLEAR,
        MotionExecutionState.CLEARING_PIECE,
        MotionExecutionState.NAV_TO_PICK,
        MotionExecutionState.PICKING_PIECE,
        MotionExecutionState.NAV_TO_PLACE,
        MotionExecutionState.PLACING_PIECE,
        MotionExecutionState.IDLE,
    },
}

# Perception Context State Names
PERCEPTION_CONTEXT_NAMES: dict[int, str] = {
    PerceptionContext.IDLE_STANDBY: "IDLE_STANDBY",
    PerceptionContext.TF_TRACKING_AND_NAV: "TF_TRACKING_AND_NAV",
    PerceptionContext.BOARD_STATE_SCAN: "BOARD_STATE_SCAN",
    PerceptionContext.MANIPULATION_ACTOR: "MANIPULATION_ACTOR",
    PerceptionContext.POST_MOVE_VERIFY: "POST_MOVE_VERIFY",
    PerceptionContext.CALIBRATION_STREAM: "CALIBRATION_STREAM",
}

# Permitted state transitions for PerceptionContext
ALLOWED_PERCEPTION_TRANSITIONS: dict[int, set[int]] = {
    PerceptionContext.IDLE_STANDBY: {
        PerceptionContext.IDLE_STANDBY,
        PerceptionContext.TF_TRACKING_AND_NAV,
        PerceptionContext.BOARD_STATE_SCAN,
        PerceptionContext.CALIBRATION_STREAM,
    },
    PerceptionContext.TF_TRACKING_AND_NAV: {
        PerceptionContext.IDLE_STANDBY,
        PerceptionContext.TF_TRACKING_AND_NAV,
        PerceptionContext.BOARD_STATE_SCAN,
        PerceptionContext.MANIPULATION_ACTOR,
    },
    PerceptionContext.BOARD_STATE_SCAN: {
        PerceptionContext.IDLE_STANDBY,
        PerceptionContext.TF_TRACKING_AND_NAV,
        PerceptionContext.BOARD_STATE_SCAN,
        PerceptionContext.MANIPULATION_ACTOR,
    },
    PerceptionContext.MANIPULATION_ACTOR: {
        PerceptionContext.IDLE_STANDBY,
        PerceptionContext.TF_TRACKING_AND_NAV,
        PerceptionContext.POST_MOVE_VERIFY,
        PerceptionContext.BOARD_STATE_SCAN,
        PerceptionContext.MANIPULATION_ACTOR,
    },
    PerceptionContext.POST_MOVE_VERIFY: {
        PerceptionContext.IDLE_STANDBY,
        PerceptionContext.TF_TRACKING_AND_NAV,
        PerceptionContext.BOARD_STATE_SCAN,
        PerceptionContext.POST_MOVE_VERIFY,
    },
    PerceptionContext.CALIBRATION_STREAM: {
        PerceptionContext.IDLE_STANDBY,
        PerceptionContext.CALIBRATION_STREAM,
    },
}


def is_mission_transition_allowed(
    current_state: MacroMissionState, requested_state: MacroMissionState
) -> bool:
    """Verify whether a Level 1 MacroMissionState transition is valid.

    Guards against illegal FSM jumps to enforce safety invariants.
    """
    return requested_state in ALLOWED_MISSION_TRANSITIONS.get(current_state, set())


def is_motion_transition_allowed(
    current_state: MotionExecutionState, requested_state: MotionExecutionState
) -> bool:
    """Verify whether a Level 2 MotionExecutionState transition is valid.

    Ensures pipeline step sequencing respects physical arm and base safety rules.
    """
    return requested_state in ALLOWED_MOTION_TRANSITIONS.get(current_state, set())


def is_perception_transition_allowed(
    current_context: int, requested_context: int
) -> bool:
    """Verify whether a PerceptionContext hardware valve transition is valid.

    Protects camera pipeline switches between navigation, detection, and manipulation.
    """
    return requested_context in ALLOWED_PERCEPTION_TRANSITIONS.get(
        current_context, set()
    )
