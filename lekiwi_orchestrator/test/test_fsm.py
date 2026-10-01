# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for LeKiwi 2-Level Hierarchical State Pattern, Micro Motion FSM, and PerceptionContext."""

import itertools

from lekiwi_interfaces.msg import PerceptionContext
from lekiwi_orchestrator.fsm import (
    MacroMissionState,
    MissionState,
    MotionExecutionState,
    is_mission_transition_allowed,
    is_motion_transition_allowed,
    is_perception_transition_allowed,
)


def test_level1_macro_mission_canonical_workflow():
    sequence = [
        MacroMissionState.BOOT_INITIALIZING,
        MacroMissionState.WAITING_FOR_TF_READY,
        MacroMissionState.WAITING_FOR_PLAYER_MOVE,
        MacroMissionState.EVALUATING_BEST_MOVE,
        MacroMissionState.CHECKING_REACHABILITY,
        MacroMissionState.EXECUTING_MOVE_PIPELINE,
        MacroMissionState.TURN_COMPLETED,
        MacroMissionState.WAITING_FOR_PLAYER_MOVE,
    ]
    assert all(
        is_mission_transition_allowed(curr, next_st)
        for curr, next_st in itertools.pairwise(sequence)
    )


def test_level1_macro_mission_white_robot_shortcut():
    # Robot White plays first, going from WAITING_FOR_TF_READY to EVALUATING_BEST_MOVE
    assert is_mission_transition_allowed(
        MacroMissionState.WAITING_FOR_TF_READY, MacroMissionState.EVALUATING_BEST_MOVE
    )


def test_level1_macro_mission_illegal_transitions():
    # Cannot jump straight from BOOT_INITIALIZING to EXECUTING_MOVE_PIPELINE
    assert not is_mission_transition_allowed(
        MacroMissionState.BOOT_INITIALIZING, MacroMissionState.EXECUTING_MOVE_PIPELINE
    )
    # Cannot jump from WAITING_FOR_TF_READY to EXECUTING_MOVE_PIPELINE
    assert not is_mission_transition_allowed(
        MacroMissionState.WAITING_FOR_TF_READY,
        MacroMissionState.EXECUTING_MOVE_PIPELINE,
    )


def test_level2_motion_sub_fsm_canonical_workflow():
    # Dual-base capture sequence
    sequence = [
        MotionExecutionState.IDLE,
        MotionExecutionState.NAV_TO_CLEAR,
        MotionExecutionState.CLEARING_PIECE,
        MotionExecutionState.NAV_TO_PICK,
        MotionExecutionState.PICKING_PIECE,
        MotionExecutionState.NAV_TO_PLACE,
        MotionExecutionState.PLACING_PIECE,
        MotionExecutionState.POST_MOVE_VERIFYING,
        MotionExecutionState.IDLE,
    ]
    assert all(
        is_motion_transition_allowed(curr, next_st)
        for curr, next_st in itertools.pairwise(sequence)
    )


def test_level2_motion_sub_fsm_zero_nav_workflow():
    sequence = [
        MotionExecutionState.IDLE,
        MotionExecutionState.PICKING_PIECE,
        MotionExecutionState.PLACING_PIECE,
        MotionExecutionState.POST_MOVE_VERIFYING,
        MotionExecutionState.IDLE,
    ]
    assert all(
        is_motion_transition_allowed(curr, next_st)
        for curr, next_st in itertools.pairwise(sequence)
    )


def test_level2_motion_sub_fsm_illegal_transitions():
    # Cannot jump straight from IDLE to PLACING_PIECE without picking first
    assert not is_motion_transition_allowed(
        MotionExecutionState.IDLE, MotionExecutionState.PLACING_PIECE
    )
    # Cannot jump from NAV_TO_CLEAR to PLACING_PIECE
    assert not is_motion_transition_allowed(
        MotionExecutionState.NAV_TO_CLEAR, MotionExecutionState.PLACING_PIECE
    )


def test_perception_context_canonical_sequence():
    sequence = [
        PerceptionContext.IDLE_STANDBY,
        PerceptionContext.TF_TRACKING_AND_NAV,
        PerceptionContext.BOARD_STATE_SCAN,
        PerceptionContext.MANIPULATION_ACTOR,
        PerceptionContext.POST_MOVE_VERIFY,
        PerceptionContext.BOARD_STATE_SCAN,
        PerceptionContext.IDLE_STANDBY,
    ]
    assert all(
        is_perception_transition_allowed(current, requested)
        for current, requested in itertools.pairwise(sequence)
    )


def test_perception_context_illegal_transition():
    # Cannot jump straight from IDLE_STANDBY to MANIPULATION_ACTOR
    assert not is_perception_transition_allowed(
        PerceptionContext.IDLE_STANDBY, PerceptionContext.MANIPULATION_ACTOR
    )
    # Cannot jump from POST_MOVE_VERIFY to CALIBRATION_STREAM
    assert not is_perception_transition_allowed(
        PerceptionContext.POST_MOVE_VERIFY, PerceptionContext.CALIBRATION_STREAM
    )


def test_backward_compatibility_aliases():
    # MissionState is canonical alias of MacroMissionState
    assert MissionState == MacroMissionState
