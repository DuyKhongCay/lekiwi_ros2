# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for LeKiwi 2-Level Hierarchical State Pattern, Micro Motion FSM, and PerceptionContext."""

import itertools

from lekiwi_interfaces.msg import PerceptionContext
from lekiwi_orchestrator.core.fsm import (
    MacroMissionState,
    MotionExecutionState,
    is_mission_transition_allowed,
    is_motion_transition_allowed,
    is_perception_transition_allowed,
)


def test_level1_macro_mission_canonical_workflow():
    sequence = [
        MacroMissionState.BOOT_INITIALIZING,
        MacroMissionState.WAITING_FOR_TF_READY,
        MacroMissionState.AWAITING_BOARD_SYNC,
        MacroMissionState.WAITING_FOR_NEXT_MOVE,
        MacroMissionState.EVALUATING_BEST_MOVE,
        MacroMissionState.CHECKING_REACHABILITY,
        MacroMissionState.EXECUTING_MOVE_PIPELINE,
        MacroMissionState.POST_MOVE_VERIFYING,
        MacroMissionState.TURN_COMPLETED,
        MacroMissionState.WAITING_FOR_NEXT_MOVE,
    ]
    assert all(
        is_mission_transition_allowed(curr, next_st)
        for curr, next_st in itertools.pairwise(sequence)
    )


def test_level1_macro_mission_awaiting_board_sync_transitions():
    from lekiwi_orchestrator.core.fsm import IMPORT_FEN_ALLOWED_STATES

    # All allowed states can transition to AWAITING_BOARD_SYNC
    for state in IMPORT_FEN_ALLOWED_STATES:
        assert is_mission_transition_allowed(
            state, MacroMissionState.AWAITING_BOARD_SYNC
        ), f"State {state.name} should allow transition to AWAITING_BOARD_SYNC"

    # Moving or initializing states cannot transition directly to AWAITING_BOARD_SYNC
    disallowed_states = [
        MacroMissionState.BOOT_INITIALIZING,
        MacroMissionState.CHECKING_REACHABILITY,
        MacroMissionState.EXECUTING_MOVE_PIPELINE,
        MacroMissionState.POST_MOVE_VERIFYING,
    ]
    for state in disallowed_states:
        assert not is_mission_transition_allowed(
            state, MacroMissionState.AWAITING_BOARD_SYNC
        ), f"State {state.name} should NOT allow transition to AWAITING_BOARD_SYNC"

    # From AWAITING_BOARD_SYNC, transitions to NEXT_MOVE, EVALUATING, GAME_OVER, TF_READY, ERROR_FALLBACK are allowed
    assert is_mission_transition_allowed(
        MacroMissionState.AWAITING_BOARD_SYNC, MacroMissionState.WAITING_FOR_NEXT_MOVE
    )
    assert is_mission_transition_allowed(
        MacroMissionState.AWAITING_BOARD_SYNC, MacroMissionState.EVALUATING_BEST_MOVE
    )
    assert is_mission_transition_allowed(
        MacroMissionState.AWAITING_BOARD_SYNC, MacroMissionState.GAME_OVER
    )
    assert is_mission_transition_allowed(
        MacroMissionState.AWAITING_BOARD_SYNC, MacroMissionState.RELOCALIZE
    )
    assert is_mission_transition_allowed(
        MacroMissionState.AWAITING_BOARD_SYNC, MacroMissionState.ERROR_FALLBACK
    )
    # But cannot jump straight into physical execution
    assert not is_mission_transition_allowed(
        MacroMissionState.AWAITING_BOARD_SYNC, MacroMissionState.EXECUTING_MOVE_PIPELINE
    )
    # Direct transition from WAITING_FOR_NEXT_MOVE to CHECKING_REACHABILITY is allowed for record/external moves
    assert is_mission_transition_allowed(
        MacroMissionState.WAITING_FOR_NEXT_MOVE, MacroMissionState.CHECKING_REACHABILITY
    )


def test_level1_macro_mission_tf_ready_must_go_through_board_sync():
    # Direct transition to NEXT_MOVE or EVALUATING_BEST_MOVE is disallowed (no backward-compat bypass)
    assert not is_mission_transition_allowed(
        MacroMissionState.WAITING_FOR_TF_READY, MacroMissionState.WAITING_FOR_NEXT_MOVE
    )
    assert not is_mission_transition_allowed(
        MacroMissionState.WAITING_FOR_TF_READY, MacroMissionState.EVALUATING_BEST_MOVE
    )
    # Must transition through AWAITING_BOARD_SYNC first
    assert is_mission_transition_allowed(
        MacroMissionState.WAITING_FOR_TF_READY, MacroMissionState.AWAITING_BOARD_SYNC
    )
    assert is_mission_transition_allowed(
        MacroMissionState.AWAITING_BOARD_SYNC, MacroMissionState.EVALUATING_BEST_MOVE
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
    # Dual-base capture sequence with final return to observation pose
    sequence = [
        MotionExecutionState.IDLE,
        MotionExecutionState.NAV_TO_CLEAR,
        MotionExecutionState.CLEARING_PIECE,
        MotionExecutionState.NAV_TO_PICK,
        MotionExecutionState.PICKING_PIECE,
        MotionExecutionState.NAV_TO_PLACE,
        MotionExecutionState.PLACING_PIECE,
        MotionExecutionState.NAV_TO_OBS,
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
        MotionExecutionState.NAV_TO_OBS,
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


def test_level1_macro_mission_error_fallback_to_relocalize():
    # ERROR_FALLBACK transitions to RELOCALIZE (non-backward-compatible modern recovery)
    assert is_mission_transition_allowed(
        MacroMissionState.ERROR_FALLBACK, MacroMissionState.RELOCALIZE
    )
    # Direct transition from ERROR_FALLBACK to WAITING_FOR_TF_READY is disallowed
    assert not is_mission_transition_allowed(
        MacroMissionState.ERROR_FALLBACK, MacroMissionState.WAITING_FOR_TF_READY
    )
    # RELOCALIZE can transition to AWAITING_BOARD_SYNC upon acquiring tags
    assert is_mission_transition_allowed(
        MacroMissionState.RELOCALIZE, MacroMissionState.AWAITING_BOARD_SYNC
    )
    # RELOCALIZE cannot jump straight into execution
    assert not is_mission_transition_allowed(
        MacroMissionState.RELOCALIZE, MacroMissionState.EXECUTING_MOVE_PIPELINE
    )
