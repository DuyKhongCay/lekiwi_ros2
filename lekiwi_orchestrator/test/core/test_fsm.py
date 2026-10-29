# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for LeKiwi 2-Level Hierarchical State Pattern, Micro Motion FSM, and PerceptionContext."""

import itertools
import math
from unittest.mock import Mock

import pytest
from builtin_interfaces.msg import Time as MsgTime
from geometry_msgs.msg import Quaternion
from rclpy.time import Time
from std_msgs.msg import Header

from lekiwi_interfaces.msg import PerceptionContext
from lekiwi_orchestrator.core.fsm import (
    IMPORT_FEN_ALLOWED_STATES,
    MacroMissionState,
    MotionExecutionState,
    is_mission_transition_allowed,
    is_motion_transition_allowed,
    is_perception_transition_allowed,
)
from lekiwi_orchestrator.core.utils import (
    create_pose2d_stamped,
    get_header_stamp_sec,
    get_node_now_sec,
    normalize_angle,
    yaw_to_quaternion,
)


def test_level1_macro_mission_transitions():
    """Verify canonical sequences, FEN import allowed states, board sync gating, and illegal jumps."""
    canonical_seq = [
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
        for curr, next_st in itertools.pairwise(canonical_seq)
    )

    # All allowed states can transition to AWAITING_BOARD_SYNC
    for state in IMPORT_FEN_ALLOWED_STATES:
        assert is_mission_transition_allowed(
            state, MacroMissionState.AWAITING_BOARD_SYNC
        ), f"State {state.name} should allow transition to AWAITING_BOARD_SYNC"

    # Moving or initializing states cannot transition directly to AWAITING_BOARD_SYNC
    for state in [
        MacroMissionState.BOOT_INITIALIZING,
        MacroMissionState.CHECKING_REACHABILITY,
        MacroMissionState.EXECUTING_MOVE_PIPELINE,
        MacroMissionState.POST_MOVE_VERIFYING,
    ]:
        assert not is_mission_transition_allowed(
            state, MacroMissionState.AWAITING_BOARD_SYNC
        )

    # WAITING_FOR_TF_READY must transition through AWAITING_BOARD_SYNC, not directly to NEXT_MOVE
    assert not is_mission_transition_allowed(
        MacroMissionState.WAITING_FOR_TF_READY, MacroMissionState.WAITING_FOR_NEXT_MOVE
    )
    assert not is_mission_transition_allowed(
        MacroMissionState.WAITING_FOR_TF_READY, MacroMissionState.EVALUATING_BEST_MOVE
    )
    assert is_mission_transition_allowed(
        MacroMissionState.WAITING_FOR_TF_READY, MacroMissionState.AWAITING_BOARD_SYNC
    )

    # Illegal jumps
    assert not is_mission_transition_allowed(
        MacroMissionState.BOOT_INITIALIZING, MacroMissionState.EXECUTING_MOVE_PIPELINE
    )
    assert not is_mission_transition_allowed(
        MacroMissionState.WAITING_FOR_TF_READY, MacroMissionState.EXECUTING_MOVE_PIPELINE
    )
    assert not is_mission_transition_allowed(
        MacroMissionState.AWAITING_BOARD_SYNC, MacroMissionState.EXECUTING_MOVE_PIPELINE
    )

    # Recovery from ERROR_FALLBACK to RELOCALIZE
    assert is_mission_transition_allowed(
        MacroMissionState.ERROR_FALLBACK, MacroMissionState.RELOCALIZE
    )
    assert not is_mission_transition_allowed(
        MacroMissionState.ERROR_FALLBACK, MacroMissionState.WAITING_FOR_TF_READY
    )
    assert is_mission_transition_allowed(
        MacroMissionState.RELOCALIZE, MacroMissionState.AWAITING_BOARD_SYNC
    )
    assert not is_mission_transition_allowed(
        MacroMissionState.RELOCALIZE, MacroMissionState.EXECUTING_MOVE_PIPELINE
    )


def test_level2_motion_sub_fsm_transitions():
    """Verify dual-base, single-base, zero-nav workflows and disallowed state jumps."""
    dual_base_seq = [
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
        for curr, next_st in itertools.pairwise(dual_base_seq)
    )

    zero_nav_seq = [
        MotionExecutionState.IDLE,
        MotionExecutionState.PICKING_PIECE,
        MotionExecutionState.PLACING_PIECE,
        MotionExecutionState.NAV_TO_OBS,
        MotionExecutionState.IDLE,
    ]
    assert all(
        is_motion_transition_allowed(curr, next_st)
        for curr, next_st in itertools.pairwise(zero_nav_seq)
    )

    # Illegal transitions
    assert not is_motion_transition_allowed(
        MotionExecutionState.IDLE, MotionExecutionState.PLACING_PIECE
    )
    assert not is_motion_transition_allowed(
        MotionExecutionState.NAV_TO_CLEAR, MotionExecutionState.PLACING_PIECE
    )


def test_perception_context_transitions():
    """Verify PerceptionContext lifecycle sequence and invalid mode jumps."""
    canonical_seq = [
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
        for current, requested in itertools.pairwise(canonical_seq)
    )
    assert not is_perception_transition_allowed(
        PerceptionContext.IDLE_STANDBY, PerceptionContext.MANIPULATION_ACTOR
    )
    assert not is_perception_transition_allowed(
        PerceptionContext.POST_MOVE_VERIFY, PerceptionContext.CALIBRATION_STREAM
    )


def test_core_utils_math_and_time():
    """Verify utility functions for angle wrapping, quaternion yaw conversions, and timestamps."""
    assert math.isclose(normalize_angle(3 * math.pi), math.pi, abs_tol=1e-5)
    assert math.isclose(normalize_angle(-3 * math.pi), -math.pi, abs_tol=1e-5)

    q = yaw_to_quaternion(math.pi / 2.0)
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    assert math.isclose(yaw, math.pi / 2.0, abs_tol=1e-5)

    pose = create_pose2d_stamped(1.0, 2.0, math.pi / 2.0, frame_id="odom")
    assert pose.header.frame_id == "odom"
    assert pose.pose.position.x == 1.0
    assert pose.pose.position.y == 2.0

    # Timestamp extraction
    header = Header()
    header.stamp = MsgTime(sec=100, nanosec=500_000_000)
    msg = Mock(header=header)
    assert math.isclose(get_header_stamp_sec(msg), 100.5, abs_tol=1e-6)

    # Node clock fallback
    mock_node = Mock()
    mock_node.get_clock.return_value.now.return_value = Time(seconds=42, nanoseconds=123_456_789)
    assert math.isclose(get_node_now_sec(mock_node), 42.123456789, abs_tol=1e-6)
