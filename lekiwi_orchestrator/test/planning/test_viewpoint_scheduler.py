# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for Geodesic Viewpoint Scheduling and Azimuth Metric."""

from __future__ import annotations

import math
import pytest
import rclpy
from lekiwi_orchestrator.planning.board_geometry import (
    rank_by_azimuth,
)



@pytest.fixture
def ros_context():
    """Ensure rclpy is initialized and cleanly shut down for ROS tests."""
    rclpy.init()
    yield
    rclpy.shutdown()


def test_rank_by_azimuth_geodesic_sorting():
    """Verify viewpoint ranking by global shortest tour, wraparound handling, and chained sequences."""
    candidates = [math.pi / 2, 0.1, -0.3, math.pi]
    ranked = rank_by_azimuth(0.56, 0.0, candidate_offsets=candidates, standoff_radius=0.56)
    assert len(ranked) == 4
    # Global shortest tour starting at 0.0:
    # 0.0 -> -0.3 -> 0.1 -> pi/2 -> pi
    assert math.isclose(ranked[0].angle_offset, -0.3)
    assert math.isclose(ranked[1].angle_offset, 0.1)
    assert math.isclose(ranked[2].angle_offset, math.pi / 2)
    assert math.isclose(ranked[3].angle_offset, math.pi)
    for i in range(len(ranked) - 1):
        assert ranked[i].cumulative_cost <= ranked[i + 1].cumulative_cost

    # Wraparound across +pi / -pi boundary
    curr_x = 0.56 * math.cos(3.1)
    curr_y = 0.56 * math.sin(3.1)
    ranked_wrap = rank_by_azimuth(curr_x, curr_y, candidate_offsets=[0.0, -3.1], standoff_radius=0.56)
    assert len(ranked_wrap) == 2
    assert math.isclose(ranked_wrap[0].angle_offset, -3.1)

    # Empty candidate list and origin fallback
    assert rank_by_azimuth(0.56, 0.0, candidate_offsets=[], standoff_radius=0.56) == []
    ranked_orig = rank_by_azimuth(0.0, 0.0, candidate_offsets=[0.0, math.pi - 0.1], standoff_radius=0.56, fallback_yaw=math.pi)
    assert math.isclose(ranked_orig[0].angle_offset, math.pi - 0.1)

    # Chained continuous tour
    chained_candidates = [-0.3, 0.1, 0.4, 0.7]
    ranked_chained = rank_by_azimuth(0.56, 0.0, candidate_offsets=chained_candidates, standoff_radius=0.56)
    assert len(ranked_chained) == 4
    assert math.isclose(ranked_chained[0].angle_offset, -0.3)
    assert math.isclose(ranked_chained[1].angle_offset, 0.1)
    assert math.isclose(ranked_chained[2].angle_offset, 0.4)
    assert math.isclose(ranked_chained[3].angle_offset, 0.7)


def test_obs_navigator_shortest_tour_locking(ros_context):
    """Verify that ObsNavigator locks the global shortest tour once and follows it sequentially."""
    from unittest.mock import Mock
    from rclpy.node import Node
    from lekiwi_orchestrator.core.mission_types import ObservationIntent
    from lekiwi_orchestrator.planning.obs_navigator import ObsNavigator

    node = Node("test_obs_nav_locking_node")
    try:
        mock_dispatcher = Mock()
        mock_dispatcher.send_navigation_goal.return_value = True

        candidates = [0.0, 0.4, -0.4, 0.8]
        navigator = ObsNavigator(
            node=node,
            dispatcher=mock_dispatcher,
            standoff_distance=1.0,
            angle_offsets_map={ObservationIntent.POST_MOVE_VERIFY: candidates},
        )

        # Initial retreat viewpoint (P0)
        p0 = navigator.compute_observation_pose_for_base(
            None, intent=ObservationIntent.POST_MOVE_VERIFY
        )
        assert p0 is not None
        assert navigator.get_viewpoint_index(ObservationIntent.POST_MOVE_VERIFY) == 1

        tour = navigator.get_active_tour(ObservationIntent.POST_MOVE_VERIFY)
        assert len(tour) == 4

        # Next repositioning steps follow tour[1], tour[2], tour[3]
        for expected_idx in [1, 2, 3]:
            assert (
                navigator.has_exhausted_viewpoints(
                    ObservationIntent.POST_MOVE_VERIFY
                )
                is False
            )
            assert (
                navigator.get_viewpoint_index(
                    ObservationIntent.POST_MOVE_VERIFY
                )
                == expected_idx
            )
            ok = navigator.reposition_to_next_viewpoint(
                intent=ObservationIntent.POST_MOVE_VERIFY
            )
            assert ok is True

        # Now all 4 viewpoints visited
        assert (
            navigator.has_exhausted_viewpoints(
                ObservationIntent.POST_MOVE_VERIFY
            )
            is True
        )
        assert (
            navigator.get_viewpoint_index(ObservationIntent.POST_MOVE_VERIFY)
            == 4
        )

        # Next call returns False (no % len wrap-around)
        assert (
            navigator.reposition_to_next_viewpoint(
                intent=ObservationIntent.POST_MOVE_VERIFY
            )
            is False
        )
    finally:
        node.destroy_node()


def test_post_move_verifier_exhaustion_finalizes_turn(ros_context):
    """Verify that PostMoveVerifier exits state by finalizing turn when viewpoints are exhausted."""
    import threading
    from types import SimpleNamespace
    from unittest.mock import Mock
    from rclpy.node import Node
    from lekiwi_orchestrator.core.fsm import MacroMissionState
    from lekiwi_orchestrator.core.mission_types import ObservationIntent
    from lekiwi_orchestrator.execution.turn_workflow import PostMoveVerifier
    from lekiwi_orchestrator.planning.obs_navigator import ObsNavigator

    node = Node("test_pmv_exhaustion_node")
    try:
        mock_dispatcher = Mock()
        mock_dispatcher.send_navigation_goal.return_value = True

        candidates = [0.0, 0.3]
        navigator = ObsNavigator(
            node=node,
            dispatcher=mock_dispatcher,
            standoff_distance=1.0,
            angle_offsets_map={ObservationIntent.POST_MOVE_VERIFY: candidates},
        )

        mock_config = SimpleNamespace(
            navigation=True,
            observation_scan_timeout_sec=5.0,
            action_timeout_sec=10.0,
        )
        mock_perception = Mock()
        state_lock = threading.RLock()
        finalized = []

        verifier = PostMoveVerifier(
            node=node,
            config=mock_config,
            observation_nav=navigator,
            perception=mock_perception,
            state_lock=state_lock,
            get_mission_state=lambda: MacroMissionState.POST_MOVE_VERIFYING,
            get_last_interaction_pose=lambda: None,
            on_finalize_turn=lambda: finalized.append(True),
        )

        # Initialize tour (P0 selected during retreat)
        p0 = navigator.compute_observation_pose_for_base(None)
        assert p0 is not None
        assert verifier.attempts == 1

        # Timeout 1: dispatches candidate 1 (the 2nd candidate)
        verifier.on_timeout()
        assert len(finalized) == 0
        assert verifier.attempts == 2

        # Timeout 2: candidates are now exhausted -> must call on_finalize_turn!
        verifier.on_timeout()
        assert len(finalized) == 1
    finally:
        node.destroy_node()


def test_rank_by_azimuth_large_candidate_set_performance():
    """Verify that rank_by_azimuth evaluates large candidate sets in O(N^2) time without hanging."""
    import time

    # Real 17 candidates from orchestrator.yaml relocalize
    candidates = [
        -2.831, -2.544, -2.248, -1.910, -1.568, -1.067, -0.667, -0.290,
        0.082, 0.484, 0.761, 1.147, 1.530, 1.934, 2.312, 2.699, 3.092
    ]
    curr_x = 0.58 * math.cos(-0.35)
    curr_y = 0.58 * math.sin(-0.35)

    t0 = time.perf_counter()
    ranked = rank_by_azimuth(
        curr_x=curr_x,
        curr_y=curr_y,
        candidate_offsets=candidates,
        standoff_radius=0.58,
    )
    elapsed_sec = time.perf_counter() - t0

    # Must complete in well under 50ms (previously took 400+ days with O(N!))
    assert elapsed_sec < 0.05, f"Execution too slow: {elapsed_sec*1000:.2f} ms"
    assert len(ranked) == 17

    # All unique candidates must be visited
    visited_angles = [vp.angle_offset for vp in ranked]
    assert len(set(visited_angles)) == 17

    # Total cumulative cost must be less than 2*pi * standoff_radius (~3.64m)
    assert ranked[-1].cumulative_cost < 2.0 * math.pi * 0.58

    # Cumulative cost must be strictly increasing
    for i in range(len(ranked) - 1):
        assert ranked[i].cumulative_cost < ranked[i + 1].cumulative_cost
