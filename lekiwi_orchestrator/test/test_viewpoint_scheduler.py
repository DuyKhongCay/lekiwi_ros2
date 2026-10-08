# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for Geodesic Viewpoint Scheduling and Azimuth Metric."""

from __future__ import annotations

import math
import pytest
from lekiwi_orchestrator.board_geometry import (
    normalize_angle,
    rank_by_azimuth,
    yaw_to_quaternion,
)


def test_geometry_angle_and_quaternion_transforms():
    """Verify angle normalization and yaw-to-quaternion planar conversions."""
    # Test angle normalization across wrap boundaries
    assert math.isclose(normalize_angle(0.0), 0.0)
    assert math.isclose(normalize_angle(math.pi), math.pi, abs_tol=1e-6)
    assert math.isclose(normalize_angle(-math.pi), -math.pi, abs_tol=1e-6)
    assert math.isclose(normalize_angle(3 * math.pi), math.pi, abs_tol=1e-6)
    assert math.isclose(normalize_angle(-3 * math.pi), -math.pi, abs_tol=1e-6)
    assert math.isclose(normalize_angle(math.pi / 2), math.pi / 2)

    # Test yaw to quaternion
    q0 = yaw_to_quaternion(0.0)
    assert (q0.x, q0.y, q0.z, q0.w) == (0.0, 0.0, 0.0, 1.0)
    q_pi2 = yaw_to_quaternion(math.pi / 2)
    assert math.isclose(q_pi2.z, math.sin(math.pi / 4), abs_tol=1e-6)
    assert math.isclose(q_pi2.w, math.cos(math.pi / 4), abs_tol=1e-6)


def test_rank_by_azimuth_sorting_and_edges():
    """Verify viewpoint ranking by angular distance, wraparound handling, and edge cases."""
    candidates = [math.pi / 2, 0.1, -0.3, math.pi]
    ranked = rank_by_azimuth(0.56, 0.0, candidate_offsets=candidates, standoff_radius=0.56)
    assert len(ranked) == 4
    assert math.isclose(ranked[0].angle_offset, 0.1)
    assert math.isclose(ranked[1].angle_offset, -0.3)
    assert math.isclose(ranked[2].angle_offset, math.pi / 2)
    assert math.isclose(ranked[3].angle_offset, math.pi)
    for i in range(len(ranked) - 1):
        assert ranked[i].arc_cost <= ranked[i + 1].arc_cost

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
