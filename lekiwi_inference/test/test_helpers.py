# Copyright 2026 LeKiwi Robotics Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit tests for LeRobot 0.6.1 helper dataclasses and aggregation functions."""

import numpy as np
import pytest

from lekiwi_inference.helpers import (
    AGGREGATE_FUNCTIONS,
    FPSTracker,
    LatencyTracker,
    RemotePolicyConfig,
    TimedAction,
    TimedData,
    TimedObservation,
)


def test_timed_data_accessors():
    """Validates timestamp and timestep getter methods on TimedData hierarchy."""
    td = TimedData(timestamp=100.5, timestep=10)
    assert td.get_timestamp() == 100.5
    assert td.get_timestep() == 10

    act_arr = np.array([0.1, 0.2], dtype=np.float32)
    ta = TimedAction(timestamp=101.0, timestep=11, action=act_arr)
    assert ta.get_timestamp() == 101.0
    assert ta.get_timestep() == 11
    assert np.allclose(ta.get_action(), act_arr)

    obs_dict = {"observation.state": act_arr}
    t_obs = TimedObservation(timestamp=102.0, timestep=12, observation=obs_dict, must_go=True)
    assert t_obs.get_timestamp() == 102.0
    assert t_obs.get_timestep() == 12
    assert t_obs.get_observation() == obs_dict
    assert t_obs.must_go is True


def test_fps_tracker():
    """Validates average FPS calculation and reset behavior."""
    tracker = FPSTracker(target_fps=25.0)
    m1 = tracker.calculate_fps_metrics(current_timestamp=10.0)
    assert m1["target_fps"] == 25.0
    assert m1["avg_fps"] == 0.0

    # 10 frames over 0.4 seconds -> 25 Hz
    for i in range(1, 11):
        metrics = tracker.calculate_fps_metrics(current_timestamp=10.0 + i * 0.04)

    assert metrics["avg_fps"] == pytest.approx(25.0, abs=0.1)

    tracker.reset()
    assert tracker.first_timestamp is None
    assert tracker.total_obs_count == 0


def test_aggregation_functions():
    """Validates temporal smoothing algorithms between overlapping predictions."""
    old_act = np.array([10.0, 20.0], dtype=np.float32)
    new_act = np.array([0.0, 100.0], dtype=np.float32)

    # weighted_average: 0.3 * old + 0.7 * new
    weighted = AGGREGATE_FUNCTIONS["weighted_average"](old_act, new_act)
    assert np.allclose(weighted, [3.0, 76.0])

    # latest_only: new
    latest = AGGREGATE_FUNCTIONS["latest_only"](old_act, new_act)
    assert np.allclose(latest, new_act)

    # average: 0.5 * old + 0.5 * new
    avg = AGGREGATE_FUNCTIONS["average"](old_act, new_act)
    assert np.allclose(avg, [5.0, 60.0])

    # conservative: 0.7 * old + 0.3 * new
    cons = AGGREGATE_FUNCTIONS["conservative"](old_act, new_act)
    assert np.allclose(cons, [7.0, 44.0])


def test_observations_similar_identical():
    """Identical state vectors must evaluate as similar for any positive atol."""
    from lekiwi_inference.helpers import observations_similar

    s1 = np.array([0.1, -0.2, 0.3, 0.4, -0.5, 0.6], dtype=np.float32)
    s2 = s1.copy()
    assert observations_similar(s1, s2, atol=0.05) is True


def test_observations_similar_within_tolerance():
    """Displacement strictly less than Euclidean norm atol must evaluate as True."""
    from lekiwi_inference.helpers import observations_similar

    s1 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
    # Norm is sqrt(0.02^2 + 0.02^2) = 0.02828 < 0.05
    s2 = np.array([0.02, 0.02, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
    assert observations_similar(s1, s2, atol=0.05) is True


def test_observations_similar_exceed_tolerance():
    """Displacement equal or greater than atol must evaluate as False."""
    from lekiwi_inference.helpers import observations_similar

    s1 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
    # Norm is sqrt(0.04^2 + 0.04^2) = 0.05657 > 0.05
    s2 = np.array([0.04, 0.04, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
    assert observations_similar(s1, s2, atol=0.05) is False


def test_latency_tracker():
    """Validates LatencyTracker rolling window, max, mean, delay computation and reset."""
    tracker = LatencyTracker(window_size=4)
    # Default without history
    assert tracker.compute_delay_steps(dt=0.04, default_steps=14) == 14

    # Add samples: 0.1s, 0.2s, 0.15s, 0.28s -> max=0.28s -> ceil(0.28 / 0.04) = 7 steps
    tracker.add(0.10)
    tracker.add(0.20)
    tracker.add(0.15)
    tracker.add(0.28)
    assert tracker.max() == pytest.approx(0.28)
    assert tracker.mean() == pytest.approx((0.10 + 0.20 + 0.15 + 0.28) / 4)
    assert tracker.compute_delay_steps(dt=0.04) == 7

    # Add 5th sample, 1st sample (0.10) is evicted
    tracker.add(0.08)
    assert len(tracker.history) == 4
    assert 0.10 not in tracker.history

    tracker.reset()
    assert len(tracker.history) == 0
    assert tracker.compute_delay_steps(dt=0.04, default_steps=14) == 14


def test_timed_observation_rtc():
    """Validates TimedObservation initialized with RTC delay and leftover prefix."""
    prev_actions = np.zeros((10, 6), dtype=np.float32)
    obs = TimedObservation(
        timestamp=200.0,
        timestep=5,
        observation={"state": np.zeros(6)},
        inference_delay=14,
        prev_chunk_left_over=prev_actions,
    )
    assert obs.inference_delay == 14
    assert obs.prev_chunk_left_over is not None
    assert obs.prev_chunk_left_over.shape == (10, 6)

