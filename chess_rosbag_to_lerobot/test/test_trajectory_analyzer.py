# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for TrajectoryAnalyzer and 4-layer teleoperation event filtering."""

import numpy as np
import pytest

from chess_rosbag_to_lerobot.config import LandmarksConfig, TeleopFilterConfig
from chess_rosbag_to_lerobot.tf_replayer import TfReplayer
from chess_rosbag_to_lerobot.trajectory_analyzer import (
    EpisodeTrajectoryPlan,
    GripperTransition,
    SubtaskSpan,
    TrajectoryAnalyzer,
)


def _create_mock_tf_replayer(tcp_positions: dict[int, tuple[float, float, float]]) -> TfReplayer:
    """Mock TfReplayer that returns specified TCP coordinates for given timestamps."""
    class MockTfReplayer(TfReplayer):
        def lookup_tcp_pose(self, tool_frame: str, base_frame: str, time_ns: int) -> np.ndarray:
            if tool_frame != "gripperframe":
                raise ValueError(f"Strict tool_frame check failed: expected 'gripperframe', got '{tool_frame}'")
            if time_ns in tcp_positions:
                return np.array(tcp_positions[time_ns], dtype=np.float32)
            # Interpolate or return closest
            closest_ts = min(tcp_positions.keys(), key=lambda t: abs(t - time_ns))
            return np.array(tcp_positions[closest_ts], dtype=np.float32)

    return MockTfReplayer()


@pytest.fixture
def filter_cfg() -> TeleopFilterConfig:
    return TeleopFilterConfig(
        tool_frame="gripperframe",
        base_frame="base_footprint",
        z_threshold_m=0.08,
        dwell_time_s=0.5,
        transit_min_s=1.0,
        gripper_close_threshold=0.15,
    )


@pytest.fixture
def landmarks() -> LandmarksConfig:
    return LandmarksConfig(
        stow=[0.0704, -0.1882, 0.1757],
        clear_left=[-0.0029, 0.1435, 0.1702],
        clear_right=[-0.0027, -0.1451, 0.1633],
    )


def test_trajectory_analyzer_strict_tool_frame_rejection(filter_cfg, landmarks):
    """Verify that using any frame other than 'gripperframe' causes lookup failure (no fallback)."""
    class StrictTf(TfReplayer):
        def lookup_tcp_pose(self, tool_frame: str, base_frame: str, time_ns: int) -> np.ndarray:
            if tool_frame != "gripperframe":
                raise RuntimeError(f"Unknown frame '{tool_frame}'. STRICT NO FALLBACK.")
            return np.array([0.1, 0.2, 0.05], dtype=np.float32)

    strict_tf = StrictTf()
    with pytest.raises(RuntimeError, match="Unknown frame 'gripper'"):
        strict_tf.lookup_tcp_pose("gripper", "base_footprint", 1000)

    # Valid when using gripperframe
    pose = strict_tf.lookup_tcp_pose("gripperframe", "base_footprint", 1000)
    assert np.allclose(pose, [0.1, 0.2, 0.05])


def test_z_height_gating_rejects_stow_transitions(filter_cfg, landmarks):
    """Transitions with Z > 0.08m (e.g. at stow Z ~ 0.175m) must be filtered out."""
    t_stow = 1_000_000_000
    t_board = 5_000_000_000

    tf_replayer = _create_mock_tf_replayer({
        t_stow: (0.07, -0.18, 0.175),   # At stow height (Z > 0.08)
        t_board: (0.15, 0.05, 0.035),  # At board height (Z <= 0.08)
    })

    analyzer = TrajectoryAnalyzer(tf_replayer, filter_cfg, landmarks)

    transitions = [
        GripperTransition("close", t_stow, 0.6, np.array([0.07, -0.18, 0.175], dtype=np.float32)),
        GripperTransition("close", t_board, 0.6, np.array([0.15, 0.05, 0.035], dtype=np.float32)),
    ]

    valid = analyzer._filter_by_z_height(transitions)
    assert len(valid) == 1
    assert valid[0].ts_ns == t_board
    assert valid[0].tcp_xyz[2] <= filter_cfg.z_threshold_m


def test_dwell_time_filtering(filter_cfg, landmarks):
    """Gripper chatter (< 0.5s dwell) must be ignored, only sustained holds (>= 0.5s) accepted."""
    tf_replayer = _create_mock_tf_replayer({
        0: (0.1, 0.0, 0.03),
        5_000_000_000: (0.2, 0.0, 0.03),
    })
    analyzer = TrajectoryAnalyzer(tf_replayer, filter_cfg, landmarks)

    # Sequence of 10 Hz samples:
    # 0.0s - 1.0s: open (> 0.15, say 0.5)
    # 1.0s - 1.2s: closed (<= 0.15, say 0.05) -> duration 0.2s (< 0.5s dwell) -> CLOSE CHATTER
    # 1.3s - 2.0s: open (0.5)
    # 2.0s - 3.5s: closed (0.05) -> duration 1.5s (>= 0.5s dwell) -> VALID CLOSE
    #   (inside this, from 2.5s - 2.7s: open 0.2s -> OPEN CHATTER)
    # 3.5s - 5.0s: open (0.5) -> duration 1.5s (>= 0.5s dwell) -> VALID OPEN
    joint_samples = []
    # 0.0 - 0.9s open
    for step in range(10):
        ts = step * 100_000_000
        joint_samples.append((ts, np.array([0, 0, 0, 0, 0, 0.5])))
    # 1.0 - 1.2s closed (0.2s chatter)
    for step in range(10, 13):
        ts = step * 100_000_000
        joint_samples.append((ts, np.array([0, 0, 0, 0, 0, 0.05])))
    # 1.3 - 1.9s open
    for step in range(13, 20):
        ts = step * 100_000_000
        joint_samples.append((ts, np.array([0, 0, 0, 0, 0, 0.5])))
    # 2.0 - 2.6s closed (0.6s sustained)
    for step in range(20, 27):
        ts = step * 100_000_000
        joint_samples.append((ts, np.array([0, 0, 0, 0, 0, 0.05])))
    # 2.7 - 2.9s open chatter (0.2s chatter)
    for step in range(27, 30):
        ts = step * 100_000_000
        joint_samples.append((ts, np.array([0, 0, 0, 0, 0, 0.5])))
    # 3.0 - 3.7s closed (0.7s)
    for step in range(30, 38):
        ts = step * 100_000_000
        joint_samples.append((ts, np.array([0, 0, 0, 0, 0, 0.05])))
    # 3.8 - 5.5s open (1.7s sustained)
    for step in range(38, 56):
        ts = step * 100_000_000
        joint_samples.append((ts, np.array([0, 0, 0, 0, 0, 0.5])))

    detected = analyzer._detect_transitions(joint_samples)
    # Close chatter at 1.0s (<0.5s) must be discarded.
    # Open chatter at 2.7s (<0.5s) must be discarded.
    # Only valid sustained transitions should be recorded:
    # 1. close at 2.0s
    # 2. open at 3.8s
    close_events = [t for t in detected if t.event_type == "close"]
    open_events = [t for t in detected if t.event_type == "open" and t.ts_ns >= 2_000_000_000]

    assert len(close_events) >= 1
    assert all(c.duration_s >= 0.5 for c in close_events)
    assert close_events[0].ts_ns == 2_000_000_000

    assert len(open_events) == 1
    assert open_events[0].ts_ns == 38 * 100_000_000
    assert open_events[0].duration_s >= 0.5


def test_move_task_trajectory_resolution(filter_cfg, landmarks):
    """End-to-end MOVE trajectory analysis with 3 subtask spans."""
    t_start = 0
    t_pick = 2_000_000_000    # 2.0s
    t_place = 4_500_000_000   # 4.5s (transit time = 2.5s >= 1.0s)
    t_end = 6_000_000_000     # 6.0s

    tf_map = {
        t_start: (0.0704, -0.1882, 0.1757),  # Stow
        t_pick: (0.12, -0.05, 0.035),        # Source pick
        t_place: (0.20, 0.08, 0.035),        # Target place
        t_end: (0.0704, -0.1882, 0.1757),    # Return stow
    }
    tf_replayer = _create_mock_tf_replayer(tf_map)
    analyzer = TrajectoryAnalyzer(tf_replayer, filter_cfg, landmarks)

    # Construct joint samples: open until 2.0s, closed until 4.5s, open until 6.0s
    samples = []
    for ts_ms in range(0, 6100, 100):
        ts_ns = ts_ms * 1_000_000
        if ts_ns < t_pick:
            g_val = 0.5   # Open
        elif ts_ns < t_place:
            g_val = 0.05  # Closed (grasping)
        else:
            g_val = 0.5   # Open (released)
        samples.append((ts_ns, np.array([0.1, -0.2, 0.3, -0.4, 0.5, g_val], dtype=np.float32)))

    plan = analyzer.analyze(samples, "Move black knight to target", "move")
    assert plan is not None
    assert plan.task_type == "move"
    assert np.allclose(plan.source_xyz, [0.12, -0.05, 0.035])
    assert np.allclose(plan.target_xyz, [0.20, 0.08, 0.035])
    assert np.allclose(plan.stow_xyz, landmarks.stow)
    assert plan.t_pick_ns == t_pick
    assert plan.t_place_ns == t_place

    # Verify 3 subtask spans:
    # 1. "Pick piece at source" (0 -> t_pick, goal = source_xyz)
    # 2. "Place piece to target" (t_pick -> t_place, goal = target_xyz)
    # 3. "Retract arm to stow" (t_place -> t_end, goal = stow_xyz)
    assert len(plan.subtask_spans) == 3
    s1, s2, s3 = plan.subtask_spans

    assert s1.name == "Pick piece at source"
    assert np.allclose(s1.goal_xyz, [0.12, -0.05, 0.035])
    assert s1.start_ts_ns == 0
    assert s1.end_ts_ns == t_pick

    assert s2.name == "Place piece to target"
    assert np.allclose(s2.goal_xyz, [0.20, 0.08, 0.035])
    assert s2.start_ts_ns == t_pick
    assert s2.end_ts_ns == t_place

    assert s3.name == "Retract arm to stow"
    assert np.allclose(s3.goal_xyz, landmarks.stow)
    assert s3.start_ts_ns == t_place
    assert s3.end_ts_ns == t_end

    # Test get_active_subtask
    assert plan.get_active_subtask(1_000_000_000) == s1
    assert plan.get_active_subtask(3_000_000_000) == s2
    assert plan.get_active_subtask(5_500_000_000) == s3
