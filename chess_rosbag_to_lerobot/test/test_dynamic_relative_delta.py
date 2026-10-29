# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for Dynamic Relative Delta 12D observation state assembly."""

import numpy as np
import pytest

from chess_rosbag_to_lerobot.buffers import LastBuffer
from chess_rosbag_to_lerobot.config import Config, FeatureSpec, LandmarksConfig, TeleopFilterConfig
from chess_rosbag_to_lerobot.converter import _build_reference_frame, build_lerobot_features
from chess_rosbag_to_lerobot.tf_replayer import TfReplayer
from chess_rosbag_to_lerobot.trajectory_analyzer import EpisodeTrajectoryPlan, SubtaskSpan


class DummyTfReplayer(TfReplayer):
    def __init__(self, trajectory_map: dict[int, np.ndarray]):
        self.trajectory_map = trajectory_map

    def lookup_tcp_pose(self, tool_frame: str, base_frame: str, time_ns: int) -> np.ndarray:
        if tool_frame != "gripperframe":
            raise ValueError(f"Strict tool_frame violated: {tool_frame}")
        if time_ns in self.trajectory_map:
            return self.trajectory_map[time_ns]
        closest_ts = min(self.trajectory_map.keys(), key=lambda t: abs(t - time_ns))
        return self.trajectory_map[closest_ts]


@pytest.fixture
def mock_config() -> Config:
    return Config(
        robot_type="lekiwi",
        fps=25,
        reference_topic="/cam",
        default_max_age_s=0.1,
        task="default",
        features=[
            FeatureSpec(key="observation.state", topic="/joint_states", msg_type="sensor_msgs/msg/JointState", names=["j1", "j2", "j3", "j4", "j5", "gripper"]),
            FeatureSpec(key="observation.images.wrist", topic="/cam", msg_type="sensor_msgs/msg/CompressedImage", shape=[480, 640, 3]),
        ],
        teleop_filter=TeleopFilterConfig(
            tool_frame="gripperframe",
            base_frame="base_footprint",
            z_threshold_m=0.08,
            dwell_time_s=0.5,
            transit_min_s=1.0,
            gripper_close_threshold=0.15,
        ),
    )


def test_build_reference_frame_12d_convergence(mock_config):
    """Verify that observation.state has exactly 12D and delta converges to 0 at goal."""
    goal_pick = np.array([0.15, -0.05, 0.035], dtype=np.float32)
    goal_place = np.array([0.25, 0.10, 0.035], dtype=np.float32)

    plan = EpisodeTrajectoryPlan(
        task_type="move",
        canonical_task="Move white pawn",
        source_xyz=goal_pick,
        target_xyz=goal_place,
        stow_xyz=np.array([0.07, -0.18, 0.175], dtype=np.float32),
        t_pick_ns=2_000_000_000,
        t_place_ns=4_000_000_000,
        subtask_spans=[
            SubtaskSpan("Pick piece at source", goal_pick, 0, 2_000_000_000),
            SubtaskSpan("Place piece to target", goal_place, 2_000_000_000, 4_000_000_000),
        ],
    )

    # 1. At t=0, TCP is far from pick goal: [0.07, -0.18, 0.175]
    tcp_t0 = np.array([0.07, -0.18, 0.175], dtype=np.float32)
    # 2. At t=2s (t_pick), TCP reaches pick goal: [0.15, -0.05, 0.035]
    tcp_t_pick = goal_pick.copy()

    tf_replayer = DummyTfReplayer({
        0: tcp_t0,
        2_000_000_000: tcp_t_pick,
    })

    # Prepare buffer for joint states
    joint_buffer = LastBuffer(max_age_ns=int(0.1 * 1e9))
    q_dummy = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.05], dtype=np.float32)
    joint_buffer.push(0, q_dummy)

    buffers = {"/joint_states": joint_buffer}
    ref_spec = mock_config.features[1]  # /cam

    # Test at t=0
    dummy_cam_msg = None
    # Mock decode helper for image
    import chess_rosbag_to_lerobot.converter as conv_mod
    original_decode = conv_mod.decode
    conv_mod.decode = lambda msg, spec: np.zeros((480, 640, 3), dtype=np.uint8) if spec.key.startswith("observation.images") else q_dummy

    try:
        frame_t0 = _build_reference_frame(
            dummy_cam_msg,
            0,
            ref_spec,
            plan,
            buffers,
            tf_replayer,
            mock_config,
        )
        assert frame_t0 is not None
        assert frame_t0["task"] == "Move white pawn"

        state_t0 = frame_t0["observation.state"]
        assert isinstance(state_t0, np.ndarray)
        assert state_t0.shape == (12,)
        assert state_t0.dtype == np.float32

        # 6 joints
        assert np.allclose(state_t0[:6], q_dummy)
        # 3D ee_xyz
        assert np.allclose(state_t0[6:9], tcp_t0)
        # 3D delta = goal_pick - tcp_t0
        expected_delta_t0 = goal_pick - tcp_t0
        assert np.allclose(state_t0[9:12], expected_delta_t0)
        assert np.linalg.norm(state_t0[9:12]) > 0.1  # Significant delta away from goal

        # Advance chronological stream to t=2s (robot reaches pick goal)
        joint_buffer.push(2_000_000_000, q_dummy)
        frame_t_pick = _build_reference_frame(
            dummy_cam_msg,
            2_000_000_000,
            ref_spec,
            plan,
            buffers,
            tf_replayer,
            mock_config,
        )
        assert frame_t_pick is not None
        state_t_pick = frame_t_pick["observation.state"]
        assert state_t_pick.shape == (12,)
        # At pick goal, TCP == goal_pick -> delta == 0
        assert np.allclose(state_t_pick[6:9], goal_pick)
        assert np.allclose(state_t_pick[9:12], [0.0, 0.0, 0.0], atol=1e-5)

    finally:
        conv_mod.decode = original_decode
