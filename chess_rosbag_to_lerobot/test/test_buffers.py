# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

import pytest
import numpy as np
from chess_rosbag_to_lerobot.buffers import LastBuffer


def test_buffer_push_and_asof_exact():
    buf = LastBuffer(max_age_ns=100_000_000)  # 100ms
    val = np.array([1.0, 2.0])
    buf.push(1_000_000_000, val)

    # Sample at exactly same timestamp
    res = buf.asof(1_000_000_000)
    assert res is not None
    np.testing.assert_array_equal(res, val)


def test_buffer_anti_future_leakage():
    buf = LastBuffer(max_age_ns=100_000_000)
    val = np.array([1.0])
    # Sample is from t=1.05s
    buf.push(1_050_000_000, val)

    # Reference tick is at t=1.00s (sample is in future relative to ref tick)
    res = buf.asof(1_000_000_000)
    assert res is None  # Must NOT return future sample!

    stats = buf.summary()
    assert stats["miss_future"] == 1
    assert stats["matched"] == 0


def test_buffer_stale_dropped():
    buf = LastBuffer(max_age_ns=50_000_000)  # 50ms window
    val = np.array([1.0])
    buf.push(1_000_000_000, val)

    # Query at 1.10s (dt = 100ms > 50ms)
    res = buf.asof(1_100_000_000)
    assert res is None

    stats = buf.summary()
    assert stats["miss_stale"] == 1
    assert stats["matched"] == 0


def test_buffer_reservoir_p95():
    buf = LastBuffer(max_age_ns=200_000_000, collect_p95=True, reservoir_cap=100)
    val = np.array([1.0])

    # Push and query multiple times with 10ms dt
    for i in range(50):
        t = (i + 1) * 100_000_000
        buf.push(t - 10_000_000, val)
        assert buf.asof(t) is not None

    stats = buf.summary()
    assert stats["matched"] == 50
    assert stats["match_rate"] == 1.0
    assert stats["p95_dt_s"] is not None
    assert pytest.approx(stats["p95_dt_s"], abs=1e-3) == 0.010

def test_tf_replayer_seed_and_missing_chain_raises():
    from chess_rosbag_to_lerobot.tf_replayer import TfReplayer
    replayer = TfReplayer()
    if not replayer.enabled:
        return

    # Seed nominal map -> chessboard_frame
    replayer.seed_static_transform("map", "chessboard_frame", [0.0, 0.0, 0.006, 0.0, 0.0, 0.0])

    # Trying to transform to base_footprint when map -> odom or odom -> base_footprint is missing
    # MUST raise RuntimeError, NEVER silently fall back or assume map == odom!
    with pytest.raises(RuntimeError) as excinfo:
        replayer.transform_point(
            np.array([0.1, 0.2, 0.035]),
            source_frame="chessboard_frame",
            target_frame="base_footprint",
            time_ns=1_000_000_000,
        )
    assert "CRITICAL TF TRANSFORMATION FAILURE" in str(excinfo.value)
