# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

import io
import pytest
import numpy as np
from types import SimpleNamespace
from PIL import Image

from chess_rosbag_to_lerobot.config import FeatureSpec
from chess_rosbag_to_lerobot.decoders import (
    decode_compressed_image,
    decode_joint_state,
    decode_float64_multiarray,
    decode_string,
)


def test_decode_compressed_image():
    # Create a small 64x64 synthetic JPEG
    img = Image.new("RGB", (64, 64), color=(255, 128, 64))
    buf = io.BytesIO()
    img.save(buf, format="JPEG")
    jpeg_bytes = buf.getvalue()

    msg = SimpleNamespace(data=jpeg_bytes)
    spec = FeatureSpec(
        key="observation.images.wrist",
        msg_type="sensor_msgs/msg/CompressedImage",
        shape=[64, 64, 3],
    )

    arr = decode_compressed_image(msg, spec)
    assert isinstance(arr, np.ndarray)
    assert arr.shape == (64, 64, 3)
    assert arr.dtype == np.uint8


def test_decode_joint_state():
    joint_names = [
        "arm_shoulder_pan",
        "arm_shoulder_lift",
        "arm_elbow_flex",
        "arm_wrist_flex",
        "arm_wrist_roll",
        "arm_gripper",
    ]
    # Message has extra wheel joints and arbitrary order
    msg = SimpleNamespace(
        name=[
            "base_left_wheel",
            "arm_gripper",
            "arm_shoulder_pan",
            "arm_wrist_roll",
            "arm_shoulder_lift",
            "arm_elbow_flex",
            "arm_wrist_flex",
        ],
        position=[10.0, 0.5, 0.1, 0.4, 0.2, -0.3, 0.0],
    )

    spec = FeatureSpec(
        key="observation.state",
        msg_type="sensor_msgs/msg/JointState",
        names=joint_names,
    )

    arr = decode_joint_state(msg, spec)
    assert isinstance(arr, np.ndarray)
    assert arr.shape == (6,)
    assert arr.dtype == np.float32

    expected = np.array([0.1, 0.2, -0.3, 0.0, 0.4, 0.5], dtype=np.float32)
    np.testing.assert_allclose(arr, expected, atol=1e-5)


def test_decode_float64_multiarray():
    msg = SimpleNamespace(data=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    spec = FeatureSpec(
        key="action",
        msg_type="std_msgs/msg/Float64MultiArray",
        shape=[6],
    )
    arr = decode_float64_multiarray(msg, spec)
    assert arr.shape == (6,)
    assert arr.dtype == np.float32
    assert pytest.approx(arr[0], abs=1e-5) == 0.1


def test_decode_string():
    msg = SimpleNamespace(data="rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq e3 0 1")
    spec = FeatureSpec(key="observation.chess.fen", msg_type="std_msgs/msg/String")
    s = decode_string(msg, spec)
    assert s.startswith("rnbqkbnr")
