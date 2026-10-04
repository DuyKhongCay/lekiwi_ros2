# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Decoder registry for converting ROS messages into LeRobot numpy arrays."""

from __future__ import annotations

import io
from typing import Any, Callable, Dict
import numpy as np

try:
    import imageio.v3 as iio
except ImportError:
    import imageio as iio

from chess_rosbag_to_lerobot.config import FeatureSpec

DecoderFn = Callable[[Any, FeatureSpec], Any]

DECODERS: Dict[str, DecoderFn] = {}
DTYPES: Dict[str, str] = {}


def register_decoder(msg_type: str, *, lerobot_dtype: str) -> Callable[[DecoderFn], DecoderFn]:
    """Decorator to register a decoder function for a given ROS msg_type."""
    def decorator(fn: DecoderFn) -> DecoderFn:
        DECODERS[msg_type] = fn
        DTYPES[msg_type] = lerobot_dtype
        return fn
    return decorator


def get_decoder(msg_type: str) -> DecoderFn:
    if msg_type not in DECODERS:
        raise KeyError(
            f"No decoder registered for msg_type='{msg_type}'. "
            f"Registered: {sorted(DECODERS.keys())}"
        )
    return DECODERS[msg_type]


def get_lerobot_dtype(msg_type: str) -> str:
    return DTYPES.get(msg_type, "float32")


def decode(msg: Any, spec: FeatureSpec) -> Any:
    """Decode message according to spec.msg_type using the registry."""
    fn = get_decoder(spec.msg_type)
    return fn(msg, spec)


def _ensure_shape(arr: np.ndarray, spec: FeatureSpec) -> np.ndarray:
    """Validate shape matches expected shape from config if specified."""
    if spec.shape is None:
        return arr
    expected = tuple(int(x) for x in spec.shape)
    if arr.shape != expected:
        raise ValueError(
            f"Feature '{spec.key}': decoded shape {arr.shape} != expected {expected} "
            f"(topic={spec.topic}, msg_type={spec.msg_type})"
        )
    return arr


# -----------------------------------------------------------------------------
# Decoders
# -----------------------------------------------------------------------------

@register_decoder("sensor_msgs/msg/CompressedImage", lerobot_dtype="video")
def decode_compressed_image(msg: Any, spec: FeatureSpec) -> np.ndarray:
    """Decode JPEG/PNG compressed image into HWC uint8 RGB numpy array."""
    buf = bytes(msg.data)
    img = iio.imread(io.BytesIO(buf))

    if img.ndim == 2:  # Grayscale -> 3 channel
        img = np.stack([img, img, img], axis=-1)
    elif img.ndim == 3 and img.shape[2] == 4:  # RGBA -> RGB
        img = img[:, :, :3]

    img = np.ascontiguousarray(img, dtype=np.uint8)
    return _ensure_shape(img, spec)


@register_decoder("sensor_msgs/msg/Image", lerobot_dtype="video")
def decode_raw_image(msg: Any, spec: FeatureSpec) -> np.ndarray:
    """Decode raw uncompressed Image into HWC uint8 RGB numpy array."""
    enc = (msg.encoding or "").lower()
    h, w = int(msg.height), int(msg.width)

    if enc in ("rgb8", "bgr8"):
        ch = 3
    elif enc in ("rgba8", "bgra8"):
        ch = 4
    elif enc in ("mono8", "8uc1"):
        ch = 1
    else:
        raise ValueError(f"Unsupported image encoding '{msg.encoding}' for {spec.topic}")

    step = int(getattr(msg, "step", 0) or (w * ch))
    if step != w * ch:
        raise ValueError(f"Padded row step ({step} != {w * ch}) not supported for {spec.topic}")

    raw = np.frombuffer(memoryview(msg.data), dtype=np.uint8).reshape(h, w, ch)
    if enc == "bgr8":
        raw = raw[..., ::-1].copy()
    elif enc == "rgba8":
        raw = raw[..., :3].copy()
    elif enc == "bgra8":
        raw = raw[..., 2::-1].copy()
    elif enc in ("mono8", "8uc1"):
        raw = np.repeat(raw[:, :, None], 3, axis=2)

    return _ensure_shape(np.ascontiguousarray(raw, dtype=np.uint8), spec)


@register_decoder("sensor_msgs/msg/JointState", lerobot_dtype="float32")
def decode_joint_state(msg: Any, spec: FeatureSpec) -> np.ndarray:
    """Decode JointState message, reordering and filtering by spec.names."""
    pos = np.asarray(msg.position, dtype=np.float32).flatten()
    if not spec.names:
        return _ensure_shape(pos, spec)

    name_to_idx = {name: idx for idx, name in enumerate(list(msg.name))}
    out = np.zeros((len(spec.names),), dtype=np.float32)

    for j, joint in enumerate(spec.names):
        i = name_to_idx.get(joint)
        if i is not None and i < len(pos):
            out[j] = pos[i]
        else:
            out[j] = 0.0

    return _ensure_shape(out, spec)


@register_decoder("std_msgs/msg/Float64MultiArray", lerobot_dtype="float32")
def decode_float64_multiarray(msg: Any, spec: FeatureSpec) -> np.ndarray:
    """Decode Float64MultiArray into flat float32 array."""
    arr = np.asarray(msg.data, dtype=np.float32).flatten()
    return _ensure_shape(arr, spec)


@register_decoder("std_msgs/msg/String", lerobot_dtype="string")
def decode_string(msg: Any, spec: FeatureSpec) -> str:
    """Decode String message (e.g. FEN)."""
    return str(msg.data)
