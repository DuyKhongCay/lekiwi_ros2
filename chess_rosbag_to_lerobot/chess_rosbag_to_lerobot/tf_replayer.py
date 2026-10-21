# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Offline TF Replayer: reconstructs coordinate frames from /tf and /tf_static in rosbags."""

from __future__ import annotations

import logging
from typing import Any, Optional
import numpy as np

try:
    import tf2_ros
    from rclpy.time import Time
    _TF2_AVAILABLE = True
except ImportError:
    _TF2_AVAILABLE = False

logger = logging.getLogger(__name__)


def _quat_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Convert quaternion (qx, qy, qz, qw) to 3x3 rotation matrix."""
    norm = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm > 0.0:
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

    r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
    r01 = 2.0 * (qx * qy - qz * qw)
    r02 = 2.0 * (qx * qz + qy * qw)

    r10 = 2.0 * (qx * qy + qz * qw)
    r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
    r12 = 2.0 * (qy * qz - qx * qw)

    r20 = 2.0 * (qx * qz - qy * qw)
    r21 = 2.0 * (qy * qz + qx * qw)
    r22 = 1.0 - 2.0 * (qx * qx + qy * qy)

    return np.array([[r00, r01, r02],
                     [r10, r11, r12],
                     [r20, r21, r22]], dtype=np.float64)


class TfReplayer:
    """Maintains an offline tf2 BufferCore fed by messages from rosbag."""

    def __init__(self, cache_time_sec: float = 600.0) -> None:
        self.enabled = _TF2_AVAILABLE
        if self.enabled:
            from rclpy.duration import Duration
            self.buffer: Optional[Any] = tf2_ros.BufferCore(Duration(seconds=cache_time_sec))
        else:
            self.buffer = None
            logger.debug("tf2_ros not available; offline TF transformation disabled")

    def handle_tf_message(self, msg: Any, is_static: bool = False) -> None:
        """Feed a tf2_msgs/msg/TFMessage into the buffer."""
        if not self.enabled or self.buffer is None:
            return

        for transform in msg.transforms:
            if is_static:
                self.buffer.set_transform_static(transform, "rosbag_static")
            else:
                self.buffer.set_transform(transform, "rosbag_dynamic")

    def transform_point(
        self,
        point_xyz: np.ndarray,
        source_frame: str,
        target_frame: str,
        time_ns: int,
    ) -> Optional[np.ndarray]:
        """Transform a 3D point from source_frame to target_frame at time_ns."""
        src = (source_frame or "").strip().lstrip("/")
        tgt = (target_frame or "").strip().lstrip("/")

        if src == tgt:
            return point_xyz.astype(np.float32)

        if not self.enabled or self.buffer is None:
            return None

        try:
            lookup_time = Time(nanoseconds=time_ns)
            tf = self.buffer.lookup_transform_core(tgt, src, lookup_time)

            tx = float(tf.transform.translation.x)
            ty = float(tf.transform.translation.y)
            tz = float(tf.transform.translation.z)

            qx = float(tf.transform.rotation.x)
            qy = float(tf.transform.rotation.y)
            qz = float(tf.transform.rotation.z)
            qw = float(tf.transform.rotation.w)

            R = _quat_to_matrix(qx, qy, qz, qw)
            t = np.array([tx, ty, tz], dtype=np.float64)

            transformed = R @ point_xyz.astype(np.float64) + t
            return transformed.astype(np.float32)

        except Exception as e:
            logger.debug(
                "TF lookup failed from %s to %s at %d ns: %s",
                src,
                tgt,
                time_ns,
                e,
            )
            return None
