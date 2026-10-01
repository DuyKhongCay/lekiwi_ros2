# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Offline TF Replayer: reconstructs coordinate frames from /tf and /tf_static in rosbags."""

from __future__ import annotations

import logging
from typing import Optional
import numpy as np

try:
    import tf2_ros
    from rclpy.time import Time
    _TF2_AVAILABLE = True
except ImportError:
    _TF2_AVAILABLE = False

logger = logging.getLogger(__name__)


class TfReplayer:
    """Maintains an offline tf2 BufferCore fed by messages from rosbag."""

    def __init__(self, cache_time_sec: float = 600.0) -> None:
        self.enabled = _TF2_AVAILABLE
        if self.enabled:
            # BufferCore(Duration)
            from rclpy.duration import Duration
            self.buffer = tf2_ros.BufferCore(Duration(seconds=cache_time_sec))
        else:
            self.buffer = None
            logger.warning("tf2_ros not available; offline TF transformation disabled")

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
        """Transform a 3D point from source_frame to target_frame at time_ns.

        If target_frame == source_frame or TF lookup fails, returns appropriate fallback.
        """
        if source_frame == target_frame:
            return point_xyz

        if not self.enabled or self.buffer is None:
            return point_xyz

        try:
            lookup_time = Time(nanoseconds=time_ns)
            tf = self.buffer.lookup_transform_core(target_frame, source_frame, lookup_time)

            tx = tf.transform.translation.x
            ty = tf.transform.translation.y
            tz = tf.transform.translation.z

            qx = tf.transform.rotation.x
            qy = tf.transform.rotation.y
            qz = tf.transform.rotation.z
            qw = tf.transform.rotation.w

            # Compute rotation matrix from quaternion
            r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
            r01 = 2.0 * (qx * qy - qz * qw)
            r02 = 2.0 * (qx * qz + qy * qw)

            r10 = 2.0 * (qx * qy + qz * qw)
            r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
            r12 = 2.0 * (qy * qz - qx * qw)

            r20 = 2.0 * (qx * qz - qy * qw)
            r21 = 2.0 * (qy * qz + qx * qw)
            r22 = 1.0 - 2.0 * (qx * qx + qy * qy)

            R = np.array([[r00, r01, r02],
                          [r10, r11, r12],
                          [r20, r21, r22]], dtype=np.float32)
            t = np.array([tx, ty, tz], dtype=np.float32)

            transformed = R @ point_xyz + t
            return transformed.astype(np.float32)

        except Exception as e:
            logger.debug(
                "TF lookup failed from %s to %s at %d ns: %s",
                source_frame,
                target_frame,
                time_ns,
                e,
            )
            return None
