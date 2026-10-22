# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Offline TF Replayer: reconstructs coordinate frames from /tf and /tf_static in rosbags."""

from __future__ import annotations

import logging
import math
from typing import Any, List, Optional, Sequence
import numpy as np

try:
    import tf2_ros
    from rclpy.time import Time
    from geometry_msgs.msg import TransformStamped
    _TF2_AVAILABLE = True
except ImportError:
    _TF2_AVAILABLE = False
    TransformStamped = Any

logger = logging.getLogger(__name__)


def _rpy_to_quat(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Convert roll, pitch, yaw (radians) to quaternion (qx, qy, qz, qw)."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


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

    def seed_static_transform(
        self,
        parent_frame: str,
        child_frame: str,
        pose_6d: Sequence[float],
    ) -> None:
        """Seed a nominal static transform (e.g. map -> chessboard_frame) into BufferCore.

        Args:
            parent_frame: Reference frame ID (e.g. 'map').
            child_frame: Target child frame ID (e.g. 'chessboard_frame').
            pose_6d: 6D pose [x, y, z, roll, pitch, yaw] in meters and radians.
        """
        if not self.enabled or self.buffer is None or len(pose_6d) < 6:
            return

        parent = (parent_frame or "").strip().lstrip("/")
        child = (child_frame or "").strip().lstrip("/")
        if not parent or not child or parent == child:
            return

        tf_msg = TransformStamped()
        tf_msg.header.stamp.sec = 0
        tf_msg.header.stamp.nanosec = 0
        tf_msg.header.frame_id = parent
        tf_msg.child_frame_id = child

        tf_msg.transform.translation.x = float(pose_6d[0])
        tf_msg.transform.translation.y = float(pose_6d[1])
        tf_msg.transform.translation.z = float(pose_6d[2])

        qx, qy, qz, qw = _rpy_to_quat(float(pose_6d[3]), float(pose_6d[4]), float(pose_6d[5]))
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        self.buffer.set_transform_static(tf_msg, "nominal_config_seed")
        logger.info(
            "Seeded nominal static transform: '%s' -> '%s' (xyz=[%.3f, %.3f, %.3f])",
            parent,
            child,
            pose_6d[0],
            pose_6d[1],
            pose_6d[2],
        )

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
            raise RuntimeError(
                f"TF transformation required from '{src}' to '{tgt}', but tf2_ros BufferCore is disabled."
            )

        try:
            lookup_time = Time(nanoseconds=time_ns)
            tf = self.buffer.lookup_transform_core(tgt, src, lookup_time)
        except Exception:
            # Fallback to latest available transform at Time(0) to tolerate edge timestamp jitter (e.g. final frame)
            try:
                tf = self.buffer.lookup_transform_core(tgt, src, Time(nanoseconds=0))
            except Exception as e:
                raise RuntimeError(
                    f"CRITICAL TF TRANSFORMATION FAILURE: Failed to transform point from '{src}' to '{tgt}' at {time_ns} ns. "
                    f"Likely missing transform chain (e.g. 'map' -> 'odom' or '/tf_static'). "
                    f"Aborting episode conversion to avoid corrupting spatial conditioning. Underlying error: {e}"
                ) from e

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
