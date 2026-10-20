# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Shared utility functions and helpers across the orchestrator package."""

from __future__ import annotations

import math
import time
from typing import Any

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion


def get_node_now_sec(node: Any) -> float:
    """Extract current timestamp in seconds from node clock, fallback to monotonic clock."""
    try:
        if hasattr(node, "get_clock"):
            clk = node.get_clock()
            if clk is not None:
                now = clk.now()
                if hasattr(now, "nanoseconds") and isinstance(now.nanoseconds, (int, float)):
                    return float(now.nanoseconds) * 1e-9
    except Exception:
        pass
    return time.monotonic()


def get_header_stamp_sec(
    header_or_msg: Any, fallback_sec: float | None = None
) -> float:
    """Extract timestamp in seconds from a ROS 2 Header or message with header.stamp."""
    header = getattr(header_or_msg, "header", header_or_msg)
    stamp = getattr(header, "stamp", None)
    if stamp is not None:
        sec = getattr(stamp, "sec", None)
        nanosec = getattr(stamp, "nanosec", None)
        if isinstance(sec, (int, float)) and isinstance(nanosec, (int, float)):
            total = float(sec) + float(nanosec) * 1e-9
            if total > 0.0:
                return total
    return fallback_sec if fallback_sec is not None else time.monotonic()


def normalize_angle(angle: float) -> float:
    """Normalize any planar angle in radians to the principal interval [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert planar yaw angle in radians to a geometry_msgs Quaternion."""
    half_yaw = yaw * 0.5
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(half_yaw),
        w=math.cos(half_yaw),
    )


def create_pose2d_stamped(
    x: float,
    y: float,
    yaw: float,
    frame_id: str = "map",
    z: float = 0.0,
) -> PoseStamped:
    """Construct a PoseStamped from planar SE(2) coordinates and heading."""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose = Pose(
        position=Point(x=float(x), y=float(y), z=float(z)),
        orientation=yaw_to_quaternion(yaw),
    )
    return pose


__all__ = [
    "get_node_now_sec",
    "get_header_stamp_sec",
    "normalize_angle",
    "yaw_to_quaternion",
    "create_pose2d_stamped",
]
