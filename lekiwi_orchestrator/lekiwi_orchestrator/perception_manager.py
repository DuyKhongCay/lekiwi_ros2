# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""
Perception Management & Active Vision Domain for LeKiwi Autonomous Chess Robot.

Encapsulates:
1. PerceptionContext state machine & hardware gating (valves, Hailo-8 NPU, LeRobot wrist cam).
2. Latched ROS 2 publishers (/perception_context, /camera_mode) and /orchestrator/set_perception_context service.
3. Active observation geometry: Camera FOV, standoff distances, and candidate viewpoints calculation.
"""

from __future__ import annotations

import math
import threading
from typing import Any

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from lekiwi_interfaces.msg import PerceptionContext
from lekiwi_interfaces.srv import SetPerceptionContext
from lekiwi_orchestrator.fsm import (
    PERCEPTION_CONTEXT_NAMES,
    is_perception_transition_allowed,
)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert planar yaw angle (radians) to geometry_msgs Quaternion."""
    half_yaw = yaw * 0.5
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(half_yaw),
        w=math.cos(half_yaw),
    )


def compute_observation_pose(
    board_x: float,
    board_y: float,
    board_yaw: float = 0.0,
    standoff_distance: float = 0.65,
    angle_offset: float = 0.0,
    frame_id: str = "map",
) -> PoseStamped:
    """
    Compute a single observation PoseStamped oriented towards the chessboard center.

    :param board_x: X coordinate of chessboard center in reference frame.
    :param board_y: Y coordinate of chessboard center in reference frame.
    :param board_yaw: Base orientation of the board in radians.
    :param standoff_distance: Distance (m) from board center to robot base footprint.
    :param angle_offset: Angular offset (radians) relative to the baseline orientation.
    :param frame_id: Reference frame for PoseStamped header (typically 'map').
    """
    viewpoint_angle = board_yaw + angle_offset
    robot_x = board_x - standoff_distance * math.cos(viewpoint_angle)
    robot_y = board_y - standoff_distance * math.sin(viewpoint_angle)

    facing_yaw = math.atan2(board_y - robot_y, board_x - robot_x)

    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose = Pose(
        position=Point(x=robot_x, y=robot_y, z=0.0),
        orientation=yaw_to_quaternion(facing_yaw),
    )
    return pose


def generate_candidate_observation_poses(
    board_x: float,
    board_y: float,
    board_yaw: float = 0.0,
    standoff_distance: float = 0.65,
    frame_id: str = "map",
) -> list[PoseStamped]:
    """
    Generate an ordered list of candidate observation poses covering different viewpoints.

    Order of viewpoints:
    1. Primary Baseline (offset 0.0)
    2. Diagonal Right (+45 degrees / +pi/4)
    3. Diagonal Left (-45 degrees / -pi/4)
    4. Lateral Right (+90 degrees / +pi/2)
    5. Lateral Left (-90 degrees / -pi/2)
    """
    angle_offsets = [
        0.0,
        math.pi / 4.0,
        -math.pi / 4.0,
        math.pi / 2.0,
        -math.pi / 2.0,
    ]

    return [
        compute_observation_pose(
            board_x=board_x,
            board_y=board_y,
            board_yaw=board_yaw,
            standoff_distance=standoff_distance,
            angle_offset=offset,
            frame_id=frame_id,
        )
        for offset in angle_offsets
    ]


class PerceptionContextCoordinator:
    """
    Manages vision operational context and controls camera valve gating / NPU state.
    Provides latched /perception_context topic and service handling decoupled from mission orchestration.
    """

    def __init__(
        self,
        node: Node,
        perception_context_topic: str = "/perception_context",
        set_perception_service_name: str = "/orchestrator/set_perception_context",
        callback_group: Any = None,
        initial_context: int = PerceptionContext.IDLE_STANDBY,
    ) -> None:
        self._node = node
        self._lock = threading.RLock()
        self._current_context = initial_context

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._perception_context_pub = self._node.create_publisher(
            PerceptionContext, perception_context_topic, latched_qos
        )

        self._service = self._node.create_service(
            SetPerceptionContext,
            set_perception_service_name,
            self.handle_set_context_service,
            callback_group=callback_group,
        )

        # Publish initial latched message
        self._publish_current_context()

    @property
    def context(self) -> int:
        with self._lock:
            return self._current_context

    def set_context(self, requested_context: int) -> bool:
        """
        Transition perception context verifying against allowed transition matrix.
        Publishes updated context to latched topic upon success.
        """
        with self._lock:
            current = self._current_context
            if requested_context == current:
                return True

            if not is_perception_transition_allowed(current, requested_context):
                curr_name = PERCEPTION_CONTEXT_NAMES.get(current, "UNKNOWN")
                req_name = PERCEPTION_CONTEXT_NAMES.get(requested_context, "UNKNOWN")
                self._node.get_logger().warn(
                    f"[PERCEPTION] Rejected transition: {curr_name} -> {req_name}"
                )
                return False

            self._current_context = requested_context
            curr_name = PERCEPTION_CONTEXT_NAMES.get(current, "UNKNOWN")
            new_name = PERCEPTION_CONTEXT_NAMES.get(requested_context, "UNKNOWN")
            self._node.get_logger().info(
                f"[PERCEPTION] Switched context: {curr_name} -> {new_name}"
            )

        self._publish_current_context()
        return True

    def _publish_current_context(self) -> None:
        p_msg = PerceptionContext()
        p_msg.value = self._current_context
        self._perception_context_pub.publish(p_msg)

    def handle_set_context_service(
        self,
        request: SetPerceptionContext.Request,
        response: SetPerceptionContext.Response,
    ) -> SetPerceptionContext.Response:
        """ROS 2 Service callback for external/manual context setting."""
        req_val = request.requested_context.value
        success = self.set_context(req_val)
        response.success = success
        response.applied_context.value = self.context
        response.message = (
            f"Applied context {PERCEPTION_CONTEXT_NAMES.get(self.context, 'UNKNOWN')}"
            if success
            else f"Failed to switch to context {PERCEPTION_CONTEXT_NAMES.get(req_val, 'UNKNOWN')}"
        )
        return response

    def destroy(self) -> None:
        """Destroy publisher and service server upon shutdown."""
        if hasattr(self, "_service") and self._service is not None:
            self._node.destroy_service(self._service)
        if (
            hasattr(self, "_perception_context_pub")
            and self._perception_context_pub is not None
        ):
            self._node.destroy_publisher(self._perception_context_pub)
