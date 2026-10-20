# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""
Perception context management & active vision gating for LeKiwi Autonomous Chess Robot.

Encapsulates:
1. PerceptionContext state machine & hardware gating (valves, Hailo-8 NPU, LeRobot wrist cam).
2. Latched ROS 2 publishers (/perception_context) and /orchestrator/set_perception_context service.
"""

from __future__ import annotations

import threading
from typing import Any

from lekiwi_interfaces.msg import PerceptionContext
from lekiwi_interfaces.srv import SetPerceptionContext
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from lekiwi_orchestrator.fsm import (
    PERCEPTION_CONTEXT_NAMES,
    is_perception_transition_allowed,
)


class PerceptionContextManager:
    """Manages vision operational context and controls camera valve gating and NPU state.

    Provides a latched /perception_context topic and ROS 2 service server decoupled
    from macro mission orchestration.

    Thread-safety:
        Context transitions and queries are synchronized using a reentrant lock (`_lock`).
    """

    def __init__(
        self,
        node: Node,
        perception_context_topic: str = "/perception_context",
        set_perception_service_name: str = "/orchestrator/set_perception_context",
        callback_group: Any = None,
        initial_context: int = PerceptionContext.IDLE_STANDBY,
    ) -> None:
        """Initialize perception context publisher, service server, and initial latched state."""
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

        # Seed latched topic immediately so late-joining perception nodes receive initial state
        self._publish_current_context()

    @property
    def context(self) -> int:
        """Return the current active PerceptionContext enum value under state lock."""
        with self._lock:
            return self._current_context

    def set_context(self, requested_context: int) -> bool:
        """Transition perception context and update downstream camera valves.

        Validates requested context against the allowed transition matrix before applying.
        Publishes the updated context to the latched ROS 2 topic upon success.

        Args:
            requested_context: Target PerceptionContext integer value.

        Returns:
            True if transition was permitted and broadcast, False if rejected by matrix.
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
        """Publish the current perception context message to the latched topic."""
        p_msg = PerceptionContext()
        p_msg.value = self._current_context
        self._perception_context_pub.publish(p_msg)

    def handle_set_context_service(
        self,
        request: SetPerceptionContext.Request,
        response: SetPerceptionContext.Response,
    ) -> SetPerceptionContext.Response:
        """Handle incoming SetPerceptionContext service requests from external clients."""
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
        """Destroy publisher and service handles upon orchestrator shutdown."""
        if hasattr(self, "_service") and self._service is not None:
            self._node.destroy_service(self._service)
        if (
            hasattr(self, "_perception_context_pub")
            and self._perception_context_pub is not None
        ):
            self._node.destroy_publisher(self._perception_context_pub)


