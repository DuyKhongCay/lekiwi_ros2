"""Readiness-gated Nav2 startup owned by orchestration, not the control monitor."""

import math
import time

from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class NavigationStartup:
    """Start Nav2 once a fresh readiness heartbeat arrives, with bounded service waits."""

    def __init__(self, node):
        """Confine mutable startup state to one mutually-exclusive callback group."""
        self.node = node
        self.timeout = node.declare_parameter(
            "navigation.readiness_timeout_sec", 1.0
        ).value
        self.service_timeout = node.declare_parameter(
            "navigation.service_timeout_sec", 5.0
        ).value
        service = node.declare_parameter(
            "navigation.lifecycle_service", "/lifecycle_manager_navigation/manage_nodes"
        ).value
        if not service or any(
            not math.isfinite(v) or v <= 0 for v in (self.timeout, self.service_timeout)
        ):
            raise ValueError(
                "Navigation timeouts must be positive and lifecycle service nonempty"
            )
        self.group = MutuallyExclusiveCallbackGroup()
        self.client = node.create_client(
            ManageLifecycleNodes, service, callback_group=self.group
        )
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.sub = node.create_subscription(
            Bool, "/system/tf_ready", self.on_ready, qos, callback_group=self.group
        )
        self.ready = False
        self.last_heartbeat = None
        self.started = False
        self.pending = None
        self.dispatch_time = 0.0
        self.next_attempt = 0.0
        self.timer = node.create_timer(
            0.2,
            self.tick,
            callback_group=self.group,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

    def on_ready(self, msg):
        """Use local receipt time to expire a lost gatekeeper independently of ROS time."""
        self.ready = bool(msg.data)
        self.last_heartbeat = time.monotonic()

    def tick(self):
        """Poll completed futures in the serialized timer instead of detached callbacks."""
        now = time.monotonic()
        if self.pending is not None:
            if self.pending.done():
                try:
                    self.started = bool(self.pending.result().success)
                except Exception as err:  # noqa: BLE001
                    self.node.get_logger().warning(f"Nav2 startup failed: {err}")
                self.pending = None
                self.next_attempt = now + 1.0
            elif now - self.dispatch_time >= self.service_timeout:
                self.client.remove_pending_request(self.pending)
                self.pending.cancel()
                self.pending = None
                self.next_attempt = now + 1.0
                self.node.get_logger().warning(
                    "Nav2 startup timed out; retrying after backoff"
                )
            return
        if (
            self.started
            or not self.ready
            or self.last_heartbeat is None
            or now - self.last_heartbeat > self.timeout
            or now < self.next_attempt
            or not self.client.service_is_ready()
        ):
            return
        req = ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.STARTUP)
        self.dispatch_time = now
        self.pending = self.client.call_async(req)
