# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

import sys
import time

from lekiwi_control.fsm import MODE_NAMES
from lekiwi_interfaces.msg import CameraMode
from lekiwi_interfaces.srv import SetCamMode
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray


class ModeDemoClient(Node):
    """Drive the camera mode sequence and validate observable streams."""

    def __init__(self):
        super().__init__("camera_mode_demo_client")
        self.declare_parameter("orchestrator_service", "/orchestrator/set_mode")
        self.declare_parameter("settle_seconds", 2.0)
        self.settle_seconds = float(self.get_parameter("settle_seconds").value)
        srv_name = self.get_parameter("orchestrator_service").value
        self.client = self.create_client(SetCamMode, srv_name)
        self.counts = {"hailo": 0, "right": 0, "wrist": 0, "side": 0}

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Detection2DArray,
            "/chess/detections_2d",
            lambda _: self._count("hailo"),
            sensor_qos,
        )
        self.create_subscription(
            Image,
            "/cameras/stereo_right/image_raw",
            lambda _: self._count("right"),
            sensor_qos,
        )
        self.create_subscription(
            Image,
            "/cameras/usb_wrist/image_raw",
            lambda _: self._count("wrist"),
            sensor_qos,
        )
        self.create_subscription(
            Image,
            "/cameras/usb_side/image_raw",
            lambda _: self._count("side"),
            sensor_qos,
        )

    def _count(self, stream):
        """Increment one debug subscriber frame counter."""
        self.counts[stream] += 1

    def spin_for(self, seconds):
        """Process callbacks for a bounded wall-clock interval."""
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def request_mode(self, mode):
        """Send one mode request from the main thread."""
        request = SetCamMode.Request()
        request.requested_mode.value = mode
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        return future.result() if future.done() else None

    def wait_for_camera_hub(self, timeout_sec=20.0):
        """Wait for the orchestrator service to be available."""
        if self.client.wait_for_service(timeout_sec=timeout_sec):
            self.spin_for(0.5)
            return True
        return False

    def validate_mode(self, mode, before):
        """Validate status and stream activity for one mode."""
        delta = {name: self.counts[name] - before[name] for name in self.counts}
        if mode == CameraMode.CHESS_THINKING:
            return delta["hailo"] > 0
        if mode == CameraMode.MANIPULATION_LEROBOT:
            return all(delta[name] > 0 for name in ("right", "wrist", "side"))
        return True

    def run_demo(self):
        """Execute the canonical FSM sequence."""
        if not self.client.wait_for_service(timeout_sec=20.0):
            self.get_logger().error("Orchestrator service is unavailable")
            return False
        if not self.wait_for_camera_hub():
            self.get_logger().error("Active camera hub status is unavailable")
            return False
        sequence = [
            CameraMode.STANDBY,
            CameraMode.NAVIGATING,
            CameraMode.CHESS_THINKING,
            CameraMode.MANIPULATION_LEROBOT,
            CameraMode.CHESS_THINKING,
            CameraMode.STANDBY,
        ]
        for mode in sequence:
            response = self.request_mode(mode)
            if response is None or not response.success:
                message = response.message if response else "timeout"
                self.get_logger().error(f"{MODE_NAMES[mode]} failed: {message}")
                return False
            self.spin_for(0.5)
            before = dict(self.counts)
            self.spin_for(self.settle_seconds)
            if not self.validate_mode(mode, before):
                self.get_logger().error(
                    f"{MODE_NAMES[mode]} stream validation failed; counts={self.counts}"
                )
                return False
            self.get_logger().info(f"{MODE_NAMES[mode]} validated")
        return True


def main(args=None):
    """Run the camera mode integration demo."""
    rclpy.init(args=args)
    node = ModeDemoClient()
    success = node.run_demo()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
