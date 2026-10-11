# Copyright 2026 LeKiwi Robotics Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Thin ROS 2 node coordinating sensor ingest, 50Hz action actuation, and safety supervision."""

import threading
import time
from typing import Optional

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
import numpy as np
from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Bool, Float64MultiArray
from std_srvs.srv import SetBool

from lekiwi_inference.async_client import AsyncInferenceClient, ClientConfig
from lekiwi_inference.safety_watchdog import SafetyState, SafetyWatchdog
from lekiwi_inference.transport.zmq_transport import ZmqPolicyTransport
from lekiwi_inference.utils import extract_joint_positions, image_msg_to_bytes


class AsyncRos2ClientNode(Node):
    """ROS 2 Jazzy node bridging sensory topics to the remote policy server and controlling robot arm joints."""

    def __init__(self) -> None:
        """Declares validated parameters, binds subscriber callbacks, and initializes real-time timers."""
        super().__init__("async_ros2_inference_client")

        # -------------------------------------------------------------------
        # 1. Parameter Declarations with Descriptors & Validated Ranges
        # -------------------------------------------------------------------
        self.declare_parameter(
            "server_address",
            "127.0.0.1:8090",
            ParameterDescriptor(description="ZMQ Policy Server endpoint (host:port)"),
        )
        self.declare_parameter(
            "policy_type",
            "act",
            ParameterDescriptor(description="Physical-AI policy model architecture"),
        )
        self.declare_parameter(
            "repo_id",
            "legalaspro/act_so101_pnp_microsanity_20_50hz_v0",
            ParameterDescriptor(description="HuggingFace repository ID or local weights path"),
        )
        self.declare_parameter(
            "control_rate",
            50.0,
            ParameterDescriptor(
                description="Deterministic arm forward command rate in Hz",
                floating_point_range=[FloatingPointRange(from_value=10.0, to_value=100.0, step=1.0)],
            ),
        )
        self.declare_parameter(
            "max_age_s",
            0.25,
            ParameterDescriptor(
                description="Max allowable action age before watchdog holds position",
                floating_point_range=[FloatingPointRange(from_value=0.05, to_value=2.0, step=0.01)],
            ),
        )
        self.declare_parameter(
            "actions_per_chunk",
            100,
            ParameterDescriptor(
                description="Horizon chunk size predicted by ACT policy",
                integer_range=[IntegerRange(from_value=10, to_value=500, step=1)],
            ),
        )
        self.declare_parameter(
            "wrist_cam_topic",
            "/cameras/usb_wrist/image_raw/compressed",
            ParameterDescriptor(description="Topic for wrist-mounted camera compressed feed"),
        )
        self.declare_parameter(
            "side_cam_topic",
            "/cameras/usb_side/image_raw/compressed",
            ParameterDescriptor(description="Topic for side/top camera compressed feed"),
        )
        self.declare_parameter(
            "joints_topic",
            "/joint_states",
            ParameterDescriptor(description="Topic for robot joint telemetry"),
        )
        self.declare_parameter(
            "fwd_cmd_topic",
            "/arm_forward_controller/commands",
            ParameterDescriptor(description="Output forward command controller topic"),
        )
        self.declare_parameter(
            "arm_joints",
            [
                "arm_shoulder_pan",
                "arm_shoulder_lift",
                "arm_elbow_flex",
                "arm_wrist_flex",
                "arm_wrist_roll",
                "arm_gripper",
            ],
            ParameterDescriptor(description="Ordered joint names commanded by policy"),
        )
        self.declare_parameter(
            "aggregation_fn",
            "weighted_average",
            ParameterDescriptor(description="Temporal smoothing algorithm: weighted_average/conservative/latest_only"),
        )
        self.declare_parameter(
            "auto_start",
            True,
            ParameterDescriptor(description="Automatically establish connection and start inference"),
        )

        # Retrieve parsed parameters
        self.server_address = str(self.get_parameter("server_address").value)
        self.policy_type = str(self.get_parameter("policy_type").value)
        self.repo_id = str(self.get_parameter("repo_id").value)
        self.control_rate = float(self.get_parameter("control_rate").value)
        self.max_age_s = float(self.get_parameter("max_age_s").value)
        self.actions_per_chunk = int(self.get_parameter("actions_per_chunk").value)
        self.wrist_cam_topic = str(self.get_parameter("wrist_cam_topic").value)
        self.side_cam_topic = str(self.get_parameter("side_cam_topic").value)
        self.joints_topic = str(self.get_parameter("joints_topic").value)
        self.fwd_cmd_topic = str(self.get_parameter("fwd_cmd_topic").value)
        self.arm_joints = list(self.get_parameter("arm_joints").value)
        self.aggregation_fn = str(self.get_parameter("aggregation_fn").value)
        self.auto_start = bool(self.get_parameter("auto_start").value)

        # -------------------------------------------------------------------
        # 2. State & Concurrency Structures
        # -------------------------------------------------------------------
        self._lock = threading.Lock()
        self._latest_wrist_bytes: Optional[bytes] = None
        self._latest_side_bytes: Optional[bytes] = None
        self._latest_joint_positions: Optional[np.ndarray] = None
        self._is_active = False

        # Initialize Safety Watchdog
        self.watchdog = SafetyWatchdog(max_age_s=self.max_age_s)

        # Initialize Transport and Client
        self.transport = ZmqPolicyTransport(server_address=self.server_address)
        client_cfg = ClientConfig(
            server_address=self.server_address,
            policy_type=self.policy_type,
            repo_id=self.repo_id,
            fps=self.control_rate,
            actions_per_chunk=self.actions_per_chunk,
            aggregation_fn=self.aggregation_fn,
            features={
                "observation.state": {"shape": (len(self.arm_joints),), "dtype": "float32"},
                "observation.images.wrist": {"dtype": "image", "shape": (480, 640, 3)},
                "observation.images.side": {"dtype": "image", "shape": (480, 640, 3)},
            },
        )
        self.client = AsyncInferenceClient(transport=self.transport, config=client_cfg)

        # -------------------------------------------------------------------
        # 3. ROS 2 Subscriptions & Publishers
        # -------------------------------------------------------------------
        self.sub_wrist = self.create_subscription(
            CompressedImage,
            self.wrist_cam_topic,
            self._wrist_cam_callback,
            qos_profile_sensor_data,
        )
        self.sub_side = self.create_subscription(
            CompressedImage,
            self.side_cam_topic,
            self._side_cam_callback,
            qos_profile_sensor_data,
        )
        self.sub_joints = self.create_subscription(
            JointState,
            self.joints_topic,
            self._joint_state_callback,
            qos_profile_sensor_data,
        )
        self.sub_estop = self.create_subscription(
            Bool,
            "/lekiwi/estop",
            self._estop_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
        )

        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            self.fwd_cmd_topic,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.diag_pub = self.create_publisher(
            DiagnosticStatus,
            "/lekiwi/inference/status",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
        )

        # Service for enabling/pausing inference
        self.srv_enable = self.create_service(
            SetBool,
            "/lekiwi/inference/enable",
            self._handle_enable_service,
        )

        # -------------------------------------------------------------------
        # 4. Deterministic Timers
        # -------------------------------------------------------------------
        dt = 1.0 / self.control_rate
        self.control_timer = self.create_timer(dt, self._control_timer_callback)
        self.watchdog_timer = self.create_timer(0.1, self._watchdog_timer_callback)

        if self.auto_start:
            self._start_inference()

    def _start_inference(self) -> bool:
        """Initializes client worker thread and marks inference active."""
        if self.client.start():
            self._is_active = True
            self.get_logger().info("Inference streaming successfully activated.")
            return True
        else:
            self.get_logger().error("Failed to start inference client.")
            return False

    def _stop_inference(self) -> None:
        """Pauses worker thread and clears buffered motion targets."""
        self._is_active = False
        self.client.stop()
        self.get_logger().info("Inference streaming paused.")

    def _wrist_cam_callback(self, msg: CompressedImage) -> None:
        """Stores latest compressed wrist camera image snapshot atomically."""
        with self._lock:
            self._latest_wrist_bytes = image_msg_to_bytes(msg)
        self._try_dispatch_observation()

    def _side_cam_callback(self, msg: CompressedImage) -> None:
        """Stores latest compressed side camera image snapshot atomically."""
        with self._lock:
            self._latest_side_bytes = image_msg_to_bytes(msg)
        self._try_dispatch_observation()

    def _joint_state_callback(self, msg: JointState) -> None:
        """Extracts and updates ordered arm joint feedback positions atomically."""
        positions = extract_joint_positions(msg, self.arm_joints)
        if positions is not None:
            with self._lock:
                self._latest_joint_positions = positions

    def _estop_callback(self, msg: Bool) -> None:
        """Processes emergency stop triggers to instantly inhibit motion publishing."""
        self.watchdog.trigger_estop(msg.data)
        if msg.data:
            self.get_logger().warn("E-STOP triggered via /lekiwi/estop! Stopping arm motion.")

    def _try_dispatch_observation(self) -> None:
        """Submits complete synchronized sensor observations to background worker queue."""
        if not self._is_active:
            return

        with self._lock:
            wrist = self._latest_wrist_bytes
            side = self._latest_side_bytes
            joints = self._latest_joint_positions

        if wrist is not None and side is not None and joints is not None:
            obs = {
                "observation.state": joints,
                "observation.images.wrist": wrist,
                "observation.images.side": side,
            }
            self.client.submit_observation(obs)

    def _control_timer_callback(self) -> None:
        """Executes real-time 50Hz control step, arbitrating between planned actions and position hold."""
        with self._lock:
            current_joints = (
                self._latest_joint_positions.copy()
                if self._latest_joint_positions is not None
                else None
            )

        if current_joints is None:
            return

        # Check for new action arrival to update watchdog timer
        last_action_ts = self.client.get_last_action_timestamp()
        if last_action_ts > 0:
            self.watchdog.notify_action_received(last_action_ts)

        candidate_action = self.client.get_next_action() if self._is_active else None

        safe_cmd, state = self.watchdog.arbitrate_command(
            candidate_action=candidate_action,
            current_joint_positions=current_joints,
        )

        msg = Float64MultiArray()
        msg.data = [float(val) for val in safe_cmd]
        self.cmd_pub.publish(msg)

    def _watchdog_timer_callback(self) -> None:
        """Evaluates communication health at 10Hz and publishes diagnostic telemetry."""
        state = self.watchdog.evaluate_state()
        age = self.watchdog.get_action_age()
        timeouts = self.watchdog.get_timeout_count()

        if state == SafetyState.STALE_HOLD and self._is_active:
            self.get_logger().warn(
                f"Inference latency exceeded! Action age: {age:.3f}s > {self.max_age_s}s. Holding arm position.",
                throttle_duration_sec=2.0,
            )

        # Publish diagnostic status
        diag = DiagnosticStatus()
        diag.name = "LeKiwi Policy Inference"
        diag.hardware_id = "RaspberryPi5-ZeroMQ"
        diag.level = (
            DiagnosticStatus.OK
            if state == SafetyState.HEALTHY
            else DiagnosticStatus.WARN
            if state == SafetyState.STALE_HOLD
            else DiagnosticStatus.ERROR
        )
        diag.message = state.value
        diag.values = [
            KeyValue(key="ActionAge_s", value=f"{age:.4f}"),
            KeyValue(key="QueueSize", value=str(self.client.action_queue.size())),
            KeyValue(key="ServerRTT_ms", value=f"{self.transport.ping():.1f}"),
            KeyValue(key="TimeoutEvents", value=str(timeouts)),
            KeyValue(key="IsActive", value=str(self._is_active)),
        ]
        self.diag_pub.publish(diag)

    def _handle_enable_service(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        """Handles external service calls to start or pause policy motion."""
        if request.data:
            if not self._is_active:
                success = self._start_inference()
                response.success = success
                response.message = (
                    "Inference streaming started" if success else "Failed to start inference"
                )
            else:
                response.success = True
                response.message = "Inference is already running"
        else:
            self._stop_inference()
            response.success = True
            response.message = "Inference streaming paused"
        return response

    def destroy_node(self) -> None:
        """Cleans up client thread and transport sockets upon node shutdown."""
        self._stop_inference()
        super().destroy_node()


def main(args=None) -> None:
    """Initializes rclpy runtime and spins the async inference client node."""
    rclpy.init(args=args)
    node = AsyncRos2ClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

