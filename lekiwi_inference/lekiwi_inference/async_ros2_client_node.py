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

"""ROS 2 Jazzy node bridging sensory topics to the remote LeRobot policy server.

Implements the ExecuteChessMove Action Server for physical manipulation, offloading
10D environment_state to the remote policy and streaming joint target commands to ros2_control.
"""

from __future__ import annotations

import threading
import time
from typing import Optional

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticStatusWrapper, Updater
from lekiwi_interfaces.action import ExecuteChessMove
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool, Trigger

from lekiwi_inference.async_client import AsyncInferenceClient, ClientConfig
from lekiwi_inference.transport.zmq_transport import ZmqPolicyTransport
from lekiwi_inference.utils import extract_joint_positions, image_msg_to_bytes

TASK_DESCRIPTION_MAP: dict[int, str] = {
    ExecuteChessMove.Goal.OP_MOVE: "Chess pick and place move",
    ExecuteChessMove.Goal.OP_CLEAR_LEFT: "Clear to left onboard bin",
    ExecuteChessMove.Goal.OP_CLEAR_RIGHT: "Clear to right onboard bin",
    ExecuteChessMove.Goal.OP_PICK: "Pick chess piece",
    ExecuteChessMove.Goal.OP_PLACE: "Place chess piece",
}


class AsyncRos2ClientNode(Node):
    """ROS 2 Jazzy node driving physical-AI arm policy execution over ZeroMQ."""

    def __init__(self, *args, **kwargs) -> None:
        """Declares validated parameters, binds subscriber callbacks, and initializes real-time timers."""
        super().__init__("async_ros2_inference_client", *args, **kwargs)

        # -------------------------------------------------------------------
        # 1. Parameter Declarations
        # -------------------------------------------------------------------
        self.declare_parameter("server_address", "127.0.0.1:8090")
        self.declare_parameter("policy_type", "smolvla")
        self.declare_parameter("repo_id", "DuyKhongCay/smolvla_lekiwi_chess")
        self.declare_parameter("control_rate", 25.0)
        self.declare_parameter("max_age_s", 0.08)
        self.declare_parameter("actions_per_chunk", 40)
        self.declare_parameter("chunk_size_threshold", 0.15)
        self.declare_parameter("observation_filter_epsilon", 0.05)
        self.declare_parameter("wrist_cam_topic", "/cameras/usb_wrist/image_raw/compressed")
        self.declare_parameter("side_cam_topic", "/cameras/usb_side/image_raw/compressed")
        self.declare_parameter("overhead_cam_topic", "/cameras/stereo_right/image_raw/compressed")
        self.declare_parameter("joints_topic", "/joint_states")
        self.declare_parameter("fwd_cmd_topic", "/arm_forward_controller/commands")
        self.declare_parameter("action_name", "/manipulation/execute_chess_move")
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
        )
        self.declare_parameter("aggregation_fn", "weighted_average")
        self.declare_parameter("auto_start", True)

        # Landmark poses (SSOT injected from Tier 0 orchestrator.yaml:L41)
        self.declare_parameter(
            "named_poses.stow",
            [1.5616, -1.7411, 1.5570, 0.7747, -2.7888, 0.0813],
        )

        # Episode convergence & completion criteria (Zero Backward-Compatibility)
        self.declare_parameter("convergence.max_duration_s", 180.0)
        self.declare_parameter("convergence.min_actions", 50)
        self.declare_parameter("convergence.stow_tolerance_rad", 0.20)
        self.declare_parameter("convergence.settle_steps", 25)
        self.declare_parameter("convergence.starvation_timeout_s", 2.0)


        # Network transport & safety watchdogs
        self.declare_parameter("transport.config_timeout_s", 60.0)
        self.declare_parameter("transport.infer_timeout_ms", 500)
        self.declare_parameter("stream_timeout_s", 2.0)

        # Retrieve parameters
        self.server_address = str(self.get_parameter("server_address").value)
        self.policy_type = str(self.get_parameter("policy_type").value)
        self.repo_id = str(self.get_parameter("repo_id").value)
        self.control_rate = float(self.get_parameter("control_rate").value)
        self.max_age_s = float(self.get_parameter("max_age_s").value)
        self.actions_per_chunk = int(self.get_parameter("actions_per_chunk").value)
        self.chunk_size_threshold = float(self.get_parameter("chunk_size_threshold").value)
        self.observation_filter_epsilon = float(self.get_parameter("observation_filter_epsilon").value)
        self.wrist_cam_topic = str(self.get_parameter("wrist_cam_topic").value)
        self.side_cam_topic = str(self.get_parameter("side_cam_topic").value)
        self.overhead_cam_topic = str(self.get_parameter("overhead_cam_topic").value)
        self.joints_topic = str(self.get_parameter("joints_topic").value)
        self.fwd_cmd_topic = str(self.get_parameter("fwd_cmd_topic").value)
        self.action_name = str(self.get_parameter("action_name").value)
        self.arm_joints = list(self.get_parameter("arm_joints").value)
        self.aggregation_fn = str(self.get_parameter("aggregation_fn").value)
        self.auto_start = bool(self.get_parameter("auto_start").value)

        self.stow_pose = np.array(
            self.get_parameter("named_poses.stow").value,
            dtype=np.float64,
        )
        self.max_duration_s = float(
            self.get_parameter("convergence.max_duration_s").value
        )
        self.min_actions = int(
            self.get_parameter("convergence.min_actions").value
        )
        self.stow_tolerance_rad = float(
            self.get_parameter("convergence.stow_tolerance_rad").value
        )
        self.settle_steps = int(
            self.get_parameter("convergence.settle_steps").value
        )
        self.starvation_timeout_s = float(
            self.get_parameter("convergence.starvation_timeout_s").value
        )


        self.config_timeout_s = float(
            self.get_parameter("transport.config_timeout_s").value
        )
        self.infer_timeout_ms = int(
            self.get_parameter("transport.infer_timeout_ms").value
        )
        self.stream_timeout_s = float(
            self.get_parameter("stream_timeout_s").value
        )

        # -------------------------------------------------------------------
        # 2. State & Concurrency Structures
        # -------------------------------------------------------------------
        self._lock = threading.RLock()
        self._latest_wrist_bytes: Optional[bytes] = None
        self._latest_wrist_stamp: float = 0.0
        self._latest_side_bytes: Optional[bytes] = None
        self._latest_side_stamp: float = 0.0
        self._latest_overhead_bytes: Optional[bytes] = None
        self._latest_overhead_stamp: float = 0.0
        self._latest_joint_positions: Optional[np.ndarray] = None
        self._latest_joint_stamp: float = 0.0

        self._estop_active = False
        self._is_active = False

        # Action execution tracking
        self._active_goal_lock = threading.Lock()
        self._active_goal_handle = None
        self._active_env_state: Optional[np.ndarray] = None
        self._active_task: str = "Chess pick and place move"
        self._active_goal_start_time: float = 0.0
        self._active_settle_count: int = 0
        self._active_policy_actions_executed: int = 0
        self._active_starvation_start_time: Optional[float] = None
        self._goal_finished_event = threading.Event()
        self._goal_success = False
        self._goal_result_message = ""

        # Initialize Transport and Client
        self.transport = ZmqPolicyTransport(
            server_address=self.server_address,
            timeout_ms=self.infer_timeout_ms,
            config_timeout_ms=int(self.config_timeout_s * 1000),
        )
        client_cfg = ClientConfig(
            server_address=self.server_address,
            policy_type=self.policy_type,
            repo_id=self.repo_id,
            fps=self.control_rate,
            actions_per_chunk=self.actions_per_chunk,
            chunk_size_threshold=self.chunk_size_threshold,
            observation_filter_epsilon=self.observation_filter_epsilon,
            aggregation_fn=self.aggregation_fn,
            features={
                "observation.state": {"shape": (12,), "dtype": "float32"},
                "observation.images.wrist": {"dtype": "image", "shape": (384, 384, 3)},
                "observation.images.side": {"dtype": "image", "shape": (384, 384, 3)},
                "observation.images.overhead": {"dtype": "image", "shape": (384, 384, 3)},
            },
        )
        self.client = AsyncInferenceClient(transport=self.transport, config=client_cfg, logger=self.get_logger())

        # Callback Groups
        self._action_cb_group = ReentrantCallbackGroup()
        self._sub_cb_group = MutuallyExclusiveCallbackGroup()

        # -------------------------------------------------------------------
        # 3. ROS 2 Subscriptions & Publishers
        # -------------------------------------------------------------------
        self.sub_wrist = self.create_subscription(
            CompressedImage,
            self.wrist_cam_topic,
            self._wrist_cam_callback,
            qos_profile_sensor_data,
            callback_group=self._sub_cb_group,
        )
        self.sub_side = self.create_subscription(
            CompressedImage,
            self.side_cam_topic,
            self._side_cam_callback,
            qos_profile_sensor_data,
            callback_group=self._sub_cb_group,
        )
        self.sub_overhead = self.create_subscription(
            CompressedImage,
            self.overhead_cam_topic,
            self._overhead_cam_callback,
            qos_profile_sensor_data,
            callback_group=self._sub_cb_group,
        )
        self.sub_joints = self.create_subscription(
            JointState,
            self.joints_topic,
            self._joint_state_callback,
            qos_profile_sensor_data,
            callback_group=self._sub_cb_group,
        )

        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            self.fwd_cmd_topic,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
        )

        # Service for enabling/pausing inference client
        self.srv_enable = self.create_service(
            SetBool,
            "/lekiwi/inference/enable",
            self._handle_enable_service,
            callback_group=self._action_cb_group,
        )

        # Service for triggering emergency stop
        self.srv_estop = self.create_service(
            Trigger,
            "/lekiwi/estop",
            self._handle_estop_service,
            callback_group=self._action_cb_group,
        )

        # Action Server for ExecuteChessMove
        self._action_server = ActionServer(
            self,
            ExecuteChessMove,
            self.action_name,
            execute_callback=self._execute_chess_move_goal,
            goal_callback=self._handle_goal_received,
            cancel_callback=self._handle_cancel_received,
            callback_group=self._action_cb_group,
        )

        # -------------------------------------------------------------------
        # 4. Standard ROS 2 Diagnostic Updater
        # -------------------------------------------------------------------
        self.updater = Updater(self)
        self.updater.setHardwareID("RaspberryPi5-ZeroMQ")
        self.updater.add("Policy Inference Status", self._produce_diagnostics)

        # -------------------------------------------------------------------
        # 5. Deterministic Timers
        # -------------------------------------------------------------------
        dt = 1.0 / self.control_rate
        self.control_timer = self.create_timer(dt, self._control_timer_callback)

        if self.auto_start:
            self._start_inference()

    def _start_inference(self) -> bool:
        """Initializes client worker thread and marks inference active."""
        if self.client.start():
            self._is_active = True
            self.get_logger().info("Inference client connected and ready.")
            return True
        else:
            self.get_logger().error("Failed to connect inference client to policy server.")
            return False

    def _stop_inference(self) -> None:
        """Pauses worker thread and clears buffered motion targets."""
        self._is_active = False
        self.client.stop()
        self.get_logger().info("Inference client stopped.")

    def _wrist_cam_callback(self, msg: CompressedImage) -> None:
        with self._lock:
            self._latest_wrist_bytes = image_msg_to_bytes(msg)
            self._latest_wrist_stamp = time.time()

    def _side_cam_callback(self, msg: CompressedImage) -> None:
        with self._lock:
            self._latest_side_bytes = image_msg_to_bytes(msg)
            self._latest_side_stamp = time.time()

    def _overhead_cam_callback(self, msg: CompressedImage) -> None:
        with self._lock:
            self._latest_overhead_bytes = image_msg_to_bytes(msg)
            self._latest_overhead_stamp = time.time()

    def _joint_state_callback(self, msg: JointState) -> None:
        positions = extract_joint_positions(msg, self.arm_joints)
        if positions is not None:
            with self._lock:
                self._previous_joint_positions = (
                    self._latest_joint_positions.copy()
                    if self._latest_joint_positions is not None
                    else positions.copy()
                )
                self._latest_joint_positions = positions
                self._latest_joint_stamp = time.time()

    def _handle_estop_service(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Triggers emergency stop, halts policy streaming, and aborts any active goal."""
        del request
        self._estop_active = True
        self.get_logger().warn("E-STOP triggered via /lekiwi/estop service! Halting motion.")

        with self._active_goal_lock:
            if self._active_goal_handle is not None and self._active_goal_handle.is_active:
                self._goal_success = False
                self._goal_result_message = "Emergency stop triggered via /lekiwi/estop service"
                self._goal_finished_event.set()

        response.success = True
        response.message = "Emergency stop activated; arm motion halted"
        return response

    def _is_sensor_data_fresh(self, now: float) -> bool:
        max_age = self.max_age_s
        with self._lock:
            if (
                self._latest_wrist_bytes is None
                or self._latest_side_bytes is None
                or self._latest_overhead_bytes is None
                or self._latest_joint_positions is None
            ):
                return False

            if (now - self._latest_wrist_stamp) > max_age:
                return False
            if (now - self._latest_side_stamp) > max_age:
                return False
            if (now - self._latest_overhead_stamp) > max_age:
                return False
            if (now - self._latest_joint_stamp) > max_age:
                return False
            return True

    # -----------------------------------------------------------------------
    # Action Server Callbacks
    # -----------------------------------------------------------------------

    def _handle_goal_received(self, goal_request: ExecuteChessMove.Goal) -> GoalResponse:
        with self._active_goal_lock:
            if self._active_goal_handle is not None and self._active_goal_handle.is_active:
                self.get_logger().warn("Rejecting goal: arm is currently busy with another active goal.")
                return GoalResponse.REJECT

            if not self._is_active or not self.transport.is_connected():
                self.get_logger().error(
                    "Rejecting ExecuteChessMove goal: inference client is inactive or transport is disconnected."
                )
                return GoalResponse.REJECT

        op = goal_request.operation
        if op not in TASK_DESCRIPTION_MAP:
            self.get_logger().error(f"Rejecting goal: unknown operation enum {op}.")
            return GoalResponse.REJECT

        if len(goal_request.environment_state) != 6:
            self.get_logger().error(
                f"Rejecting goal: expected 6D spatial coordinates in environment_state [src(3), tgt(3)], got {len(goal_request.environment_state)}D."
            )
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def _handle_cancel_received(self, goal_handle) -> CancelResponse:
        self.get_logger().warn("Received cancellation request for active manipulation goal.")
        return CancelResponse.ACCEPT

    def _execute_chess_move_goal(self, goal_handle) -> ExecuteChessMove.Result:
        goal = goal_handle.request
        op = goal.operation
        instruction = goal.instruction.strip() if goal.instruction else ""
        task = instruction or TASK_DESCRIPTION_MAP.get(op, "Chess manipulation move")
        env_state = np.array(goal.environment_state[:6], dtype=np.float32)

        self.get_logger().info(
            f"Executing manipulation goal: op={op} ('{task}'), env_state={env_state.tolist()}..."
        )

        with self._active_goal_lock:
            self._active_goal_handle = goal_handle
            self._active_env_state = env_state
            self._active_task = task
            self._active_goal_start_time = time.time()
            self._active_settle_count = 0
            self._active_policy_actions_executed = 0
            self._active_starvation_start_time = None
            self._goal_finished_event.clear()
            self._goal_success = False
            self._goal_result_message = ""

        # Flush stale actions from any previous transaction
        self.client.reset_episode()

        poll_interval = 0.05
        max_duration = self.max_duration_s

        while not self._goal_finished_event.wait(timeout=poll_interval):
            if goal_handle.is_cancel_requested:
                with self._active_goal_lock:
                    self._goal_success = False
                    self._goal_result_message = "Goal canceled by client request."
                    self._goal_finished_event.set()
                break

            elapsed = time.time() - self._active_goal_start_time
            if elapsed >= max_duration:
                with self._active_goal_lock:
                    self._goal_success = False
                    self._goal_result_message = f"Policy rollout timeout after {elapsed:.1f}s."
                    self._goal_finished_event.set()
                break

        total_duration = float(time.time() - self._active_goal_start_time)
        result = ExecuteChessMove.Result()
        result.execution_time_sec = total_duration

        with self._active_goal_lock:
            success = self._goal_success
            result.success = success
            result.message = self._goal_result_message or ("Success" if success else "Failed")

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warn(f"Goal canceled: {result.message}")
            elif success:
                goal_handle.succeed()
                self.get_logger().info(f"Goal completed successfully in {total_duration:.2f}s")
            else:
                goal_handle.abort()
                self.get_logger().error(f"Goal aborted: {result.message}")

            self._active_goal_handle = None
            self._active_env_state = None

        self.client.reset_episode()
        return result

    # -----------------------------------------------------------------------
    # 25 Hz Control & Observation Loop
    # -----------------------------------------------------------------------

    def _control_timer_callback(self) -> None:
        now = time.time()

        with self._lock:
            current_joints = (
                self._latest_joint_positions.copy()
                if self._latest_joint_positions is not None
                else None
            )
            wrist_bytes = self._latest_wrist_bytes
            side_bytes = self._latest_side_bytes
            overhead_bytes = self._latest_overhead_bytes

        if current_joints is None:
            return

        with self._active_goal_lock:
            active_handle = self._active_goal_handle
            base_env_state = self._active_env_state
            task = self._active_task
            start_time = self._active_goal_start_time

        # 1. 12D Sensorimotor Observation Synthesis & Dispatch
        if active_handle is not None and base_env_state is not None:
            # S_t = [q1..q6, x_src, y_src, z_src, x_tgt, y_tgt, z_tgt] (12D)
            obs_state_12d = np.concatenate([current_joints[:6], base_env_state[:6]]).astype(np.float32)

            if self.client.ready_to_send(obs_state_12d) and self._is_sensor_data_fresh(now):
                raw_obs = {
                    "observation.images.wrist": wrist_bytes,
                    "observation.images.side": side_bytes,
                    "observation.images.overhead": overhead_bytes,
                    "observation.state": obs_state_12d,
                    "task": task,
                }
                self.client.submit_observation(raw_obs, timestamp=now)

        # 2. Action Actuation & Starvation Watchdog
        if self._estop_active:
            # E-stop holds current position
            safe_cmd = current_joints
        elif active_handle is not None and self.client.actions_available():
            candidate = self.client.pop_action()
            if candidate is not None:
                safe_cmd = candidate
                with self._active_goal_lock:
                    self._active_policy_actions_executed += 1
                    self._active_starvation_start_time = None
            else:
                safe_cmd = current_joints
        else:
            # Default hold position
            safe_cmd = current_joints
            if active_handle is not None and active_handle.is_active:
                with self._active_goal_lock:
                    if self._active_starvation_start_time is None:
                        self._active_starvation_start_time = now
                    elif (now - self._active_starvation_start_time) >= self.starvation_timeout_s:
                        self._goal_success = False
                        self._goal_result_message = (
                            f"Policy action stream starved for > {self.starvation_timeout_s:.1f}s."
                        )
                        self._goal_finished_event.set()

        msg = Float64MultiArray()
        msg.data = [float(val) for val in safe_cmd]
        self.cmd_pub.publish(msg)

        # 3. Publish Feedback & Evaluate Convergence
        if active_handle is not None and active_handle.is_active:
            elapsed = now - start_time
            progress = min(100.0, float((elapsed / self.max_duration_s) * 100.0))

            feedback = ExecuteChessMove.Feedback()
            feedback.current_phase = "POLICY_ROLLOUT"
            feedback.progress_percent = progress
            feedback.instruction = task
            active_handle.publish_feedback(feedback)

            with self._active_goal_lock:
                executed_count = self._active_policy_actions_executed

            is_terminal, message = self._evaluate_convergence(
                current_joints=current_joints,
                elapsed=elapsed,
                executed_actions=executed_count,
            )

            if is_terminal and message:
                with self._active_goal_lock:
                    self._goal_success = ("Successfully converged" in message)
                    self._goal_result_message = message
                    self._goal_finished_event.set()

    def _evaluate_convergence(
        self,
        current_joints: np.ndarray,
        elapsed: float,
        executed_actions: int,
    ) -> tuple[bool, Optional[str]]:
        """Evaluates whether the policy rollout has reached terminal stow posture.

        Single Responsibility: Verifies execution gating, timeout watchdogs,
        and kinematic convergence to the pre-calibrated landmark stow pose.

        Returns:
            Tuple of (is_terminal, optional_result_message).
        """
        # 1. Episode timeout watchdog
        if elapsed >= self.max_duration_s:
            return True, f"Episode timed out after {elapsed:.1f}s without reaching stow posture."

        # 2. Minimum execution prerequisite
        if executed_actions < self.min_actions:
            self._active_settle_count = 0
            return False, None

        # 3. Kinematic proximity to stow landmark (first 5 arm joints)
        arm_stow_target = self.stow_pose[:5]
        arm_current = current_joints[:5]
        stow_err = float(np.max(np.abs(arm_current - arm_stow_target)))

        if stow_err < self.stow_tolerance_rad:
            self._active_settle_count += 1
            if self._active_settle_count >= self.settle_steps:
                return True, (
                    f"Successfully converged to stow posture after {elapsed:.2f}s "
                    f"({executed_actions} actions executed, stow_err={stow_err:.3f} rad)."
                )
        else:
            self._active_settle_count = 0

        return False, None

    def _produce_diagnostics(
        self, stat: DiagnosticStatusWrapper
    ) -> DiagnosticStatusWrapper:
        """Produces diagnostic telemetry for the standard ROS /diagnostics pipeline."""
        is_connected = self.transport.is_connected()
        rtt_latency = self.transport.ping()

        # Determine health status level and summary
        if self._estop_active:
            stat.summary(DiagnosticStatus.WARN, "Emergency Stop Active")
        elif not is_connected:
            stat.summary(DiagnosticStatus.ERROR, "ZeroMQ Policy Server Disconnected")
        elif rtt_latency > 150.0:
            stat.summary(DiagnosticStatus.WARN, f"High Latency ({rtt_latency:.1f} ms)")
        elif not self._is_active:
            stat.summary(DiagnosticStatus.OK, "Idle (Inference Paused)")
        else:
            stat.summary(DiagnosticStatus.OK, "Streaming Active")

        # Telemetry key-values
        stat.add("transport_connected", str(is_connected))
        stat.add("inference_active", str(self._is_active))
        stat.add("estop_active", str(self._estop_active))
        stat.add("queue_size", str(self.client.get_queue_size()))
        stat.add("obs_filtered_count", str(self.client.get_obs_filtered_count()))
        stat.add("rtt_latency_ms", f"{rtt_latency:.1f}")
        stat.add("server_address", self.server_address)
        stat.add("policy_type", self.policy_type)
        stat.add("repo_id", self.repo_id)

        with self._active_goal_lock:
            is_goal_active = (
                self._active_goal_handle is not None and self._active_goal_handle.is_active
            )
            stat.add("active_goal", str(is_goal_active))
            stat.add("active_task", self._active_task if is_goal_active else "None")

        return stat

    def _handle_enable_service(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        """Handles external service calls to start or pause policy client connection."""
        if request.data:
            if not self._is_active:
                success = self._start_inference()
                response.success = success
                response.message = "Inference client started" if success else "Failed to connect"
            else:
                response.success = True
                response.message = "Inference is already connected"
        else:
            self._stop_inference()
            response.success = True
            response.message = "Inference client stopped"
        return response

    def destroy_node(self) -> None:
        """Cleans up client thread and transport sockets upon node shutdown."""
        self._stop_inference()
        super().destroy_node()


def main(args=None) -> None:
    """Initializes rclpy runtime and spins the async inference client node using MultiThreadedExecutor."""
    rclpy.init(args=args)
    node = AsyncRos2ClientNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
