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
from geometry_msgs.msg import Point, PointStamped
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
import tf2_geometry_msgs
import tf2_ros

from lekiwi_inference.async_client import AsyncInferenceClient, ClientConfig
from lekiwi_inference.joint_command_smoother import JointCommandSmoother, apply_soft_deadband
from lekiwi_inference.transport.zmq_transport import ZmqPolicyTransport
from lekiwi_inference.utils import extract_joint_positions, image_msg_to_bytes


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
        self.declare_parameter("repo_id", "DuyKhongCay/lekiwi_chess_smolvla")
        self.declare_parameter("control_rate", 25.0)
        self.declare_parameter("max_age_s", 0.08)
        self.declare_parameter("actions_per_chunk", 30)
        self.declare_parameter("chunk_size_threshold", 0.35)
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

        # Real-Time Chunking (RTC) parameters
        self.declare_parameter("enable_rtc", True)
        self.declare_parameter("default_inference_delay", 14)
        self.declare_parameter("rtc_execution_horizon", 18)

        # Reference frames and spatial guidance
        self.declare_parameter("tool_frame", "gripper")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("landmarks.stow", [0.0704, -0.1882, 0.1757])
        self.declare_parameter("landmarks.clear_left", [-0.0029, 0.1435, 0.1702])
        self.declare_parameter("landmarks.clear_right", [-0.0027, -0.1451, 0.1633])
        self.declare_parameter("delta_reach_threshold_m", 0.035)

        # Spatial reach thresholds & phase dwell-time filtering
        self.declare_parameter("dwell_time_s", 0.24)
        self.declare_parameter("xy_reach_threshold_m", 0.035)
        self.declare_parameter("z_reach_threshold_m", 0.08)
        self.declare_parameter("gripper_close_threshold", 0.15)
        self.declare_parameter("delta_deadband_m", 0.012)
        self.declare_parameter("actuation_rate", 50.0)
        self.declare_parameter("ema_alpha", 0.75)
        self.declare_parameter("enable_actuation_smoothing", True)

        # Landmark poses (SSOT joint angles for terminal posture)
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
        self.enable_rtc = bool(self.get_parameter("enable_rtc").value)
        self.default_inference_delay = int(self.get_parameter("default_inference_delay").value)
        self.rtc_execution_horizon = int(self.get_parameter("rtc_execution_horizon").value)

        self.tool_frame = str(self.get_parameter("tool_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.landmark_stow = np.array(self.get_parameter("landmarks.stow").value, dtype=np.float32)
        self.landmark_clear_left = np.array(self.get_parameter("landmarks.clear_left").value, dtype=np.float32)
        self.landmark_clear_right = np.array(self.get_parameter("landmarks.clear_right").value, dtype=np.float32)
        self.delta_reach_threshold_m = float(self.get_parameter("delta_reach_threshold_m").value)
        self.dwell_time_s = float(self.get_parameter("dwell_time_s").value)
        self.xy_reach_threshold_m = float(self.get_parameter("xy_reach_threshold_m").value)
        self.z_reach_threshold_m = float(self.get_parameter("z_reach_threshold_m").value)
        self.gripper_close_threshold = float(self.get_parameter("gripper_close_threshold").value)
        self.delta_deadband_m = float(self.get_parameter("delta_deadband_m").value)
        self.actuation_rate = float(self.get_parameter("actuation_rate").value)
        self.ema_alpha = float(self.get_parameter("ema_alpha").value)
        self.enable_actuation_smoothing = bool(self.get_parameter("enable_actuation_smoothing").value)

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
        # 1.1 TF2 Listener
        # -------------------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

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
        self._active_task: str = ""
        self._active_op: int = ExecuteChessMove.Goal.OP_MOVE
        self._active_p_pick: np.ndarray = np.zeros(3, dtype=np.float32)
        self._active_p_place: np.ndarray = np.zeros(3, dtype=np.float32)
        self._active_p_bin: np.ndarray = np.zeros(3, dtype=np.float32)
        self._active_subtask_phase: int = 0
        self._active_goal_xyz: np.ndarray = np.zeros(3, dtype=np.float32)
        self._active_goal_start_time: float = 0.0
        self._active_settle_count: int = 0
        self._active_policy_actions_executed: int = 0
        self._active_starvation_start_time: Optional[float] = None
        self._goal_finished_event = threading.Event()
        self._goal_success = False
        self._goal_result_message = ""
        self._phase_dwell_ticks: int = 0
        self._required_dwell_ticks: int = max(1, int(round(self.dwell_time_s * self.control_rate)))

        # 50Hz Actuation smoother & raw action register
        self._smoother = JointCommandSmoother(
            ema_alpha=self.ema_alpha,
            num_joints=len(self.arm_joints),
        )
        self._latest_raw_candidate: Optional[np.ndarray] = None

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
            enable_rtc=self.enable_rtc,
            default_inference_delay=self.default_inference_delay,
            rtc_execution_horizon=self.rtc_execution_horizon,
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
        # 5. Deterministic Timers (Decoupled 25Hz Cognition & 50Hz Actuation)
        # -------------------------------------------------------------------
        dt_cognition = 1.0 / self.control_rate
        self.cognition_timer = self.create_timer(dt_cognition, self._cognition_timer_cb)
        self.control_timer = self.cognition_timer  # Reference alias for compatibility

        dt_actuation = 1.0 / self.actuation_rate
        self.actuation_timer = self.create_timer(dt_actuation, self._actuation_timer_cb)

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
    # Spatial Transformation Helpers
    # -----------------------------------------------------------------------

    def _transform_point_to_base(self, pt: Point, target_frame: str) -> np.ndarray:
        """Transforms a geometry_msgs/Point from target_frame to base_frame."""
        if not target_frame or target_frame == self.base_frame:
            return np.array([pt.x, pt.y, pt.z], dtype=np.float32)
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                self.base_frame,
                target_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
            pt_stamped = PointStamped()
            pt_stamped.header.frame_id = target_frame
            pt_stamped.point = pt
            transformed = tf2_geometry_msgs.do_transform_point(pt_stamped, tf_stamped)
            return np.array(
                [transformed.point.x, transformed.point.y, transformed.point.z],
                dtype=np.float32,
            )
        except Exception as exc:
            self.get_logger().error(
                f"Failed to transform point from '{target_frame}' to '{self.base_frame}': {exc}"
            )
            raise RuntimeError(f"TF transform unavailable: {exc}") from exc

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

        instruction = (goal_request.instruction or "").strip()
        if not instruction:
            self.get_logger().error("Rejecting ExecuteChessMove goal: instruction is empty.")
            return GoalResponse.REJECT

        op = goal_request.operation
        if op not in (
            ExecuteChessMove.Goal.OP_MOVE,
            ExecuteChessMove.Goal.OP_CLEAR_LEFT,
            ExecuteChessMove.Goal.OP_CLEAR_RIGHT,
            ExecuteChessMove.Goal.OP_PICK,
            ExecuteChessMove.Goal.OP_PLACE,
        ):
            self.get_logger().error(f"Rejecting goal: unknown operation enum {op}.")
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def _handle_cancel_received(self, goal_handle) -> CancelResponse:
        self.get_logger().warn("Received cancellation request for active manipulation goal.")
        return CancelResponse.ACCEPT

    def _execute_chess_move_goal(self, goal_handle) -> ExecuteChessMove.Result:
        goal = goal_handle.request
        op = goal.operation
        instruction = goal.instruction.strip()
        target_frame = goal.target_frame or "chessboard_frame"

        try:
            p_pick = self._transform_point_to_base(goal.pick_point, target_frame)
            p_place = self._transform_point_to_base(goal.place_point, target_frame)
        except Exception as exc:
            self.get_logger().error(f"Aborting goal: failed to transform coordinates: {exc}")
            goal_handle.abort()
            res = ExecuteChessMove.Result()
            res.success = False
            res.message = f"Coordinate transform failed: {exc}"
            return res

        if op == ExecuteChessMove.Goal.OP_CLEAR_LEFT:
            p_bin = self.landmark_clear_left
        elif op == ExecuteChessMove.Goal.OP_CLEAR_RIGHT:
            p_bin = self.landmark_clear_right
        else:
            p_bin = self.landmark_clear_right

        initial_goal_xyz = p_place if op == ExecuteChessMove.Goal.OP_PLACE else p_pick

        self.get_logger().info(
            f"Executing manipulation goal: op={op} ('{instruction}'), "
            f"target_frame='{target_frame}', initial_target={initial_goal_xyz.tolist()}..."
        )

        with self._active_goal_lock:
            self._active_goal_handle = goal_handle
            self._active_task = instruction
            self._active_op = op
            self._active_p_pick = p_pick
            self._active_p_place = p_place
            self._active_p_bin = p_bin
            self._active_subtask_phase = 0
            self._phase_dwell_ticks = 0
            self._active_goal_xyz = initial_goal_xyz.copy()
            self._active_goal_start_time = time.time()
            self._active_settle_count = 0
            self._active_policy_actions_executed = 0
            self._active_starvation_start_time = None
            self._goal_finished_event.clear()
            self._goal_success = False
            self._goal_result_message = ""

        # Flush stale actions from any previous transaction and reset smoother
        self.client.reset_episode()
        with self._lock:
            if self._latest_joint_positions is not None:
                self._smoother.reset(self._latest_joint_positions)
        self._latest_raw_candidate = None

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

        self.client.reset_episode()
        return result

    # -----------------------------------------------------------------------
    # Decoupled Timers: 25Hz Cognition Loop & 50Hz Actuation Loop
    # -----------------------------------------------------------------------

    def _cognition_timer_cb(self) -> None:
        now = time.time()

        with self._active_goal_lock:
            active_handle = self._active_goal_handle
            if active_handle is None or not active_handle.is_active:
                # IDLE: Do NOT stream observations or update targets
                return

            task = self._active_task
            start_time = self._active_goal_start_time
            op = self._active_op
            subtask_phase = self._active_subtask_phase
            current_target_xyz = self._active_goal_xyz.copy()

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

        # 1. End-Effector TCP Lookup via TF
        try:
            tf_tcp = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tool_frame,
                rclpy.time.Time(),
            )
            ee_xyz = np.array(
                [
                    tf_tcp.transform.translation.x,
                    tf_tcp.transform.translation.y,
                    tf_tcp.transform.translation.z,
                ],
                dtype=np.float32,
            )
        except Exception as exc:
            self.get_logger().warn(
                f"Failed to lookup TCP '{self.tool_frame}' in '{self.base_frame}': {exc}",
                throttle_duration_sec=1.0,
            )
            return

        # 2. Dual-Condition Subtask Goal State Machine with Dwell Time Verification
        delta_xyz = (current_target_xyz - ee_xyz).astype(np.float32)
        dist_xy = float(np.linalg.norm(delta_xyz[:2]))
        tcp_z = float(ee_xyz[2])

        grip = float(current_joints[5])  # arm_gripper joint (rad)
        near_target = (dist_xy < self.xy_reach_threshold_m) and (tcp_z < self.z_reach_threshold_m)
        target_changed = False

        with self._active_goal_lock:
            if op in (
                ExecuteChessMove.Goal.OP_MOVE,
                ExecuteChessMove.Goal.OP_CLEAR_LEFT,
                ExecuteChessMove.Goal.OP_CLEAR_RIGHT,
            ):
                if subtask_phase == 0:
                    # Grasp piece: gripper closed (grip <= gripper_close_threshold) AND near target
                    if grip <= self.gripper_close_threshold and near_target:
                        self._phase_dwell_ticks += 1
                        if self._phase_dwell_ticks >= self._required_dwell_ticks:
                            self._active_subtask_phase = 1
                            self._phase_dwell_ticks = 0
                            self._active_goal_xyz = (
                                self._active_p_place
                                if op == ExecuteChessMove.Goal.OP_MOVE
                                else self._active_p_bin
                            )
                            target_changed = True
                            self.get_logger().info(
                                f"Subtask Phase 0 -> 1 confirmed after {self.dwell_time_s:.2f}s dwell: "
                                f"grip={grip:.3f}, dist_xy={dist_xy:.4f}m, tcp_z={tcp_z:.4f}m"
                            )
                    else:
                        self._phase_dwell_ticks = 0
                elif subtask_phase == 1:
                    # Release piece: gripper open (grip > gripper_close_threshold) AND near target
                    if grip > self.gripper_close_threshold and near_target:
                        self._phase_dwell_ticks += 1
                        if self._phase_dwell_ticks >= self._required_dwell_ticks:
                            self._active_subtask_phase = 2
                            self._phase_dwell_ticks = 0
                            self._active_goal_xyz = self.landmark_stow
                            target_changed = True
                            self.get_logger().info(
                                f"Subtask Phase 1 -> 2 confirmed after {self.dwell_time_s:.2f}s dwell: "
                                f"grip={grip:.3f}, dist_xy={dist_xy:.4f}m, tcp_z={tcp_z:.4f}m"
                            )
                    else:
                        self._phase_dwell_ticks = 0
            elif op == ExecuteChessMove.Goal.OP_PICK:
                if subtask_phase == 0:
                    if grip <= self.gripper_close_threshold and near_target:
                        self._phase_dwell_ticks += 1
                        if self._phase_dwell_ticks >= self._required_dwell_ticks:
                            self._active_subtask_phase = 1
                            self._phase_dwell_ticks = 0
                            self._active_goal_xyz = self.landmark_stow
                            target_changed = True
                            self.get_logger().info(
                                f"Subtask Phase 0 -> 1 confirmed (pick) after dwell: grip={grip:.3f}"
                            )
                    else:
                        self._phase_dwell_ticks = 0
            elif op == ExecuteChessMove.Goal.OP_PLACE:
                if subtask_phase == 0:
                    if grip > self.gripper_close_threshold and near_target:
                        self._phase_dwell_ticks += 1
                        if self._phase_dwell_ticks >= self._required_dwell_ticks:
                            self._active_subtask_phase = 1
                            self._phase_dwell_ticks = 0
                            self._active_goal_xyz = self.landmark_stow
                            target_changed = True
                            self.get_logger().info(
                                f"Subtask Phase 0 -> 1 confirmed (place) after dwell: grip={grip:.3f}"
                            )
                    else:
                        self._phase_dwell_ticks = 0

            if target_changed:
                delta_xyz = (self._active_goal_xyz - ee_xyz).astype(np.float32)

        if target_changed:
            self.client.flush_action_queue(keep_steps=2)
            self.client.notify_phase_transition()

        # 3. 12D Sensorimotor Dynamic Relative Delta Observation Synthesis & Dispatch
        # Apply soft deadband to delta_xyz to prevent limit-cycle hunting near target
        deadband_delta = apply_soft_deadband(
            delta_xyz,
            xy_threshold_m=self.delta_deadband_m,
            z_threshold_m=self.delta_deadband_m,
        )
        obs_state_12d = np.concatenate([current_joints[:6], ee_xyz, deadband_delta]).astype(np.float32)

        if self.client.ready_to_send(obs_state_12d) and self._is_sensor_data_fresh(now):
            raw_obs = {
                "observation.images.wrist": wrist_bytes,
                "observation.images.side": side_bytes,
                "observation.images.overhead": overhead_bytes,
                "observation.state": obs_state_12d,
                "task": task,
            }
            self.client.submit_observation(raw_obs, timestamp=now)

        # 4. Action Queue Management & Starvation Watchdog
        if self.client.actions_available():
            candidate = self.client.pop_action()
            if candidate is not None:
                self._latest_raw_candidate = candidate
                self._smoother.update_policy_target(candidate)
                with self._active_goal_lock:
                    self._active_policy_actions_executed += 1
                    self._active_starvation_start_time = None
        else:
            with self._active_goal_lock:
                if self._active_starvation_start_time is None:
                    self._active_starvation_start_time = now
                elif (now - self._active_starvation_start_time) >= self.starvation_timeout_s:
                    self._goal_success = False
                    self._goal_result_message = (
                        f"Policy action stream starved for > {self.starvation_timeout_s:.1f}s."
                    )
                    self._goal_finished_event.set()

        # 5. Publish Feedback & Evaluate Convergence
        elapsed = now - start_time
        progress = min(100.0, float((elapsed / self.max_duration_s) * 100.0))

        feedback = ExecuteChessMove.Feedback()
        feedback.current_phase = f"SUBTASK_PHASE_{self._active_subtask_phase}"
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

    def _actuation_timer_cb(self) -> None:
        """50Hz Actuation timer callback publishing commands to /arm_forward_controller/commands."""
        with self._active_goal_lock:
            active_handle = self._active_goal_handle
            if active_handle is None or not active_handle.is_active:
                # IDLE: Do NOT publish commands when idle
                return

        with self._lock:
            current_joints = (
                self._latest_joint_positions.copy()
                if self._latest_joint_positions is not None
                else None
            )

        if current_joints is None:
            return

        if self._estop_active:
            safe_cmd = current_joints
        elif self.enable_actuation_smoothing and self._smoother.is_initialized:
            smoothed = self._smoother.step_actuation()
            safe_cmd = smoothed if smoothed is not None else current_joints
        elif self._latest_raw_candidate is not None:
            safe_cmd = self._latest_raw_candidate
        else:
            safe_cmd = current_joints

        msg = Float64MultiArray()
        msg.data = [float(val) for val in safe_cmd]
        self.cmd_pub.publish(msg)

    def _control_timer_callback(self) -> None:
        """Synchronous stepping executing both cognition and actuation cycles."""
        self._cognition_timer_cb()
        self._actuation_timer_cb()

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
        stat.add("rtc_enabled", str(self.enable_rtc))
        stat.add("rtc_inference_delay", str(self.client.get_last_inference_delay()))
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
