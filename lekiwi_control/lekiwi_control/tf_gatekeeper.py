# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""
TF Tree Readiness Gatekeeper for LeKiwi mobile manipulation robot.

Monitors localization and kinematic readiness:
1. Zero-velocity check: Confirms robot is stationary after manual gamepad teleop.
2. Global EKF convergence: Verifies covariance from /odometry/global is within chess tolerances.
3. Kinematic joint completeness: Verifies all 6 arm joints and wheel states are active.
4. TF2 chain completeness & freshness: Verifies map -> odom -> base_footprint -> gripperframe.

Publishes latched /system/tf_ready (TRANSIENT_LOCAL) to notify Task Orchestrator when game can start.
"""

from __future__ import annotations

import math
from typing import Sequence, Set, Tuple

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

# Default parameters stored directly in script
DEFAULT_REQUIRED_ARM_JOINTS = (
    "arm_shoulder_pan",
    "arm_shoulder_lift",
    "arm_elbow_flex",
    "arm_wrist_flex",
    "arm_wrist_roll",
    "arm_gripper",
)
DEFAULT_MAX_TRANSFORM_AGE_SEC = (
    0.30  # 300 ms (relaxed from 0.15s to tolerate Pi 5 load & jitter)
)
DEFAULT_MAX_POS_VARIANCE = (
    0.0012  # ~3.5cm tolerance (accommodates visual update cycle breathing)
)
DEFAULT_MAX_YAW_VARIANCE = 0.0030  # ~3.1 deg tolerance
DEFAULT_MAX_STOP_VELOCITY = (
    0.03  # 3 cm/s linear velocity (tolerate wheel encoder jitter)
)
DEFAULT_MAX_STOP_ANGULAR_VEL = 0.08  # 0.08 rad/s angular velocity (tolerate IMU bias)
DEFAULT_CHECK_FREQUENCY_HZ = 10.0
DEFAULT_TRIGGER_NAV2 = True
DEFAULT_NAV2_LIFECYCLE_SERVICE = "/lifecycle_manager_navigation/manage_nodes"
DEFAULT_AUTO_PAUSE_NAV2_ON_TF_LOSS = False
DEFAULT_NAV2_RETRY_INTERVAL_SEC = 2.0


# ================= Pure helper functions for testability =================


def check_robot_stationary(
    vx: float,
    vy: float,
    wz: float,
    max_linear_vel: float = DEFAULT_MAX_STOP_VELOCITY,
    max_angular_vel: float = DEFAULT_MAX_STOP_ANGULAR_VEL,
) -> bool:
    """Return True if robot linear and angular speeds are below stop thresholds."""
    speed = math.hypot(vx, vy)
    return (speed <= max_linear_vel) and (abs(wz) <= max_angular_vel)


def check_covariance_converged(
    covariance: Sequence[float],
    max_pos_var: float = DEFAULT_MAX_POS_VARIANCE,
    max_yaw_var: float = DEFAULT_MAX_YAW_VARIANCE,
) -> Tuple[bool, float, float]:
    """
    Extract pos variance (var_x + var_y) and yaw variance from 6x6 covariance.
    Returns (is_converged, pos_variance, yaw_variance).
    """
    if len(covariance) < 36:
        return False, float("inf"), float("inf")

    cov_matrix = np.array(covariance, dtype=np.float64).reshape((6, 6))
    pos_var = float(cov_matrix[0, 0] + cov_matrix[1, 1])
    yaw_var = float(cov_matrix[5, 5])

    # Check for NaN, negative variance, or values exceeding threshold
    if math.isnan(pos_var) or math.isnan(yaw_var) or pos_var < 0.0 or yaw_var < 0.0:
        return False, float("inf"), float("inf")

    converged = (pos_var <= max_pos_var) and (yaw_var <= max_yaw_var)
    return converged, pos_var, yaw_var


def check_arm_joints_complete(
    received_joints: Set[str],
    required_joints: Set[str],
) -> Tuple[bool, Set[str]]:
    """Return True if all required arm joints are present in received joints."""
    missing = required_joints - received_joints
    return (len(missing) == 0), missing


def check_stamp_freshness(
    current_time_sec: float,
    stamp_time_sec: float,
    max_age_sec: float = DEFAULT_MAX_TRANSFORM_AGE_SEC,
) -> Tuple[bool, float]:
    """Calculate transform/message age and verify it does not exceed max_age_sec."""
    age = abs(current_time_sec - stamp_time_sec)
    return (age <= max_age_sec), age


# ================= ROS 2 Node Implementation =================


class TfReadinessGatekeeper(Node):
    """Coordinates TF readiness verification and gates downstream systems."""

    def __init__(self):
        super().__init__("tf_readiness_gatekeeper")

        # Declare parameters with embedded defaults (no external YAML required)
        self.declare_parameter("max_transform_age_sec", DEFAULT_MAX_TRANSFORM_AGE_SEC)
        self.declare_parameter("max_pos_variance", DEFAULT_MAX_POS_VARIANCE)
        self.declare_parameter("max_yaw_variance", DEFAULT_MAX_YAW_VARIANCE)
        self.declare_parameter("max_stop_velocity", DEFAULT_MAX_STOP_VELOCITY)
        self.declare_parameter("max_stop_angular_vel", DEFAULT_MAX_STOP_ANGULAR_VEL)
        self.declare_parameter("check_frequency_hz", DEFAULT_CHECK_FREQUENCY_HZ)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("ee_frame", "gripperframe")
        self.declare_parameter("board_frame", "chessboard_frame")
        self.declare_parameter("required_arm_joints", list(DEFAULT_REQUIRED_ARM_JOINTS))
        self.declare_parameter("trigger_nav2", DEFAULT_TRIGGER_NAV2)
        self.declare_parameter("nav2_lifecycle_service", DEFAULT_NAV2_LIFECYCLE_SERVICE)
        self.declare_parameter(
            "auto_pause_nav2_on_tf_loss", DEFAULT_AUTO_PAUSE_NAV2_ON_TF_LOSS
        )

        self._max_age = float(self.get_parameter("max_transform_age_sec").value)
        self._max_pos_var = float(self.get_parameter("max_pos_variance").value)
        self._max_yaw_var = float(self.get_parameter("max_yaw_variance").value)
        self._max_vel = float(self.get_parameter("max_stop_velocity").value)
        self._max_ang_vel = float(self.get_parameter("max_stop_angular_vel").value)
        self._check_freq = float(self.get_parameter("check_frequency_hz").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._ee_frame = str(self.get_parameter("ee_frame").value)
        self._board_frame = str(self.get_parameter("board_frame").value)
        self._required_joints = set(self.get_parameter("required_arm_joints").value)
        self._trigger_nav2 = bool(self.get_parameter("trigger_nav2").value)
        self._nav2_service_name = str(
            self.get_parameter("nav2_lifecycle_service").value
        )
        self._auto_pause_nav2 = bool(
            self.get_parameter("auto_pause_nav2_on_tf_loss").value
        )

        # TF2 Buffer and Listener
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriptions
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._local_odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self._on_local_odom, sensor_qos
        )
        self._global_odom_sub = self.create_subscription(
            Odometry, "/odometry/global", self._on_global_odom, sensor_qos
        )
        self._joint_sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_states, sensor_qos
        )

        # Publishers
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._tf_ready_pub = self.create_publisher(
            Bool, "/system/tf_ready", latched_qos
        )
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )

        # Service
        self._status_srv = self.create_service(
            Trigger, "/system/check_tf_readiness", self._handle_trigger_query
        )

        # Internal State Tracking
        self.is_tf_ready = False
        self._latest_local_odom: Odometry | None = None
        self._latest_global_odom: Odometry | None = None
        self._latest_joint_time_sec: float | None = None
        self._received_joints: Set[str] = set()

        # Nav2 Lifecycle Client & Tracking
        self._nav2_client = self.create_client(
            ManageLifecycleNodes, self._nav2_service_name
        )
        self._nav2_started = False
        self._nav2_call_in_progress = False
        self._nav2_last_dispatch_time = 0.0

        # Evaluation Timer
        period = 1.0 / max(self._check_freq, 1.0)
        self._eval_timer = self.create_timer(period, self._evaluate_system_readiness)

        # Publish initial state (False)
        self._publish_tf_ready(False)

        self.get_logger().info(
            f"TfReadinessGatekeeper initialized ({self._check_freq} Hz). "
            f"Waiting for robot standstill and Global EKF convergence..."
        )

    def _on_local_odom(self, msg: Odometry) -> None:
        self._latest_local_odom = msg

    def _on_global_odom(self, msg: Odometry) -> None:
        self._latest_global_odom = msg

    def _on_joint_states(self, msg: JointState) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self._latest_joint_time_sec = now_sec
        self._received_joints = set(msg.name)

    def _lookup_transform_freshness(
        self,
        target_frame: str,
        source_frame: str,
        now_sec: float,
        is_static: bool = False,
    ) -> Tuple[bool, float, str]:
        """Check if transform exists and is not stale."""
        try:
            if not self.tf_buffer.can_transform(
                target_frame, source_frame, Time(), timeout=Duration(seconds=0.02)
            ):
                return (
                    False,
                    float("inf"),
                    f"Cannot transform {source_frame} -> {target_frame}",
                )

            t: TransformStamped = self.tf_buffer.lookup_transform(
                target_frame, source_frame, Time()
            )
            stamp_sec = Time.from_msg(t.header.stamp).nanoseconds * 1e-9

            # Static transforms are valid indefinitely once available in buffer
            if is_static or stamp_sec == 0.0:
                return True, 0.0, "OK (static)"

            fresh, age = check_stamp_freshness(now_sec, stamp_sec, self._max_age)
            if not fresh:
                return (
                    False,
                    age,
                    f"{source_frame}->{target_frame} is stale ({age:.3f}s > {self._max_age}s)",
                )
            return True, age, "OK"
        except TransformException as exc:
            return False, float("inf"), str(exc)

    def _evaluate_system_readiness(self) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        # 1. Joint state check
        joints_complete = False
        joints_fresh = False
        missing_joints: Set[str] = set()
        if self._latest_joint_time_sec is not None:
            joints_fresh, joint_age = check_stamp_freshness(
                now_sec, self._latest_joint_time_sec, self._max_age
            )
            joints_complete, missing_joints = check_arm_joints_complete(
                self._received_joints, self._required_joints
            )
        joints_ok = joints_complete and joints_fresh

        # 2. Robot stationary check (linear and angular speed near zero)
        is_stationary = False
        speed_val = 0.0
        ang_speed_val = 0.0
        if self._latest_local_odom is not None:
            vx = self._latest_local_odom.twist.twist.linear.x
            vy = self._latest_local_odom.twist.twist.linear.y
            wz = self._latest_local_odom.twist.twist.angular.z
            speed_val = math.hypot(vx, vy)
            ang_speed_val = abs(wz)
            is_stationary = check_robot_stationary(
                vx, vy, wz, self._max_vel, self._max_ang_vel
            )

        # 3. Global EKF convergence check
        ekf_converged = False
        pos_var = float("inf")
        yaw_var = float("inf")
        ekf_fresh = False
        if self._latest_global_odom is not None:
            g_stamp = (
                Time.from_msg(self._latest_global_odom.header.stamp).nanoseconds * 1e-9
            )
            ekf_fresh, _ = check_stamp_freshness(now_sec, g_stamp, self._max_age)
            if ekf_fresh:
                ekf_converged, pos_var, yaw_var = check_covariance_converged(
                    self._latest_global_odom.pose.covariance,
                    self._max_pos_var,
                    self._max_yaw_var,
                )

        # 4. TF Chain checks
        map_odom_ok, _, map_odom_msg = self._lookup_transform_freshness(
            self._map_frame, self._odom_frame, now_sec, is_static=False
        )
        odom_base_ok, _, odom_base_msg = self._lookup_transform_freshness(
            self._odom_frame, self._base_frame, now_sec, is_static=False
        )
        base_ee_ok, _, base_ee_msg = self._lookup_transform_freshness(
            self._base_frame, self._ee_frame, now_sec, is_static=False
        )
        map_board_ok, _, map_board_msg = self._lookup_transform_freshness(
            self._map_frame, self._board_frame, now_sec, is_static=True
        )
        tf_chains_ok = map_odom_ok and odom_base_ok and base_ee_ok and map_board_ok

        # Overall readiness
        system_ready = joints_ok and is_stationary and ekf_converged and tf_chains_ok

        # State transition handling & logging
        if system_ready != self.is_tf_ready:
            self.is_tf_ready = system_ready
            self._publish_tf_ready(self.is_tf_ready)

            if self.is_tf_ready:
                self.get_logger().info(
                    f">>> [TF GATEKEEPER] SYSTEM READY! Base stationary ({speed_val:.3f} m/s), "
                    f"EKF converged (pos_var={pos_var:.6f}, yaw_var={yaw_var:.6f}), all TF chains active."
                )
            else:
                self.get_logger().warning(
                    f"[TF GATEKEEPER] System Unready! Status: "
                    f"Stationary={is_stationary}, EKF_Converged={ekf_converged}, "
                    f"Joints={joints_ok}, TF={tf_chains_ok} "
                    f"(map->odom: {map_odom_msg}, odom->base: {odom_base_msg}, "
                    f"base->ee: {base_ee_msg}, map->board: {map_board_msg})"
                )
                if (
                    self._trigger_nav2
                    and self._nav2_started
                    and self._auto_pause_nav2
                    and not self._nav2_call_in_progress
                    and (
                        now_sec - self._nav2_last_dispatch_time
                        >= DEFAULT_NAV2_RETRY_INTERVAL_SEC
                    )
                ):
                    self._dispatch_nav2_command(
                        ManageLifecycleNodes.Request.PAUSE, "PAUSE", now_sec
                    )

        # Nav2 Lifecycle trigger: Dispatch STARTUP if ready and not yet active
        if (
            self.is_tf_ready
            and self._trigger_nav2
            and not self._nav2_started
            and not self._nav2_call_in_progress
            and (
                now_sec - self._nav2_last_dispatch_time
                >= DEFAULT_NAV2_RETRY_INTERVAL_SEC
            )
        ):
            self._dispatch_nav2_command(
                ManageLifecycleNodes.Request.STARTUP, "STARTUP", now_sec
            )

        # Publish periodic diagnostic telemetry
        self._publish_diagnostics(
            system_ready,
            is_stationary,
            speed_val,
            ekf_converged,
            pos_var,
            yaw_var,
            joints_ok,
            missing_joints,
            tf_chains_ok,
            map_odom_ok,
            odom_base_ok,
            base_ee_ok,
            map_board_ok,
        )

    def _publish_tf_ready(self, ready: bool) -> None:
        msg = Bool()
        msg.data = ready
        self._tf_ready_pub.publish(msg)

    def _publish_diagnostics(
        self,
        system_ready: bool,
        is_stationary: bool,
        speed: float,
        ekf_converged: bool,
        pos_var: float,
        yaw_var: float,
        joints_ok: bool,
        missing_joints: Set[str],
        tf_chains_ok: bool,
        map_odom_ok: bool = False,
        odom_base_ok: bool = False,
        base_ee_ok: bool = False,
        map_board_ok: bool = False,
    ) -> None:
        diag = DiagnosticStatus()
        diag.name = "TF Readiness Gatekeeper"
        diag.hardware_id = "LeKiwi_TF_System"

        if system_ready:
            diag.level = DiagnosticStatus.OK
            diag.message = "System fully localized, stationary, and ready"
        elif not is_stationary:
            diag.level = DiagnosticStatus.WARN
            diag.message = f"Robot is moving ({speed:.3f} m/s > {self._max_vel} m/s)"
        elif not ekf_converged:
            diag.level = DiagnosticStatus.WARN
            diag.message = (
                f"Global EKF converging (pos_var={pos_var:.5f}, yaw_var={yaw_var:.5f})"
            )
        elif not joints_ok:
            diag.level = DiagnosticStatus.ERROR
            diag.message = f"Joint states incomplete (missing: {list(missing_joints)})"
        else:
            diag.level = DiagnosticStatus.WARN
            diag.message = "Waiting for complete TF transforms"

        diag.values = [
            KeyValue(key="is_tf_ready", value=str(system_ready)),
            KeyValue(key="is_stationary", value=str(is_stationary)),
            KeyValue(key="linear_speed_mps", value=f"{speed:.4f}"),
            KeyValue(key="ekf_converged", value=str(ekf_converged)),
            KeyValue(key="pos_variance", value=f"{pos_var:.6f}"),
            KeyValue(key="yaw_variance", value=f"{yaw_var:.6f}"),
            KeyValue(key="joints_healthy", value=str(joints_ok)),
            KeyValue(key="tf_chains_healthy", value=str(tf_chains_ok)),
            KeyValue(key="tf_map_odom", value=str(map_odom_ok)),
            KeyValue(key="tf_odom_base", value=str(odom_base_ok)),
            KeyValue(key="tf_base_ee", value=str(base_ee_ok)),
            KeyValue(key="tf_map_board", value=str(map_board_ok)),
        ]

        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        diag_array.status.append(diag)
        self._diagnostics_pub.publish(diag_array)

    def _handle_trigger_query(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        response.success = self.is_tf_ready
        response.message = (
            "TF Tree verified and solid: Robot ready for game"
            if self.is_tf_ready
            else "TF Tree NOT ready: Robot moving, EKF not converged, or missing transforms"
        )
        return response

    def _dispatch_nav2_command(
        self, command: int, command_name: str, now_sec: float
    ) -> None:
        # Dispatches asynchronous lifecycle command to Nav2 lifecycle manager.
        self._nav2_last_dispatch_time = now_sec
        if not self._nav2_client.service_is_ready():
            self.get_logger().info(
                f"[TF GATEKEEPER] Waiting for '{self._nav2_service_name}' to become available before {command_name}...",
                throttle_duration_sec=2.0,
            )
            return

        self._nav2_call_in_progress = True
        req = ManageLifecycleNodes.Request()
        req.command = command
        self.get_logger().info(
            f"[TF GATEKEEPER] Dispatching Nav2 {command_name} command ({command})..."
        )
        future = self._nav2_client.call_async(req)
        future.add_done_callback(
            lambda f: self._on_nav2_command_completed(f, command, command_name)
        )

    def _on_nav2_command_completed(
        self, future, command: int, command_name: str
    ) -> None:
        # Handles response callback from Nav2 lifecycle manager.
        self._nav2_call_in_progress = False
        try:
            resp = future.result()
            if resp.success:
                if command == ManageLifecycleNodes.Request.STARTUP:
                    self._nav2_started = True
                    self.get_logger().info(
                        ">>> [TF GATEKEEPER] Nav2 stack successfully brought to ACTIVE state!"
                    )
                elif command == ManageLifecycleNodes.Request.PAUSE:
                    self._nav2_started = False
                    self.get_logger().info(
                        ">>> [TF GATEKEEPER] Nav2 stack successfully PAUSED."
                    )
            else:
                self.get_logger().warn(
                    f"[TF GATEKEEPER] Nav2 {command_name} command was rejected by lifecycle manager. "
                    f"Will retry in {DEFAULT_NAV2_RETRY_INTERVAL_SEC:.1f}s."
                )
                if command == ManageLifecycleNodes.Request.STARTUP:
                    self.get_logger().info(
                        "[TF GATEKEEPER] Resetting partially activated Nav2 nodes before retry..."
                    )
                    reset_req = ManageLifecycleNodes.Request()
                    reset_req.command = ManageLifecycleNodes.Request.RESET
                    self._nav2_client.call_async(reset_req)
        except Exception as exc:
            self.get_logger().error(
                f"[TF GATEKEEPER] Failed to execute Nav2 {command_name} command: {exc}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = TfReadinessGatekeeper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
