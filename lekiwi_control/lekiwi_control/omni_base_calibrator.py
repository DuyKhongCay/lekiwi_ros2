# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

import math
import threading
import time
from typing import Dict, Tuple

from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Imu


# Normalizes angle to the range [-pi, pi].
def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


# Computes updated wheel radius based on target distance versus measured travel.
def compute_wheel_radius_calib(
    curr_wheel_radius: float, target_dist: float, measured_dist: float
) -> float:
    # Scales wheel radius linearly with actual ground truth translation distance.
    if target_dist <= 0.0:
        return curr_wheel_radius
    return curr_wheel_radius * (measured_dist / target_dist)


def compute_robot_radius_calib(
    curr_robot_radius: float, target_rot_rad: float, measured_rot_rad: float
) -> float:
    # Scales robot radius inversely with ground truth angular rotation.
    # When robot turns physically less than target (measured < target), effective radius is larger.
    if abs(measured_rot_rad) <= 1e-6 or abs(target_rot_rad) <= 1e-6:
        return curr_robot_radius
    return curr_robot_radius * (target_rot_rad / measured_rot_rad)


# Evaluates UMBmark benchmark metrics for bidirectional square test runs.
def compute_square_umbmark(
    ccw_res: Tuple[float, float, float],
    cw_res: Tuple[float, float, float],
    nom_edge: float,
) -> Dict[str, float]:
    # Calculates systematic odometry error factors according to the UMBmark procedure.
    x_ccw, y_ccw, _ = ccw_res
    x_cw, y_cw, _ = cw_res

    alpha = (x_cw + x_ccw) / (-4.0 * nom_edge)
    beta = (x_cw - x_ccw) / (-4.0 * nom_edge)

    return {
        "alpha_rad": alpha,
        "beta_rad": beta,
        "ccw_err_dist": math.hypot(x_ccw, y_ccw),
        "cw_err_dist": math.hypot(x_cw, y_cw),
    }


# Manages automated calibration routines for LeKiwi omni-wheel base controllers.
class OmniBaseCalibratorNode(Node):
    # Initializes ROS 2 communication, parameter bindings, and calibration state storage.
    def __init__(self):
        super().__init__("omni_base_calibrator")

        self.declare_parameter("calib_mode", "spin")
        self.declare_parameter("test_dist", 1.0)
        self.declare_parameter("rot_cnt", 5)
        self.declare_parameter("linear_vel", 0.15)
        self.declare_parameter("angular_vel", 0.5)
        self.declare_parameter("controller_name", "omni_base_controller")
        self.declare_parameter("imu_topic", "/imu/data_transformed")
        self.declare_parameter("odom_topic", "/omni_base_controller/odom")
        self.declare_parameter("tag_pose_topic", "/chessboard/robot_pose")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_calib")
        self.declare_parameter("actual_measured_dist", 0.0)

        self._calib_mode = str(self.get_parameter("calib_mode").value)
        self._test_dist = float(self.get_parameter("test_dist").value)
        self._rot_cnt = int(self.get_parameter("rot_cnt").value)
        self._linear_vel = float(self.get_parameter("linear_vel").value)
        self._angular_vel = float(self.get_parameter("angular_vel").value)
        self._controller_name = str(self.get_parameter("controller_name").value)
        self._actual_measured_dist = float(
            self.get_parameter("actual_measured_dist").value
        )
        self._curr_wheel_radius = 0.0
        self._curr_robot_radius = 0.0

        imu_topic = str(self.get_parameter("imu_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        tag_pose_topic = str(self.get_parameter("tag_pose_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)

        self._pub_cmd_vel = self.create_publisher(TwistStamped, cmd_vel_topic, 10)
        self._sub_odom = self.create_subscription(
            Odometry, odom_topic, self._odom_cb, 10
        )
        self._sub_imu = self.create_subscription(Imu, imu_topic, self._imu_cb, 10)
        self._sub_tag_pose = self.create_subscription(
            PoseWithCovarianceStamped, tag_pose_topic, self._tag_pose_cb, 10
        )

        self._lock = threading.Lock()
        self._odom_init = False
        self._imu_init = False
        self._tag_pose_init = False
        self._curr_x = 0.0
        self._curr_y = 0.0
        self._curr_odom_yaw = 0.0

        # Optional visual ground truth from AprilTag /chessboard/robot_pose
        self._tag_curr_x = None
        self._tag_curr_y = None

        # IMU unrolled heading tracking from transformed IMU (/imu/data_transformed)
        self._imu_total_yaw = 0.0
        self._last_imu_yaw = None

        self.get_logger().info(
            f"OmniBaseCalibrator initialized in mode '{self._calib_mode}' "
            f"subscribing to odom: {odom_topic}, imu: {imu_topic}, publishing cmd_vel: {cmd_vel_topic}"
        )

    # Returns thread-safe copy of current 2D pose and yaw.
    def get_pose(self) -> Tuple[float, float, float]:
        with self._lock:
            return self._curr_x, self._curr_y, self._curr_odom_yaw

    # Returns thread-safe copy of continuous integrated IMU yaw.
    def get_imu_yaw(self) -> float:
        with self._lock:
            return self._imu_total_yaw

    # Checks if initial odometry and IMU readings have been received.
    def is_ready(self) -> bool:
        with self._lock:
            return self._odom_init and self._imu_init

    # Queries live kinematic parameters directly from the omni_base_controller node.
    def fetch_controller_parameters(self, timeout_sec: float = 5.0) -> bool:
        """Fetch wheel_radius and robot_radius live from the running omni_base_controller node."""
        service_name = f"/{self._controller_name}/get_parameters"
        self.get_logger().info(f"Connecting to parameter service: {service_name}...")
        client = self.create_client(GetParameters, service_name)

        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(
                f"[FAIL-SAFE] Service '{service_name}' is not available within {timeout_sec}s! "
                f"Ensure controller_manager and /{self._controller_name} are active. Aborting!"
            )
            return False

        req = GetParameters.Request()
        req.names = ["wheel_radius", "robot_radius"]

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if future.result() is None:
            self.get_logger().error(
                f"[FAIL-SAFE] Failed to receive response from '{service_name}'. Aborting!"
            )
            return False

        fetched = {}
        for name, param_val in zip(req.names, future.result().values):
            if param_val.type == ParameterType.PARAMETER_DOUBLE:
                fetched[name] = param_val.double_value

        wheel_r = fetched.get("wheel_radius", 0.0)
        robot_r = fetched.get("robot_radius", 0.0)

        if wheel_r <= 0.0 or robot_r <= 0.0:
            self.get_logger().error(
                f"[FAIL-SAFE] Invalid parameters retrieved from /{self._controller_name}: "
                f"wheel_radius={wheel_r}, robot_radius={robot_r}. Aborting!"
            )
            return False

        self._curr_wheel_radius = wheel_r
        self._curr_robot_radius = robot_r

        self.get_logger().info(
            f"Successfully auto-fetched parameters from /{self._controller_name}: "
            f"wheel_radius = {self._curr_wheel_radius:.6f} m, "
            f"robot_radius = {self._curr_robot_radius:.6f} m"
        )
        return True

    # Tracks absolute robot pose from AprilTag chessboard pose estimator
    def _tag_pose_cb(self, msg: PoseWithCovarianceStamped):
        with self._lock:
            self._tag_curr_x = msg.pose.pose.position.x
            self._tag_curr_y = msg.pose.pose.position.y
            self._tag_pose_init = True

    # Publishes timestamped velocity commands with header metadata for twist_mux.
    def _publish_cmd_vel(self, vx: float = 0.0, vy: float = 0.0, wz: float = 0.0):
        # Wraps twist velocities in TwistStamped message with current timestamp and base frame.
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.angular.z = wz
        self._pub_cmd_vel.publish(msg)

    # Updates position and yaw orientation from odometry messages.
    def _odom_cb(self, msg: Odometry):
        # Extracts 2D planar position and calculates heading angle from quaternion.
        if not self._odom_init:
            self.get_logger().info("Successfully connected to Odometry topic!")
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        with self._lock:
            self._curr_x = msg.pose.pose.position.x
            self._curr_y = msg.pose.pose.position.y
            self._curr_odom_yaw = yaw
            self._odom_init = True

    # Tracks heading from calibrated IMU orientation quaternion.
    def _imu_cb(self, msg: Imu):
        # Computes continuous unrolled yaw heading directly from filtered IMU orientation quaternion.
        if not self._imu_init:
            self.get_logger().info("Successfully connected to IMU topic!")
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        curr_yaw = math.atan2(siny_cosp, cosy_cosp)

        with self._lock:
            if self._last_imu_yaw is not None:
                dyaw = normalize_angle(curr_yaw - self._last_imu_yaw)
                self._imu_total_yaw += dyaw
            self._last_imu_yaw = curr_yaw
            self._imu_init = True

    # Halts robot motion immediately by publishing zero twist commands repeatedly for safety.
    def stop_robot(self):
        # Sends bursts of null velocity commands to ensure safe termination over lossy middleware.
        for _ in range(5):
            self._publish_cmd_vel(0.0, 0.0, 0.0)
            time.sleep(0.01)

    # Drives robot straight forward until target distance is accumulated by odometry with ramp-down.
    def drive_distance(
        self, target_dist: float, speed: float = 0.15, tol_dist: float = 0.005
    ) -> float:
        # Commands forward linear velocity while tracking traversed Euclidean distance.
        start_x, start_y, _ = self.get_pose()
        last_log_time = time.time()
        kp_lin = 1.5
        min_speed = 0.04
        decel_dist = 0.12

        while rclpy.ok():
            curr_x, curr_y, _ = self.get_pose()
            dx = curr_x - start_x
            dy = curr_y - start_y
            traveled = math.hypot(dx, dy)
            remaining = target_dist - traveled

            if remaining <= tol_dist:
                break

            if remaining < decel_dist:
                cmd_vx = max(min_speed, min(speed, kp_lin * remaining))
            else:
                cmd_vx = speed

            self._publish_cmd_vel(vx=cmd_vx)

            if time.time() - last_log_time >= 1.0:
                self.get_logger().info(
                    f"Driving progress: {traveled:.2f} / {target_dist:.2f} m"
                )
                last_log_time = time.time()
            time.sleep(0.02)

        self.stop_robot()
        curr_x, curr_y, _ = self.get_pose()
        return math.hypot(curr_x - start_x, curr_y - start_y)

    # Rotates robot to absolute target heading with proportional slowdown eliminating overshoot.
    def rotate_to_heading(
        self, target_heading_rad: float, max_speed: float = 0.4, tol_rad: float = 0.015
    ):
        last_log_time = time.time()
        kp_ang = 1.8
        min_speed = 0.06
        decel_angle = math.radians(25.0)
        start_time = time.time()
        timeout = 15.0

        while rclpy.ok() and (time.time() - start_time < timeout):
            _, _, curr_yaw = self.get_pose()
            error = normalize_angle(target_heading_rad - curr_yaw)
            if abs(error) <= tol_rad:
                break

            direction = 1.0 if error > 0 else -1.0
            if abs(error) < decel_angle:
                speed = max(min_speed, min(max_speed, kp_ang * abs(error)))
            else:
                speed = max_speed

            self._publish_cmd_vel(wz=direction * speed)

            if time.time() - last_log_time >= 1.0:
                self.get_logger().info(
                    f"Heading progress: curr={math.degrees(curr_yaw):.1f} deg, "
                    f"target={math.degrees(target_heading_rad):.1f} deg, "
                    f"err={math.degrees(error):.1f} deg"
                )
                last_log_time = time.time()
            time.sleep(0.02)

        self.stop_robot()

    # Rotates robot about base origin until target relative angle is traversed according to odometry.
    def rotate_angle_odom(
        self, target_angle_rad: float, speed: float = 0.4, tol_rad: float = 0.015
    ):
        # Accumulates relative angular increments handling angle boundary wrap-around with ramp-down.
        direction = 1.0 if target_angle_rad > 0 else -1.0
        total_rot = 0.0
        _, _, last_yaw = self.get_pose()
        last_log_time = time.time()
        kp_ang = 1.8
        min_speed = 0.06
        decel_angle = math.radians(25.0)
        target_abs = abs(target_angle_rad)

        while rclpy.ok():
            _, _, curr_yaw = self.get_pose()
            dyaw = normalize_angle(curr_yaw - last_yaw)
            total_rot += dyaw
            last_yaw = curr_yaw

            remaining = target_abs - abs(total_rot)
            if remaining <= tol_rad:
                break

            if remaining < decel_angle:
                cmd_wz = direction * max(min_speed, min(speed, kp_ang * remaining))
            else:
                cmd_wz = direction * speed

            self._publish_cmd_vel(wz=cmd_wz)

            if time.time() - last_log_time >= 1.0:
                self.get_logger().info(
                    f"Spin progress: {abs(total_rot):.2f} / {target_abs:.2f} rad (IMU total: {self.get_imu_yaw():.2f} rad)"
                )
                last_log_time = time.time()
            time.sleep(0.02)

        self.stop_robot()

    # Executes high-precision spin test comparing odometry rotation against integrated IMU gyro ground truth.
    def run_spin_calib(self) -> float:
        # Rotates configured number of full turns to derive accurate robot_radius multiplier.
        start_imu_yaw = self.get_imu_yaw()
        target_rot_rad = 2.0 * math.pi * self._rot_cnt

        self.get_logger().info(
            f"Starting In-Place Spin Test: {self._rot_cnt} full rotations ({target_rot_rad:.2f} rad) at {self._angular_vel} rad/s"
        )
        self.rotate_angle_odom(target_rot_rad, speed=self._angular_vel)
        time.sleep(0.5)

        actual_imu_rot_rad = self.get_imu_yaw() - start_imu_yaw
        new_robot_radius = compute_robot_radius_calib(
            self._curr_robot_radius, target_rot_rad, actual_imu_rot_rad
        )

        self.get_logger().info(
            f"\n=== SPIN TEST CALIB RESULTS ===\n"
            f"Target Odom Rotation: {target_rot_rad:.4f} rad\n"
            f"Ground Truth IMU Rotation: {actual_imu_rot_rad:.4f} rad\n"
            f"Current robot_radius: {self._curr_robot_radius:.6f} m\n"
            f"Calibrated new robot_radius: {new_robot_radius:.6f} m\n"
            f"Recommended multiplier: {target_rot_rad / actual_imu_rot_rad:.6f}\n"
            f"================================="
        )
        return new_robot_radius

    # Executes linear rollout test comparing odometry translation with ground-truth distance.
    def run_rollout_calib(self) -> float:
        # Commands straight line trajectory and recalculates wheel_radius parameter.
        self.get_logger().info(
            f"Starting Rollout Test for distance: {self._test_dist:.2f} m..."
        )
        # Check initial AprilTag visual pose if available
        with self._lock:
            start_tag_x, start_tag_y = self._tag_curr_x, self._tag_curr_y

        odom_dist = self.drive_distance(self._test_dist, speed=self._linear_vel)
        time.sleep(0.5)

        tag_measured_dist = 0.0
        with self._lock:
            curr_tag_x, curr_tag_y = self._tag_curr_x, self._tag_curr_y

        if (
            start_tag_x is not None
            and start_tag_y is not None
            and curr_tag_x is not None
            and curr_tag_y is not None
        ):
            tag_measured_dist = math.hypot(
                curr_tag_x - start_tag_x, curr_tag_y - start_tag_y
            )
            self.get_logger().info(
                f"AprilTag visual ground truth measured travel: {tag_measured_dist:.4f} m"
            )

        if self._actual_measured_dist > 0.0:
            measured_dist = self._actual_measured_dist
            gt_source = "User Tape Measurement"
        elif tag_measured_dist > 0.05:
            measured_dist = tag_measured_dist
            gt_source = "AprilTag Visual Ground Truth (/chessboard/robot_pose)"
        else:
            measured_dist = self._test_dist
            gt_source = "Target Distance (Nominal)"

        new_wheel_radius = compute_wheel_radius_calib(
            self._curr_wheel_radius, odom_dist, measured_dist
        )

        self.get_logger().info(
            f"\n=== ROLLOUT TEST CALIB RESULTS ===\n"
            f"Ground Truth Source: {gt_source}\n"
            f"Commanded Distance: {self._test_dist:.4f} m\n"
            f"Odom Traversed Distance: {odom_dist:.4f} m\n"
            f"Ground Truth Measured Distance: {measured_dist:.4f} m\n"
            f"Current wheel_radius: {self._curr_wheel_radius:.6f} m\n"
            f"Calibrated new wheel_radius: {new_wheel_radius:.6f} m\n"
            f"Recommended multiplier: {measured_dist / odom_dist:.6f}\n"
            f"==================================="
        )
        return new_wheel_radius

    # Executes standard bidirectional UMBmark square path test for combined geometric verification.
    def run_square_calib(self) -> Dict[str, float]:
        # Drives complete square path in both CCW and CW directions with absolute heading tracking.
        def _exec_square(clockwise: bool) -> Tuple[float, float, float]:
            sign = -1.0 if clockwise else 1.0
            start_x, start_y, start_yaw = self.get_pose()

            for i in range(4):
                self.get_logger().info(
                    f"Edge {i + 1}/4 (CW={clockwise}): Driving {self._test_dist}m"
                )
                self.drive_distance(self._test_dist, speed=self._linear_vel)
                time.sleep(0.3)

                # Target absolute heading for each corner: start_yaw + sign * (i + 1) * (pi / 2)
                corner_target_heading = normalize_angle(
                    start_yaw + sign * (i + 1) * (math.pi / 2.0)
                )
                self.get_logger().info(
                    f"Corner {i + 1}/4: Turning to target heading {math.degrees(corner_target_heading):.1f} deg"
                )
                self.rotate_to_heading(
                    corner_target_heading, max_speed=self._angular_vel
                )
                time.sleep(0.3)

            end_x, end_y, end_yaw = self.get_pose()
            final_yaw_err = normalize_angle(end_yaw - start_yaw)
            return (
                end_x - start_x,
                end_y - start_y,
                final_yaw_err,
            )

        self.get_logger().info("Starting CCW Square run...")
        ccw_res = _exec_square(clockwise=False)
        time.sleep(1.0)

        self.get_logger().info("Starting CW Square run...")
        cw_res = _exec_square(clockwise=True)

        stats = compute_square_umbmark(ccw_res, cw_res, self._test_dist)
        self.get_logger().info(
            f"\n=== UMBMARK SQUARE TEST RESULTS ===\n"
            f"CCW Final Error: dx={ccw_res[0]:.4f}m, dy={ccw_res[1]:.4f}m (dist={stats['ccw_err_dist']:.4f}m, dyaw={math.degrees(ccw_res[2]):.2f} deg)\n"
            f"CW Final Error: dx={cw_res[0]:.4f}m, dy={cw_res[1]:.4f}m (dist={stats['cw_err_dist']:.4f}m, dyaw={math.degrees(cw_res[2]):.2f} deg)\n"
            f"Type A (Linear) Error Factor: {stats['alpha_rad']:.6f} rad\n"
            f"Type B (Angular) Error Factor: {stats['beta_rad']:.6f} rad\n"
            f"==================================="
        )
        return stats


# Entry point for the omni-wheel base calibration executable.
def main(args=None):
    # Initializes ROS context, executes the chosen calibration routine, and shuts down safely.
    rclpy.init(args=args)
    node = OmniBaseCalibratorNode()

    # Fail-safe parameter retrieval directly from running omni_base_controller
    if not node.fetch_controller_parameters(timeout_sec=5.0):
        node.get_logger().error(
            "Calibration routine aborted immediately due to parameter retrieval failure."
        )
        node.destroy_node()
        rclpy.shutdown()
        return

    # Start multi-threaded executor in background daemon thread for realtime callback processing
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Wait for sensor readiness
    node.get_logger().info("Waiting for odometry and IMU messages...")
    while rclpy.ok() and not node.is_ready():
        time.sleep(0.05)

    try:
        if node._calib_mode == "spin":
            node.run_spin_calib()
        elif node._calib_mode == "rollout":
            node.run_rollout_calib()
        elif node._calib_mode == "square":
            node.run_square_calib()
        else:
            node.get_logger().error(
                f"Unknown calib_mode: '{node._calib_mode}'. Use spin, rollout, or square."
            )
    finally:
        node.stop_robot()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join(timeout=1.0)
