# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

import math
import time
from typing import Dict, Tuple

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


# Computes updated wheel radius based on target distance versus measured travel.
def compute_wheel_radius_calib(
    curr_wheel_radius: float, target_dist: float, measured_dist: float
) -> float:
    # Scales wheel radius linearly with actual ground truth translation distance.
    if target_dist <= 0.0:
        return curr_wheel_radius
    return curr_wheel_radius * (measured_dist / target_dist)


# Computes updated robot radius based on target rotation versus ground truth angular displacement.
def compute_robot_radius_calib(
    curr_robot_radius: float, target_rot_rad: float, measured_rot_rad: float
) -> float:
    # Scales robot radius proportionally with ground truth angular rotation.
    if abs(target_rot_rad) <= 1e-6:
        return curr_robot_radius
    return curr_robot_radius * (measured_rot_rad / target_rot_rad)


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
        self.declare_parameter("current_wheel_radius", 0.065)
        self.declare_parameter("current_robot_radius", 0.1268)
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("odom_topic", "/omni_base_controller/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_calib")
        self.declare_parameter("actual_measured_dist", 0.0)

        self._calib_mode = str(self.get_parameter("calib_mode").value)
        self._test_dist = float(self.get_parameter("test_dist").value)
        self._rot_cnt = int(self.get_parameter("rot_cnt").value)
        self._linear_vel = float(self.get_parameter("linear_vel").value)
        self._angular_vel = float(self.get_parameter("angular_vel").value)
        self._curr_wheel_radius = float(
            self.get_parameter("current_wheel_radius").value
        )
        self._curr_robot_radius = float(
            self.get_parameter("current_robot_radius").value
        )
        self._actual_measured_dist = float(
            self.get_parameter("actual_measured_dist").value
        )

        imu_topic = str(self.get_parameter("imu_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)

        self._pub_cmd_vel = self.create_publisher(TwistStamped, cmd_vel_topic, 10)
        self._sub_odom = self.create_subscription(
            Odometry, odom_topic, self._odom_cb, 10
        )
        self._sub_imu = self.create_subscription(Imu, imu_topic, self._imu_cb, 10)

        self._odom_init = False
        self._imu_init = False
        self._curr_x = 0.0
        self._curr_y = 0.0
        self._curr_odom_yaw = 0.0

        # IMU unrolled heading tracking from /imu/data
        self._imu_total_yaw = 0.0
        self._last_imu_yaw = None

        self.get_logger().info(
            f"OmniBaseCalibrator initialized in mode '{self._calib_mode}' "
            f"subscribing to odom: {odom_topic}, imu: {imu_topic}, publishing cmd_vel: {cmd_vel_topic}"
        )

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
        self._curr_x = msg.pose.pose.position.x
        self._curr_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._curr_odom_yaw = math.atan2(siny_cosp, cosy_cosp)
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

        if self._last_imu_yaw is not None:
            dyaw = curr_yaw - self._last_imu_yaw
            while dyaw > math.pi:
                dyaw -= 2.0 * math.pi
            while dyaw < -math.pi:
                dyaw += 2.0 * math.pi
            self._imu_total_yaw += dyaw
        self._last_imu_yaw = curr_yaw
        self._imu_init = True

    # Halts robot motion immediately by publishing zero twist commands.
    def stop_robot(self):
        # Sends null velocity command to ensure safe termination of motions.
        self._publish_cmd_vel(0.0, 0.0, 0.0)

    # Drives robot straight forward until target distance is accumulated by odometry.
    def drive_distance(self, target_dist: float, speed: float = 0.15) -> float:
        # Commands forward linear velocity while tracking traversed Euclidean distance.
        start_x, start_y = self._curr_x, self._curr_y
        last_log_time = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            dx = self._curr_x - start_x
            dy = self._curr_y - start_y
            traveled = math.hypot(dx, dy)
            if traveled >= target_dist:
                break
            self._publish_cmd_vel(vx=speed)

            if time.time() - last_log_time >= 1.0:
                self.get_logger().info(
                    f"Driving progress: {traveled:.2f} / {target_dist:.2f} m"
                )
                last_log_time = time.time()
            time.sleep(0.05)

        self.stop_robot()
        return math.hypot(self._curr_x - start_x, self._curr_y - start_y)

    # Rotates robot about base origin until target angle is traversed according to odometry.
    def rotate_angle_odom(self, target_angle_rad: float, speed: float = 0.4):
        # Accumulates relative angular increments handling angle boundary wrap-around.
        angular_speed = speed if target_angle_rad > 0 else -speed
        total_rot = 0.0
        last_yaw = self._curr_odom_yaw
        last_log_time = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            dyaw = self._curr_odom_yaw - last_yaw
            while dyaw > math.pi:
                dyaw -= 2.0 * math.pi
            while dyaw < -math.pi:
                dyaw += 2.0 * math.pi

            total_rot += dyaw
            last_yaw = self._curr_odom_yaw

            if abs(total_rot) >= abs(target_angle_rad):
                break
            self._publish_cmd_vel(wz=angular_speed)

            if time.time() - last_log_time >= 1.0:
                self.get_logger().info(
                    f"Spin progress: {abs(total_rot):.2f} / {abs(target_angle_rad):.2f} rad (IMU total: {self._imu_total_yaw:.2f} rad)"
                )
                last_log_time = time.time()
            time.sleep(0.05)

        self.stop_robot()

    # Executes high-precision spin test comparing odometry rotation against integrated IMU gyro ground truth.
    def run_spin_calib(self) -> float:
        # Rotates configured number of full turns to derive accurate robot_radius multiplier.
        start_imu_yaw = self._imu_total_yaw
        target_rot_rad = 2.0 * math.pi * self._rot_cnt

        self.get_logger().info(
            f"Starting In-Place Spin Test: {self._rot_cnt} full rotations ({target_rot_rad:.2f} rad) at {self._angular_vel} rad/s"
        )
        self.rotate_angle_odom(target_rot_rad, speed=self._angular_vel)
        time.sleep(0.5)

        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

        actual_imu_rot_rad = self._imu_total_yaw - start_imu_yaw
        new_robot_radius = compute_robot_radius_calib(
            self._curr_robot_radius, target_rot_rad, actual_imu_rot_rad
        )

        self.get_logger().info(
            f"\n=== SPIN TEST CALIB RESULTS ===\n"
            f"Target Odom Rotation: {target_rot_rad:.4f} rad\n"
            f"Ground Truth IMU Rotation: {actual_imu_rot_rad:.4f} rad\n"
            f"Current robot_radius: {self._curr_robot_radius:.6f} m\n"
            f"Calibrated new robot_radius: {new_robot_radius:.6f} m\n"
            f"Recommended multiplier: {actual_imu_rot_rad / target_rot_rad:.6f}\n"
            f"================================="
        )
        return new_robot_radius

    # Executes linear rollout test comparing odometry translation with ground-truth distance.
    def run_rollout_calib(self) -> float:
        # Commands straight line trajectory and recalculates wheel_radius parameter.
        self.get_logger().info(
            f"Starting Rollout Test for distance: {self._test_dist:.2f} m..."
        )
        odom_dist = self.drive_distance(self._test_dist, speed=self._linear_vel)

        measured_dist = (
            self._actual_measured_dist
            if self._actual_measured_dist > 0.0
            else self._test_dist
        )
        new_wheel_radius = compute_wheel_radius_calib(
            self._curr_wheel_radius, odom_dist, measured_dist
        )

        self.get_logger().info(
            f"\n=== ROLLOUT TEST CALIB RESULTS ===\n"
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
        # Drives complete square path in both CCW and CW directions to compute systematic errors.
        def _exec_square(clockwise: bool) -> Tuple[float, float, float]:
            sign = -1.0 if clockwise else 1.0
            start_x, start_y, start_yaw = (
                self._curr_x,
                self._curr_y,
                self._curr_odom_yaw,
            )
            for i in range(4):
                self.get_logger().info(
                    f"Edge {i + 1}/4 (CW={clockwise}): Driving {self._test_dist}m"
                )
                self.drive_distance(self._test_dist, speed=self._linear_vel)
                time.sleep(0.3)
                self.get_logger().info(f"Corner {i + 1}/4: Turning 90 deg")
                self.rotate_angle_odom(sign * (math.pi / 2.0), speed=self._angular_vel)
                time.sleep(0.3)
            return (
                self._curr_x - start_x,
                self._curr_y - start_y,
                self._curr_odom_yaw - start_yaw,
            )

        self.get_logger().info("Starting CCW Square run...")
        ccw_res = _exec_square(clockwise=False)
        time.sleep(1.0)

        self.get_logger().info("Starting CW Square run...")
        cw_res = _exec_square(clockwise=True)

        stats = compute_square_umbmark(ccw_res, cw_res, self._test_dist)
        self.get_logger().info(
            f"\n=== UMBMARK SQUARE TEST RESULTS ===\n"
            f"CCW Final Error: dx={ccw_res[0]:.4f}m, dy={ccw_res[1]:.4f}m (dist={stats['ccw_err_dist']:.4f}m)\n"
            f"CW Final Error: dx={cw_res[0]:.4f}m, dy={cw_res[1]:.4f}m (dist={stats['cw_err_dist']:.4f}m)\n"
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

    # Wait for sensor readiness
    node.get_logger().info("Waiting for odometry and IMU messages...")
    while rclpy.ok() and (not node._odom_init or not node._imu_init):
        rclpy.spin_once(node, timeout_sec=0.1)

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

    node.destroy_node()
    rclpy.shutdown()
