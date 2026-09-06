#!/usr/bin/env python3
# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Native OpenCV GUI Hand-Eye Calibration Node for LeKiwi Robot."""

import os
import sys
import threading
import time
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time, Duration
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import tf2_ros
from tf2_ros import TransformBroadcaster, Buffer, TransformListener

from handeye_calibration.charuco_detector import CharucoDetectorHelper
from handeye_calibration.robot_controller_client import RobotArmManager
from handeye_calibration.handeye_solver import HandEyeSolver


class HandEyeCalibrationNode(Node):
    def __init__(self):
        super().__init__("handeye_calibration_node")

        # ── Parameter Declarations ──
        self.declare_parameter("image_topic", "/cameras/stereo_left/image_raw")
        self.declare_parameter("camera_info_topic", "/cameras/stereo_left/camera_info")
        self.declare_parameter("camera_frame", "")  # Auto from msg if empty
        self.declare_parameter("target_frame", "handeye_target")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("robot_effector_frame", "gripperframe")
        self.declare_parameter("is_eye_in_hand", False)

        # ChArUco specs
        self.declare_parameter("squares_x", 3)
        self.declare_parameter("squares_y", 4)
        self.declare_parameter("square_length_m", 0.006)
        self.declare_parameter("marker_length_m", 0.0045)
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("min_markers", 1)
        self.declare_parameter("max_reproj_px", 3.0)

        # Controller / Torque management
        self.declare_parameter("auto_disable_arm_torque", True)
        self.declare_parameter(
            "switch_controller_service", "/controller_manager/switch_controller"
        )
        self.declare_parameter("set_torque_service", "/set_torque_enabled")
        self.declare_parameter(
            "output_yaml_path", "~/.ros/lekiwi_handeye_calibration.yaml"
        )

        # Retrieve parameters
        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self._camera_frame = self.get_parameter("camera_frame").value
        self._target_frame = self.get_parameter("target_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._effector_frame = self.get_parameter("robot_effector_frame").value
        self.is_eye_in_hand = bool(self.get_parameter("is_eye_in_hand").value)
        self.max_reproj_px = float(self.get_parameter("max_reproj_px").value)
        self.output_yaml = os.path.expanduser(
            self.get_parameter("output_yaml_path").value
        )

        # Components
        self.detector = CharucoDetectorHelper(
            squares_x=int(self.get_parameter("squares_x").value),
            squares_y=int(self.get_parameter("squares_y").value),
            square_len_m=float(self.get_parameter("square_length_m").value),
            marker_len_m=float(self.get_parameter("marker_length_m").value),
            dict_name=str(self.get_parameter("dictionary").value),
            min_markers=int(self.get_parameter("min_markers").value),
        )
        self.cbg = rclpy.callback_groups.ReentrantCallbackGroup()

        self.solver = HandEyeSolver(is_eye_in_hand=self.is_eye_in_hand)
        self.arm_manager = RobotArmManager(
            self,
            switch_controller_service=str(
                self.get_parameter("switch_controller_service").value
            ),
            set_torque_service=str(self.get_parameter("set_torque_service").value),
            callback_group=self.cbg,
        )

        # State variables
        self.bridge = CvBridge()
        self.K = None
        self.D = None
        self.latest_bgr = None
        self.latest_stamp = None
        self.last_rvec = None
        self.last_tvec = None
        self.last_reproj_err = None
        self.last_computed_result = None
        self.status_msg = "Ready. Press [T] to toggle torque, [SPACE] to take sample."

        # TF Broadcaster & Listener
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5), node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # Subscriptions (Match RELIABLE or BEST_EFFORT)
        qos_img = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.cbg = rclpy.callback_groups.ReentrantCallbackGroup()
        self.create_subscription(
            Image, self.image_topic, self._on_image, qos_img, callback_group=self.cbg
        )
        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._on_camera_info,
            10,
            callback_group=self.cbg,
        )

        self.running = True

        self.get_logger().info(
            f"HandEye Calibration Node initialized:\n"
            f"  Image topic: {self.image_topic}\n"
            f"  Info topic:  {self.camera_info_topic}\n"
            f"  Base frame:  {self._base_frame}\n"
            f"  Effector:    {self._effector_frame}\n"
            f"  Eye-in-Hand: {self.is_eye_in_hand}"
        )

        # Auto-disable arm torque on startup if configured
        if self.get_parameter("auto_disable_arm_torque").value:
            threading.Timer(
                1.0, self.arm_manager.disable_arm_for_manual_leadthrough
            ).start()

    def _on_camera_info(self, msg: CameraInfo):
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape((3, 3))
            self.D = np.array(msg.d, dtype=np.float64)
            if not self._camera_frame:
                self._camera_frame = msg.header.frame_id or "camera_optical_frame"
            self.get_logger().info(
                f"Received CameraInfo: frame='{self._camera_frame}', K shape={self.K.shape}"
            )

    def _on_image(self, msg: Image):
        try:
            self.latest_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_stamp = msg.header.stamp
            if not self._camera_frame and msg.header.frame_id:
                self._camera_frame = msg.header.frame_id
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")

    def _publish_target_tf(self, rvec, tvec, stamp):
        if not self._camera_frame or rvec is None or tvec is None:
            return

        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = tvec.flatten()

        rot = Rotation.from_matrix(R)
        quat = rot.as_quat()  # [x, y, z, w]

        tfs = TransformStamped()
        tfs.header.stamp = stamp or self.get_clock().now().to_msg()
        tfs.header.frame_id = self._camera_frame
        tfs.child_frame_id = self._target_frame

        tfs.transform.translation.x = float(tvec[0, 0] if tvec.ndim > 1 else tvec[0])
        tfs.transform.translation.y = float(tvec[1, 0] if tvec.ndim > 1 else tvec[1])
        tfs.transform.translation.z = float(tvec[2, 0] if tvec.ndim > 1 else tvec[2])

        tfs.transform.rotation.x = float(quat[0])
        tfs.transform.rotation.y = float(quat[1])
        tfs.transform.rotation.z = float(quat[2])
        tfs.transform.rotation.w = float(quat[3])

        self.tf_broadcaster.sendTransform(tfs)

    def _take_sample(self):
        """Captures pair of robot pose and tracking pose from TF."""
        if not self._camera_frame:
            self.status_msg = "⚠️ No camera frame received yet!"
            return

        if self.last_rvec is None or self.last_tvec is None:
            self.status_msg = "⚠️ Cannot sample: ChArUco target not detected!"
            return

        try:
            t = Time()
            if self.is_eye_in_hand:
                # Eye-in-Hand: robot = base -> effector
                robot_tf = self.tf_buffer.lookup_transform(
                    self._base_frame, self._effector_frame, t, Duration(seconds=1.0)
                )
            else:
                # Eye-to-Hand: robot = effector -> base
                robot_tf = self.tf_buffer.lookup_transform(
                    self._effector_frame, self._base_frame, t, Duration(seconds=1.0)
                )

            # Tracking = camera -> target
            tracking_tf = self.tf_buffer.lookup_transform(
                self._camera_frame, self._target_frame, t, Duration(seconds=1.0)
            )

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            self.status_msg = f"❌ TF Lookup failed: {e}"
            self.get_logger().error(self.status_msg)
            return

        # Convert to 4x4 matrices
        def tf_to_matrix(tf_msg):
            trans = tf_msg.translation
            rot = tf_msg.rotation
            T = np.eye(4)
            T[:3, :3] = Rotation.from_quat([rot.x, rot.y, rot.z, rot.w]).as_matrix()
            T[:3, 3] = [trans.x, trans.y, trans.z]
            return T

        robot_T = tf_to_matrix(robot_tf.transform)
        tracking_T = tf_to_matrix(tracking_tf.transform)

        count = self.solver.add_sample(robot_T, tracking_T)
        self.status_msg = f"✅ Sample #{count} taken! (Total: {count})"
        self.get_logger().info(f"[HandEye] Sample #{count} added successfully.")

    def _compute_and_report(self):
        try:
            results, best_name, metrics = self.solver.compute()
            self.last_computed_result = (results, best_name, metrics)
            t = metrics["translation"]
            rpy = metrics["rotation_rpy_deg"]
            err = metrics["residual_error"]
            self.status_msg = (
                f"🎉 Calib Success [{best_name}]! err={err:.4f}m | "
                f"t=[{t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}] | "
                f"rpy=[{rpy[0]:.1f}, {rpy[1]:.1f}, {rpy[2]:.1f}] deg"
            )
            self.get_logger().info("=== Calibration Results Across Solvers ===")
            for name, T in results.items():
                r = Rotation.from_matrix(T[:3, :3]).as_euler("xyz", degrees=True)
                self.get_logger().info(
                    f"  {name:12s}: t=({T[0,3]:+.4f}, {T[1,3]:+.4f}, {T[2,3]:+.4f}) rpy=({r[0]:+.1f}, {r[1]:+.1f}, {r[2]:+.1f})°"
                )

        except Exception as e:
            self.status_msg = f"❌ Compute failed: {e}"
            self.get_logger().error(self.status_msg)

    def _save_result(self):
        if not self.last_computed_result:
            self.status_msg = "⚠️ Please press [C] to compute calibration first!"
            return

        results, best_name, metrics = self.last_computed_result
        best_T = results[best_name]

        if self.is_eye_in_hand:
            parent_frame = self._effector_frame
            child_frame = self._camera_frame
        else:
            parent_frame = self._base_frame
            child_frame = self._camera_frame

        self.solver.save_yaml(
            self.output_yaml, parent_frame, child_frame, best_T, metrics
        )
        self.status_msg = f"💾 Saved to {self.output_yaml}"
        self.get_logger().info(f"Saved calibration YAML to: {self.output_yaml}")

    def _gui_tick(self):
        if self.latest_bgr is None:
            # Render waiting screen
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(
                frame,
                "Waiting for camera stream...",
                (50, 240),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 200, 255),
                2,
            )
            cv2.putText(
                frame,
                f"Topic: {self.image_topic}",
                (50, 280),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (180, 180, 180),
                1,
            )
        elif self.K is None:
            # We have image, but no CameraInfo yet
            frame = self.latest_bgr.copy()
            cv2.putText(
                frame,
                "Warning: Waiting for CameraInfo topic...",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )
            cv2.putText(
                frame,
                f"Topic: {self.camera_info_topic}",
                (20, 75),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (200, 200, 200),
                1,
            )
        else:
            frame, rvec, tvec, reproj_err, num_corners = (
                self.detector.detect_and_estimate_pose(
                    self.latest_bgr, self.K, self.D, self.max_reproj_px
                )
            )
            self.last_rvec = rvec
            self.last_tvec = tvec
            self.last_reproj_err = reproj_err

            if rvec is not None and tvec is not None:
                self._publish_target_tf(rvec, tvec, self.latest_stamp)

        # Scale camera frame to fit 640x380 so entire window is exactly 640x480
        frame_resized = cv2.resize(frame, (640, 380), interpolation=cv2.INTER_AREA)
        banner = np.zeros((100, 640, 3), dtype=np.uint8)

        # Header info
        samples_cnt = len(self.solver.samples)
        torque_str = "FREE" if not self.arm_manager.torque_is_enabled else "LOCKED"
        torque_col = (
            (0, 255, 0) if not self.arm_manager.torque_is_enabled else (0, 0, 255)
        )

        cv2.putText(
            banner,
            f"Samples: {samples_cnt}",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
        )
        cv2.putText(
            banner,
            f"Torque: {torque_str}",
            (140, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            torque_col,
            1,
        )

        if self.last_reproj_err is not None:
            cv2.putText(
                banner,
                f"Reproj: {self.last_reproj_err:.2f} px",
                (420, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                1,
            )

        # Status text
        cv2.putText(
            banner,
            f">> {self.status_msg}",
            (10, 55),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 200, 0),
            1,
        )

        # Instructions / Hotkeys
        hotkeys = "[SPACE] Sample | [C] Compute | [S] Save | [T] Torque | [R] Reset | [Q] Quit"
        cv2.putText(
            banner,
            hotkeys,
            (10, 85),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (180, 180, 180),
            1,
        )

        display = np.vstack([frame_resized, banner])
        win_name = "LeKiwi Hand-Eye Calibration"
        cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win_name, 640, 480)
        cv2.imshow(win_name, display)

        key = cv2.waitKey(20) & 0xFF
        if key == 32:  # SPACE
            self._take_sample()
        elif key in (ord("c"), ord("C")):
            self._compute_and_report()
        elif key in (ord("s"), ord("S")):
            self._save_result()
        elif key in (ord("t"), ord("T")):
            if self.arm_manager.torque_is_enabled:
                self.arm_manager.disable_arm_for_manual_leadthrough()
                self.status_msg = "Arm torque disabled (lead-through ready)."
            else:
                self.arm_manager.enable_arm_torque()
                self.status_msg = "Arm torque enabled."
        elif key in (ord("r"), ord("R")):
            self.solver.clear_samples()
            self.last_computed_result = None
            self.status_msg = "Samples cleared."
        elif key in (ord("q"), ord("Q"), 27):  # Q or ESC
            self.running = False


def main(args=None):
    rclpy.init(args=args)
    node = HandEyeCalibrationNode()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    # Spin ROS 2 in a background daemon thread
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Run OpenCV GUI loop in MAIN THREAD
    try:
        while node.running and rclpy.ok():
            node._gui_tick()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
