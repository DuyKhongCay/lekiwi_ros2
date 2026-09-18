"""Chessboard AprilTag Calibrator Node for LeKiwi."""

import math
import os
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

try:
    from apriltag_msgs.msg import AprilTagDetectionArray
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image
except ImportError:
    AprilTagDetectionArray = None
    CameraInfo = None
    Image = None
    HistoryPolicy = None
    QoSProfile = None
    ReliabilityPolicy = None
    rclpy = None
    Node = object

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

from lekiwi_calibration.chessboard.solver import (
    ChessboardTagCalibSolver,
    parse_camera_info,
    save_to_chessboard_yaml,
)
from lekiwi_calibration.chessboard.visualizer import (
    CalibratorVisualizer,
    CalibState,
    Notification,
)


class ChessboardTagCalibratorNode(Node):
    """ROS 2 Node coordinating multi-view capture and non-blocking calibration."""

    def __init__(self):
        """Initializes subscriptions, visualizer, thread locks, and timers."""
        super().__init__("chessboard_tag_calibrator")

        # 1. Parameter declarations
        defaults = {
            "img_topic": "/cameras/stereo_left/image_raw",
            "cam_info_topic": "/cameras/stereo_left/camera_info",
            "tag_dets_topic": "/tag_detections",
            "cam_info_path": "package://lekiwi_bringup/config/perception/camera_info/stereo_left.yaml",
            "output_yaml_path": "package://lekiwi_bringup/config/calibration/chessboard_tags.yaml",
            "tag_ids": [0, 1, 2, 3],
            "tag_names": ["A1", "H1", "H8", "A8"],
            "tag_sz": 0.022,
            "nominal_dist": 0.39,
            "z_height": 0.004,
            "min_tags_cnt": 2,
            "target_caps_cnt": 50,
            "auto_cap_interval_sec": 1.0,
            "window_name": "LeKiwi Chessboard Tag Calibrator",
            "disp_scale": 1.0,
            "headless": False,
        }
        for name, val in defaults.items():
            self.declare_parameter(name, val)
            setattr(self, name, self.get_parameter(name).value)

        # 2. Camera matrix fallback from file
        try:
            self.cam_mat, self.dist_coeffs = parse_camera_info(self.cam_info_path)
            self.get_logger().info(
                f"Loaded camera matrix fallback from: {self.cam_info_path}"
            )
        except Exception as e:
            self.get_logger().warn(
                f"Camera info fallback file not loaded ({e}), waiting for live topic."
            )
            self.cam_mat, self.dist_coeffs = None, None

        self.cam_info_received = False
        self.bridge = CvBridge() if CvBridge is not None else None

        # 3. Concurrency Lock & Shared State
        self._data_lock = threading.Lock()
        self.latest_img: Optional[np.ndarray] = None
        self.latest_dets: Dict[int, np.ndarray] = {}
        self.captured_frames: List[Dict[int, np.ndarray]] = []
        self.auto_cap_enabled: bool = False
        self.last_auto_cap_time: float = 0.0
        self.calib_res: Optional[Dict[str, Any]] = None
        self.calib_state: CalibState = CalibState.IDLE
        self.notification: Notification = Notification()

        # 4. Pure Modules
        self.solver = ChessboardTagCalibSolver(
            tag_ids=self.tag_ids,
            tag_sz=self.tag_sz,
            nominal_dist=self.nominal_dist,
            z_height=self.z_height,
        )
        self.visualizer = CalibratorVisualizer(
            window_name=self.window_name,
            disp_scale=self.disp_scale,
            tag_ids=self.tag_ids,
            tag_names=self.tag_names,
            target_caps_cnt=self.target_caps_cnt,
            min_tags_cnt=self.min_tags_cnt,
        )

        # 5. ROS 2 Subscriptions
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.img_sub = self.create_subscription(
            Image, self.img_topic, self._img_cb, qos
        )
        self.cam_info_sub = self.create_subscription(
            CameraInfo, self.cam_info_topic, self._cam_info_cb, qos
        )
        self.tag_dets_sub = self.create_subscription(
            AprilTagDetectionArray, self.tag_dets_topic, self._tag_dets_cb, qos
        )

        # 6. Main Loop Timer (~30 Hz)
        self.gui_timer = self.create_timer(0.033, self._gui_timer_cb)
        self.get_logger().info(
            f"Calibrator initialized (Headless: {self.headless}). "
            f"Topics: {self.img_topic}, {self.tag_dets_topic}"
        )

    def _set_notification(
        self, msg: str, color: Tuple[int, int, int] = (0, 255, 0), duration: float = 1.0
    ) -> None:
        """Helper to post thread-safe temporary notification messages."""
        with self._data_lock:
            self.notification = Notification(
                message=msg,
                color=color,
                timestamp=time.time(),
                duration=duration,
            )

    def _cam_info_cb(self, msg: CameraInfo) -> None:
        """Callback to update camera matrix and distortion from live ROS topic."""
        with self._data_lock:
            self.cam_mat = np.array(msg.k, dtype=np.float64).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape((-1, 1))
            if not self.cam_info_received:
                self.cam_info_received = True
                self.get_logger().info(
                    f"Received live CameraInfo from {self.cam_info_topic}: "
                    f"fx={self.cam_mat[0, 0]:.1f}, fy={self.cam_mat[1, 1]:.1f} "
                    f"({msg.width}x{msg.height})"
                )

    def _img_cb(self, msg: Image) -> None:
        """Callback to convert and update latest camera frame."""
        try:
            if self.bridge is not None:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            else:
                raw_data = np.frombuffer(msg.data, dtype=np.uint8)
                if msg.encoding in ("rgb8", "bgr8"):
                    frame = raw_data.reshape((msg.height, msg.width, 3))
                    cv_img = (
                        cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                        if msg.encoding == "rgb8"
                        else frame
                    )
                else:
                    return

            with self._data_lock:
                self.latest_img = cv_img
        except Exception as err:
            self.get_logger().warn(
                f"Image conversion failed: {err}", throttle_duration_sec=5.0
            )

    def _tag_dets_cb(self, msg: AprilTagDetectionArray) -> None:
        """Callback to parse incoming AprilTag 2D detections and trigger auto-cap."""
        tag_map = {}
        for det in msg.detections:
            tag_map[det.id] = np.array(
                [[pt.x, pt.y] for pt in det.corners], dtype=np.float64
            )

        trigger_cap = False
        with self._data_lock:
            self.latest_dets = tag_map
            if self.auto_cap_enabled:
                now = time.time()
                if now - self.last_auto_cap_time >= self.auto_cap_interval_sec:
                    if len(self.captured_frames) >= self.target_caps_cnt:
                        self.auto_cap_enabled = False
                        self._set_notification(
                            f"Auto-cap done: {self.target_caps_cnt} frames",
                            (0, 255, 255),
                        )
                    elif (
                        sum(1 for tid in self.tag_ids if tid in tag_map)
                        >= self.min_tags_cnt
                    ):
                        trigger_cap = True
                        self.last_auto_cap_time = now

        if trigger_cap:
            self.capture_current_frame()

    def capture_current_frame(self) -> bool:
        """Captures valid chessboard tag detections from the current frame."""
        with self._data_lock:
            valid = {
                tid: pts.copy()
                for tid, pts in self.latest_dets.items()
                if tid in self.tag_ids
            }
            if len(valid) >= self.min_tags_cnt:
                self.captured_frames.append(valid)
                cnt = len(self.captured_frames)
                self.get_logger().info(f"[CAPTURE] Frame #{cnt} ({len(valid)} tags)")
                self._set_notification(
                    f"Captured #{cnt} ({len(valid)} tags)", (0, 255, 0)
                )
                return True

            needed = self.min_tags_cnt
            have = len(valid)

        self._set_notification(
            f"Failed: need >={needed} tags (have {have})", (0, 100, 255)
        )
        return False

    def run_calibration_async(self) -> None:
        """Spawns non-blocking worker thread to run Bundle Adjustment solver."""
        with self._data_lock:
            if self.calib_state == CalibState.OPTIMIZING:
                self.get_logger().warn("Optimization already in progress.")
                return

            if len(self.captured_frames) < 3:
                self._set_notification("Need >= 3 captured frames", (0, 100, 255))
                self.get_logger().error(
                    f"Need >= 3 frames, have {len(self.captured_frames)}"
                )
                return

            if self.cam_mat is None or self.dist_coeffs is None:
                self._set_notification("Camera intrinsics missing", (0, 100, 255))
                self.get_logger().error(
                    "Camera matrix or distortion coefficients missing."
                )
                return

            frames_snapshot = [f.copy() for f in self.captured_frames]
            cam_mat_snapshot = self.cam_mat.copy()
            dist_snapshot = self.dist_coeffs.copy()
            self.calib_state = CalibState.OPTIMIZING

        self._set_notification(
            "Optimizing Bundle Adjustment...", (0, 255, 255), duration=5.0
        )
        self.get_logger().info(
            f"Solving Planar Bundle Adjustment on {len(frames_snapshot)} frames in background..."
        )

        worker = threading.Thread(
            target=self._solve_worker,
            args=(frames_snapshot, cam_mat_snapshot, dist_snapshot),
            daemon=True,
        )
        worker.start()

    def _solve_worker(
        self,
        frames: List[Dict[int, np.ndarray]],
        cam_mat: np.ndarray,
        dist_coeffs: np.ndarray,
    ) -> None:
        """Worker thread executing Bundle Adjustment without blocking ROS or GUI."""
        try:
            res = self.solver.solve(frames, cam_mat, dist_coeffs)
            with self._data_lock:
                self.calib_res = res
                self.calib_state = CalibState.OPTIMIZED
            self._print_calib_report(res)
            self._set_notification(
                f"Solved! RMS Error: {res['rms_err']:.3f}px", (0, 255, 0), duration=3.0
            )
        except Exception as err:
            with self._data_lock:
                self.calib_state = CalibState.ERROR
            self.get_logger().error(f"Optimization failed: {err}")
            self._set_notification(
                f"Optimization error: {err}", (0, 0, 255), duration=3.0
            )

    def _print_calib_report(self, res: Dict[str, Any]) -> None:
        """Prints formatted terminal report of calibration results."""
        print("\n" + "=" * 64)
        print("          CALIBRATION RESULTS REPORT (PLANAR)")
        print("=" * 64)
        print(
            f"Frames: {res['num_frames']} | "
            f"Mean Error: {res['mean_err']:.4f}px | "
            f"RMS Error: {res['rms_err']:.4f}px\n"
        )
        tags = res["tags"]
        print("ID | Name |    X (m)   |    Y (m)   |    Z (m)   |  Yaw (deg)")
        print("-" * 64)
        for tid in self.tag_ids:
            if tid in tags:
                t = tags[tid]
                print(
                    f"{tid:2d} | {t['name']:4s} | "
                    f"{t['x']:10.4f} | {t['y']:10.4f} | {t['z']:10.4f} | "
                    f"{math.degrees(t['yaw']):9.2f}"
                )

        coords = {
            tid: np.array([tags[tid]["x"], tags[tid]["y"], tags[tid]["z"]])
            for tid in self.tag_ids
            if tid in tags
        }
        edges = [
            ("A1->H1", self.tag_ids[0], self.tag_ids[1]),
            ("H1->H8", self.tag_ids[1], self.tag_ids[2]),
            ("H8->A8", self.tag_ids[2], self.tag_ids[3]),
            ("A8->A1", self.tag_ids[3], self.tag_ids[0]),
        ]
        print("\nCorner Distances:")
        for label, u, v in edges:
            if u in coords and v in coords:
                dist = np.linalg.norm(coords[u] - coords[v])
                print(f"  - {label:6s}: {dist * 1000.0:6.2f} mm ({dist:.4f} m)")
        print("=" * 64 + "\n")

    def save_calibration(self) -> None:
        """Saves active calibration parameters to the target YAML file."""
        with self._data_lock:
            if self.calib_res is None:
                self._set_notification("Calibrate before saving", (0, 100, 255))
                self.get_logger().warn("Run calibration before saving.")
                return
            res_snapshot = self.calib_res

        try:
            save_to_chessboard_yaml(
                res_snapshot, self.output_yaml_path, tag_ids=self.tag_ids
            )
            self._set_notification("Saved calibration to YAML", (0, 255, 0))
            self.get_logger().info(f"Saved calibration to {self.output_yaml_path}")
        except Exception as err:
            self._set_notification(f"Save failed: {err}", (0, 0, 255))
            self.get_logger().error(f"Failed to save YAML: {err}")

    def _gui_timer_cb(self) -> None:
        """Main loop timer (~30 Hz) handling visualization and user key inputs."""
        with self._data_lock:
            img = self.latest_img.copy() if self.latest_img is not None else None
            dets = {k: v.copy() for k, v in self.latest_dets.items()}
            captured_cnt = len(self.captured_frames)
            is_auto = self.auto_cap_enabled
            state = self.calib_state
            rms = self.calib_res["rms_err"] if self.calib_res is not None else None
            notif = Notification(
                message=self.notification.message,
                color=self.notification.color,
                timestamp=self.notification.timestamp,
                duration=self.notification.duration,
            )

        # In headless mode, skip OpenCV GUI calls entirely
        if self.headless:
            return

        canvas = self.visualizer.draw(
            base_img=img,
            detections=dets,
            captured_cnt=captured_cnt,
            is_auto=is_auto,
            calib_state=state,
            rms_err=rms,
            notification=notif,
        )

        key = self.visualizer.show(canvas)
        if key == 32:  # SPACE
            self.capture_current_frame()
        elif key in (ord("c"), ord("C")):
            self.run_calibration_async()
        elif key in (ord("s"), ord("S")):
            self.save_calibration()
        elif key in (ord("r"), ord("R")):
            with self._data_lock:
                self.captured_frames.clear()
                self.calib_res = None
                self.calib_state = CalibState.IDLE
            self._set_notification("Samples reset", (0, 200, 255))
            self.get_logger().info("Reset all captured frames.")
        elif key in (ord("a"), ord("A")):
            with self._data_lock:
                self.auto_cap_enabled = not self.auto_cap_enabled
                enabled = self.auto_cap_enabled
            self._set_notification(
                f"Auto-cap: {'ON' if enabled else 'OFF'}",
                (0, 255, 255) if enabled else (180, 180, 180),
            )
            self.get_logger().info(f"Auto-capture toggled: {enabled}")
        elif key in (ord("q"), ord("Q"), 27):  # Q or ESC
            cv2.destroyAllWindows()
            rclpy.shutdown()


def main(args: Optional[List[str]] = None) -> None:
    """Initializes ROS 2 context, instantiates calibrator node, and spins."""
    rclpy.init(args=args)
    node = ChessboardTagCalibratorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
