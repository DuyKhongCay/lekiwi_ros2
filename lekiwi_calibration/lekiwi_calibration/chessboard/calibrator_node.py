import json
import math
import os
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

from apriltag_msgs.msg import AprilTagDetectionArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from lekiwi_calibration.chessboard.solver import (
    ChessboardTagCalibSolver,
    format_calibration_report,
    parse_camera_info,
    resolve_path,
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
            "output_yaml_path": "package://lekiwi_calibration/config/calib_result.yaml",
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
            "enable_dataset_dump": True,
            "dataset_dump_dir": "package://lekiwi_calibration/data/chessboard_dataset",
            "save_overlay_image": True,
        }
        for name, val in defaults.items():
            self.declare_parameter(name, val)
            setattr(self, name, self.get_parameter(name).value)

        # 2. Concurrency Lock & Shared State
        self._data_lock = threading.RLock()
        self._solve_epoch: int = 0
        self.latest_img: Optional[np.ndarray] = None
        self.latest_dets: Dict[int, np.ndarray] = {}
        self.captured_frames: List[Dict[int, np.ndarray]] = []
        self.auto_cap_enabled: bool = False
        self.last_auto_cap_time: float = 0.0
        self.calib_res: Optional[Dict[str, Any]] = None
        self.calib_state: CalibState = CalibState.IDLE
        self.notification: Notification = Notification()
        self.cam_info_received: bool = False
        self.bridge: Optional[CvBridge] = CvBridge()

        # 3. Dataset dump directory setup
        if self.enable_dataset_dump:
            self.resolved_dataset_dir = resolve_path(self.dataset_dump_dir)
            os.makedirs(self.resolved_dataset_dir, exist_ok=True)
            self.get_logger().info(
                f"Dataset auto-dump enabled -> {self.resolved_dataset_dir}"
            )

        # 4. Camera matrix fallback from file
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

        if self.enable_dataset_dump and self.cam_mat is not None:
            self._save_dataset_metadata()

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
        )

        # 5. Subscribers and Timers
        qos_cam = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.cam_info_sub = self.create_subscription(
            CameraInfo, self.cam_info_topic, self._cam_info_cb, qos_cam
        )
        self.img_sub = self.create_subscription(
            Image, self.img_topic, self._img_cb, qos_cam
        )
        self.tag_sub = self.create_subscription(
            AprilTagDetectionArray, self.tag_dets_topic, self._tag_dets_cb, qos_cam
        )

        self.gui_timer = self.create_timer(0.033, self._gui_timer_cb)

        self.get_logger().info(
            f"ChessboardTagCalibrator initialized:\n"
            f"  Image topic:    {self.img_topic}\n"
            f"  Camera Info:    {self.cam_info_topic}\n"
            f"  Tags topic:     {self.tag_dets_topic}\n"
            f"  Target Tags:    {self.tag_ids} ({self.tag_names})\n"
            f"  Min tags/frame: {self.min_tags_cnt}\n"
            f"  Target frames:  {self.target_caps_cnt}"
        )

    def _save_dataset_metadata(self) -> None:
        """Saves or updates dataset metadata file with camera & chessboard specifications."""
        if not getattr(self, "enable_dataset_dump", False) or not hasattr(
            self, "resolved_dataset_dir"
        ):
            return
        with self._data_lock:
            k_list = self.cam_mat.tolist() if self.cam_mat is not None else None
            d_list = self.dist_coeffs.tolist() if self.dist_coeffs is not None else None
            cnt = len(self.captured_frames)

        meta = {
            "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "tag_ids": list(self.tag_ids),
            "tag_names": list(self.tag_names),
            "tag_sz": float(self.tag_sz),
            "nominal_dist": float(self.nominal_dist),
            "z_height": float(self.z_height),
            "camera_matrix": k_list,
            "distortion_coefficients": d_list,
            "total_captured_frames": cnt,
        }
        meta_path = os.path.join(self.resolved_dataset_dir, "dataset_meta.json")
        try:
            with open(meta_path, "w", encoding="utf-8") as f:
                json.dump(meta, f, indent=2)
        except Exception as e:
            self.get_logger().error(f"Failed to write dataset_meta.json: {e}")

    def _set_notification(
        self, msg: str, color: Tuple[int, int, int], duration_sec: float = 3.0
    ) -> None:
        """Helper to post a non-blocking temporary visual banner."""
        with self._data_lock:
            self.notification = Notification(
                message=msg,
                color=color,
                timestamp=time.time(),
                duration=duration_sec,
            )

    def _cam_info_cb(self, msg: CameraInfo) -> None:
        """Callback to update camera matrix and distortion from live ROS topic."""
        cam_mat = np.array(msg.k, dtype=np.float64).reshape((3, 3))
        dist_coeffs = np.array(msg.d, dtype=np.float64).reshape((-1, 1))

        with self._data_lock:
            self.cam_mat = cam_mat
            self.dist_coeffs = dist_coeffs
            is_first = not self.cam_info_received
            if is_first:
                self.cam_info_received = True

        if is_first:
            self.get_logger().info(
                f"Received live CameraInfo from {self.cam_info_topic}: "
                f"fx={cam_mat[0, 0]:.1f}, fy={cam_mat[1, 1]:.1f} "
                f"({msg.width}x{msg.height})"
            )
            self._save_dataset_metadata()

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
        tag_map = {
            det.id: np.array([[pt.x, pt.y] for pt in det.corners], dtype=np.float64)
            for det in msg.detections
        }

        trigger_cap = False
        auto_cap_completed = False

        with self._data_lock:
            self.latest_dets = tag_map
            if self.auto_cap_enabled:
                now = time.time()
                if now - self.last_auto_cap_time >= self.auto_cap_interval_sec:
                    if len(self.captured_frames) >= self.target_caps_cnt:
                        self.auto_cap_enabled = False
                        auto_cap_completed = True
                    elif (
                        sum(1 for tid in self.tag_ids if tid in tag_map)
                        >= self.min_tags_cnt
                    ):
                        trigger_cap = True
                        self.last_auto_cap_time = now

        if auto_cap_completed:
            self._set_notification(
                f"Auto-cap done: {self.target_caps_cnt} frames",
                (0, 255, 255),
            )
        elif trigger_cap:
            self.capture_current_frame()

    def capture_current_frame(self) -> bool:
        """Captures valid chessboard tag detections from the current frame."""
        curr_raw = None
        with self._data_lock:
            valid = {
                tid: pts.copy()
                for tid, pts in self.latest_dets.items()
                if tid in self.tag_ids
            }
            valid_cnt = len(valid)
            success = valid_cnt >= self.min_tags_cnt
            if success:
                self.captured_frames.append(valid)
                captured_cnt = len(self.captured_frames)
                if self.latest_img is not None:
                    curr_raw = self.latest_img.copy()

        if success:
            self.get_logger().info(
                f"[CAPTURE] Frame #{captured_cnt} ({valid_cnt} tags)"
            )
            self._set_notification(
                f"Captured #{captured_cnt} ({valid_cnt} tags)", (0, 255, 0)
            )
            if self.enable_dataset_dump:
                self._dump_frame_to_dataset(captured_cnt, valid, curr_raw)
            return True

        self._set_notification(
            f"Failed: need >={self.min_tags_cnt} tags (have {valid_cnt})",
            (0, 100, 255),
        )
        return False

    def _dump_frame_to_dataset(
        self,
        frame_idx: int,
        detections: Dict[int, np.ndarray],
        raw_bgr: Optional[np.ndarray],
    ) -> None:
        """Saves raw image, rendered overlay image, and tag detections JSON."""
        prefix = f"frame_{frame_idx:03d}"

        # 1. Save detections JSON
        try:
            tag_dict = {}
            for tid, pts in detections.items():
                name = (
                    self.tag_names[self.tag_ids.index(tid)]
                    if tid in self.tag_ids
                    and self.tag_ids.index(tid) < len(self.tag_names)
                    else str(tid)
                )
                pts_list = pts.tolist() if isinstance(pts, np.ndarray) else list(pts)
                cx = (
                    float(np.mean(pts[:, 0]))
                    if isinstance(pts, np.ndarray) and len(pts) > 0
                    else 0.0
                )
                cy = (
                    float(np.mean(pts[:, 1]))
                    if isinstance(pts, np.ndarray) and len(pts) > 0
                    else 0.0
                )
                tag_dict[str(tid)] = {
                    "id": int(tid),
                    "name": name,
                    "corners_2d": pts_list,
                    "center_2d": [cx, cy],
                }

            dets_data = {
                "frame_idx": frame_idx,
                "timestamp": time.time(),
                "valid_tags_count": len(detections),
                "tags": tag_dict,
            }
            dets_path = os.path.join(self.resolved_dataset_dir, f"{prefix}_dets.json")
            with open(dets_path, "w", encoding="utf-8") as f:
                json.dump(dets_data, f, indent=2)
        except Exception as e:
            self.get_logger().error(f"Failed to dump detections JSON #{frame_idx}: {e}")

        # 2. Save raw image
        try:
            if raw_bgr is not None:
                raw_path = os.path.join(self.resolved_dataset_dir, f"{prefix}_raw.png")
                cv2.imwrite(raw_path, raw_bgr)
        except Exception as e:
            self.get_logger().error(f"Failed to dump raw image #{frame_idx}: {e}")

        # 3. Save overlay image
        try:
            if self.save_overlay_image and raw_bgr is not None:
                rms = self.calib_res["rms_err"] if self.calib_res is not None else None
                overlay_bgr = self.visualizer.draw(
                    base_img=raw_bgr,
                    detections=detections,
                    captured_cnt=frame_idx,
                    is_auto=self.auto_cap_enabled,
                    calib_state=self.calib_state,
                    rms_err=rms,
                    notification=self.notification,
                )
                overlay_path = os.path.join(
                    self.resolved_dataset_dir, f"{prefix}_overlay.png"
                )
                cv2.imwrite(overlay_path, overlay_bgr)
        except Exception as e:
            self.get_logger().error(f"Failed to dump overlay image #{frame_idx}: {e}")

        # 4. Update dataset metadata
        try:
            self._save_dataset_metadata()
        except Exception as e:
            self.get_logger().error(f"Failed to update dataset metadata: {e}")

        self.get_logger().info(f"[DUMP] Saved {prefix} to {self.resolved_dataset_dir}")

    def _check_prerequisites(self) -> Optional[Tuple[str, bool]]:
        """Validates prerequisites under lock. Returns (message, is_warning) or None."""
        if self.calib_state == CalibState.OPTIMIZING:
            return "Optimization already in progress.", True
        if len(self.captured_frames) < 3:
            return (
                f"Need >= 3 captured frames (have {len(self.captured_frames)})",
                False,
            )
        if self.cam_mat is None or self.dist_coeffs is None:
            return "Camera intrinsics missing", False
        return None

    def run_calibration_async(self) -> None:
        """Spawns non-blocking worker thread to run Bundle Adjustment solver."""
        err_msg: Optional[Tuple[str, bool]] = None
        frames_snapshot = None
        cam_mat_snapshot = None
        dist_snapshot = None
        epoch = 0

        with self._data_lock:
            err_msg = self._check_prerequisites()
            if err_msg is None:
                frames_snapshot = [f.copy() for f in self.captured_frames]
                cam_mat_snapshot = self.cam_mat.copy()
                dist_snapshot = self.dist_coeffs.copy()
                self.calib_state = CalibState.OPTIMIZING
                self._solve_epoch += 1
                epoch = self._solve_epoch

        if err_msg is not None:
            text, is_warn = err_msg
            color = (0, 255, 255) if is_warn else (0, 100, 255)
            self._set_notification(text, color)
            if is_warn:
                self.get_logger().warn(text)
            else:
                self.get_logger().error(text)
            return

        self._set_notification(
            "Optimizing Bundle Adjustment...", (0, 255, 255), duration_sec=5.0
        )
        self.get_logger().info(
            f"Solving Planar Bundle Adjustment on {len(frames_snapshot)} frames in background..."
        )

        worker = threading.Thread(
            target=self._solve_worker,
            kwargs={
                "frames": frames_snapshot,
                "cam_mat": cam_mat_snapshot,
                "dist_coeffs": dist_snapshot,
                "epoch": epoch,
            },
            daemon=True,
        )
        worker.start()

    def _solve_worker(
        self,
        *,
        frames: List[Dict[int, np.ndarray]],
        cam_mat: np.ndarray,
        dist_coeffs: np.ndarray,
        epoch: int,
    ) -> None:
        """Worker thread executing Bundle Adjustment without blocking ROS or GUI."""
        try:
            res = self.solver.solve(frames, cam_mat, dist_coeffs)
            with self._data_lock:
                if epoch != self._solve_epoch:
                    return
                self.calib_res = res
                self.calib_state = CalibState.OPTIMIZED

            print(format_calibration_report(res, self.tag_ids))
            self._set_notification(
                f"Solved! RMS Error: {res['rms_err']:.3f}px",
                (0, 255, 0),
                duration_sec=3.0,
            )
        except Exception as err:
            with self._data_lock:
                if epoch != self._solve_epoch:
                    return
                self.calib_state = CalibState.ERROR

            if rclpy is not None and rclpy.ok():
                self.get_logger().error(f"Optimization failed: {err}")
            self._set_notification(
                f"Optimization error: {err}", (0, 0, 255), duration_sec=3.0
            )

    def save_calibration(self) -> None:
        """Saves active calibration parameters to the target YAML file."""
        with self._data_lock:
            res_snapshot = self.calib_res

        if res_snapshot is None:
            self._set_notification("Calibrate before saving", (0, 100, 255))
            self.get_logger().warn("Run calibration before saving.")
            return

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
        self._handle_key_event(key)

    def _reset_captured_samples(self) -> None:
        """Resets all captured frames and calibration state."""
        with self._data_lock:
            self.captured_frames.clear()
            self.calib_res = None
            self.calib_state = CalibState.IDLE
            self._solve_epoch += 1
        self._save_dataset_metadata()
        self._set_notification("Samples reset", (0, 200, 255))
        self.get_logger().info("Reset all captured frames.")

    def _toggle_auto_capture(self) -> None:
        """Toggles periodic auto-capture mode."""
        with self._data_lock:
            self.auto_cap_enabled = not self.auto_cap_enabled
            enabled = self.auto_cap_enabled
        self._set_notification(
            f"Auto-cap: {'ON' if enabled else 'OFF'}",
            (0, 255, 255) if enabled else (180, 180, 180),
        )
        self.get_logger().info(f"Auto-capture toggled: {enabled}")

    def _handle_key_event(self, key: int) -> None:
        """Dispatches actions corresponding to keyboard inputs in visualizer."""
        if key == 32:  # SPACE
            self.capture_current_frame()
        elif key in (ord("c"), ord("C")):
            self.run_calibration_async()
        elif key in (ord("s"), ord("S")):
            self.save_calibration()
        elif key in (ord("r"), ord("R")):
            self._reset_captured_samples()
        elif key in (ord("a"), ord("A")):
            self._toggle_auto_capture()
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
