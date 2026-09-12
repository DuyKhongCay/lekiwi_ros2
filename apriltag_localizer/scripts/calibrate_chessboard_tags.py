#!/usr/bin/env python3
"""Chessboard AprilTag Distance & Pose Calibrator for LeKiwi.

Optimizes 2D tag coordinates (x, y) and yaw in chessboard plane using
multi-view planar Bundle Adjustment.
Maintains manual measured z_height without optimizing the Z-axis.
"""

from dataclasses import dataclass
import datetime
from enum import Enum
import math
import os
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
from scipy.optimize import least_squares
import yaml

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None

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


# ==============================================================================
# 1. DATA MODELS & ENUMS
# ==============================================================================


class CalibState(Enum):
    """Execution state of the calibration process."""

    IDLE = "IDLE"
    CAPTURING = "CAPTURING"
    OPTIMIZING = "OPTIMIZING"
    OPTIMIZED = "OPTIMIZED"
    ERROR = "ERROR"


@dataclass
class Notification:
    """Represents a temporary fading on-screen notification."""

    message: str = ""
    color: Tuple[int, int, int] = (0, 255, 0)
    timestamp: float = 0.0
    duration: float = 1.0


# ==============================================================================
# 2. MATHEMATICAL SOLVER (PURE BUNDLE ADJUSTMENT)
# ==============================================================================


class ChessboardTagCalibSolver:
    """Solves planar 2D coordinates (x, y) and orientations for chessboard tags.

    Uses Multi-View Planar Bundle Adjustment via non-linear least squares.
    """

    def __init__(
        self,
        tag_ids: Tuple[int, ...] = (0, 1, 2, 3),
        tag_sz: float = 0.022,
        nominal_dist: float = 0.39,
        z_height: float = 0.004,
    ):
        """Initializes the calibration solver parameters."""
        self.tag_ids = list(tag_ids)
        self.tag_sz = tag_sz
        self.nominal_dist = nominal_dist
        if isinstance(z_height, (list, tuple)):
            self.z_height = float(z_height[0]) if len(z_height) > 0 else 0.0
        else:
            self.z_height = float(z_height)

    def _get_local_corners(
        self, center_pt: List[float], sz: float, yaw_rad: float
    ) -> np.ndarray:
        """Generates 4 corner coordinates for a tag given center and planar yaw."""
        h = sz / 2.0
        local_pts = np.array(
            [[h, -h, 0.0], [-h, -h, 0.0], [-h, h, 0.0], [h, h, 0.0]],
            dtype=np.float64,
        )
        c, s = math.cos(yaw_rad), math.sin(yaw_rad)
        rot_mat = np.array(
            [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        return (rot_mat @ local_pts.T).T + np.array(center_pt, dtype=np.float64)

    def init_params(self) -> Tuple[np.ndarray, Tuple[np.ndarray, np.ndarray]]:
        """Provides nominal initial estimates and parameter bounds."""
        d = self.nominal_dist
        # 9 parameters for tags 1, 2, 3: [x1, y1, yaw1, x2, y2, yaw2, x3, y3, yaw3]
        init_tags = np.array(
            [
                d,
                0.0,
                0.0,  # Tag 1 (H1)
                d,
                d,
                0.0,  # Tag 2 (H8)
                0.0,
                d,
                0.0,  # Tag 3 (A8)
            ],
            dtype=np.float64,
        )

        lb = np.array(
            [
                d - 0.15,
                -0.15,
                -math.pi,
                d - 0.15,
                d - 0.15,
                -math.pi,
                -0.15,
                d - 0.15,
                -math.pi,
            ],
            dtype=np.float64,
        )

        ub = np.array(
            [
                d + 0.15,
                0.15,
                math.pi,
                d + 0.15,
                d + 0.15,
                math.pi,
                0.15,
                d + 0.15,
                math.pi,
            ],
            dtype=np.float64,
        )

        return init_tags, (lb, ub)

    def build_3d_corners(self, tag_params: np.ndarray) -> Dict[int, np.ndarray]:
        """Constructs 3D corner coordinates for all tags from state vector."""
        corners_3d = {
            self.tag_ids[0]: self._get_local_corners(
                [0.0, 0.0, self.z_height], self.tag_sz, 0.0
            )
        }
        for idx in range(1, 4):
            base = (idx - 1) * 3
            px, py, yaw = tag_params[base : base + 3]
            corners_3d[self.tag_ids[idx]] = self._get_local_corners(
                [px, py, self.z_height], self.tag_sz, yaw
            )
        return corners_3d

    def solve(
        self,
        frames_dets: List[Dict[int, np.ndarray]],
        cam_mat: np.ndarray,
        dist_coeffs: np.ndarray,
    ) -> Dict[str, Any]:
        """Runs non-linear least squares Bundle Adjustment over captured frames."""
        if len(frames_dets) < 3:
            raise ValueError(f"Need at least 3 valid frames, got {len(frames_dets)}")

        init_tags, (lb, ub) = self.init_params()
        init_3d = self.build_3d_corners(init_tags)

        cam_poses_init = []
        valid_frames = []

        for dets in frames_dets:
            obj_pts, img_pts = [], []
            for tag_id, corners_2d in dets.items():
                if tag_id in init_3d:
                    obj_pts.extend(init_3d[tag_id])
                    img_pts.extend(corners_2d)

            if len(obj_pts) < 8:  # At least 2 tags needed for PnP
                continue

            success, rvec, tvec = cv2.solvePnP(
                np.array(obj_pts, dtype=np.float64),
                np.array(img_pts, dtype=np.float64),
                cam_mat,
                dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
            if success:
                cam_poses_init.extend(
                    [
                        rvec[0, 0],
                        rvec[1, 0],
                        rvec[2, 0],
                        tvec[0, 0],
                        tvec[1, 0],
                        tvec[2, 0],
                    ]
                )
                valid_frames.append(dets)

        if len(valid_frames) < 3:
            raise ValueError(
                f"Too few frames with good PnP initialization: {len(valid_frames)}"
            )

        x0 = np.concatenate([init_tags, np.array(cam_poses_init, dtype=np.float64)])
        num_cams = len(valid_frames)
        lower_bnds = np.concatenate([lb, np.full(num_cams * 6, -np.inf)])
        upper_bnds = np.concatenate([ub, np.full(num_cams * 6, np.inf)])

        def reproj_residual_func(param_vec: np.ndarray) -> np.ndarray:
            current_3d = self.build_3d_corners(param_vec[:9])
            residuals = []
            for k_idx, current_dets in enumerate(valid_frames):
                c_offset = 9 + k_idx * 6
                rvec = param_vec[c_offset : c_offset + 3].reshape(3, 1)
                tvec = param_vec[c_offset + 3 : c_offset + 6].reshape(3, 1)
                for tag_id, observed_corners in current_dets.items():
                    if tag_id in current_3d:
                        proj_2d, _ = cv2.projectPoints(
                            current_3d[tag_id], rvec, tvec, cam_mat, dist_coeffs
                        )
                        residuals.extend(
                            (observed_corners - proj_2d.reshape(-1, 2)).ravel()
                        )
            return np.array(residuals, dtype=np.float64)

        res = least_squares(
            reproj_residual_func,
            x0,
            bounds=(lower_bnds, upper_bnds),
            method="trf",
            ftol=1e-6,
            xtol=1e-6,
            loss="soft_l1",
            verbose=0,
        )

        opt_tags = res.x[:9]
        residuals = reproj_residual_func(res.x)

        tag_results = {
            self.tag_ids[0]: {
                "name": "A1",
                "x": 0.0,
                "y": 0.0,
                "z": float(self.z_height),
                "yaw": 0.0,
            }
        }
        names = ["H1", "H8", "A8"]
        for idx in range(1, 4):
            b = (idx - 1) * 3
            tag_results[self.tag_ids[idx]] = {
                "name": names[idx - 1],
                "x": float(opt_tags[b]),
                "y": float(opt_tags[b + 1]),
                "z": float(self.z_height),
                "yaw": float(opt_tags[b + 2]),
            }

        return {
            "tags": tag_results,
            "mean_err": float(np.mean(np.abs(residuals))),
            "rms_err": float(np.sqrt(np.mean(residuals**2))),
            "num_frames": len(valid_frames),
        }


# ==============================================================================
# 3. HUD & OPENCV VISUALIZER LAYER
# ==============================================================================


class CalibratorVisualizer:
    """Manages OpenCV GUI rendering, HUD overlays, and notification popups."""

    def __init__(
        self,
        window_name: str = "LeKiwi Chessboard Tag Calibrator",
        disp_scale: float = 1.0,
        tag_ids: Tuple[int, ...] = (0, 1, 2, 3),
        tag_names: Tuple[str, ...] = ("A1", "H1", "H8", "A8"),
        target_caps_cnt: int = 50,
        min_tags_cnt: int = 2,
    ):
        """Initializes visualizer display properties and label maps."""
        self.window_name = window_name
        self.disp_scale = disp_scale
        self.tag_ids = list(tag_ids)
        self.tag_names = list(tag_names)
        self.target_caps_cnt = target_caps_cnt
        self.min_tags_cnt = min_tags_cnt
        self.window_initialized = False

    def draw(
        self,
        base_img: Optional[np.ndarray],
        detections: Dict[int, np.ndarray],
        captured_cnt: int,
        is_auto: bool,
        calib_state: CalibState,
        rms_err: Optional[float],
        notification: Notification,
    ) -> np.ndarray:
        """Renders detections, perimeter lines, HUD elements, and popups onto canvas."""
        if base_img is not None:
            canvas = base_img.copy()
        else:
            canvas = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(
                canvas,
                "Waiting for camera image...",
                (40, 240),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (200, 200, 200),
                1,
            )

        h, w = canvas.shape[:2]
        valid_cnt = 0
        tag_centers = {}

        # 1. Draw detected AprilTags
        corner_colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255)]
        for tid, corners in detections.items():
            pts = corners.astype(np.int32)
            is_board_tag = tid in self.tag_ids
            poly_color = (0, 255, 0) if is_board_tag else (0, 165, 255)
            if is_board_tag:
                valid_cnt += 1

            cv2.polylines(canvas, [pts], True, poly_color, 2)
            for k in range(min(4, len(pts))):
                cv2.circle(canvas, tuple(pts[k]), 4, corner_colors[k], -1)

            cx, cy = int(np.mean(pts[:, 0])), int(np.mean(pts[:, 1]))
            tag_centers[tid] = (cx, cy)
            name_suffix = (
                f" ({self.tag_names[self.tag_ids.index(tid)]})"
                if is_board_tag and tid in self.tag_ids
                else ""
            )
            cv2.putText(
                canvas,
                f"ID:{tid}{name_suffix}",
                (cx - 25, cy - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                poly_color,
                1,
                cv2.LINE_AA,
            )

        # 2. Draw perimeter connecting board tags
        if len(self.tag_ids) >= 4:
            loop = list(self.tag_ids[:4]) + [self.tag_ids[0]]
            for i in range(len(loop) - 1):
                t1, t2 = loop[i], loop[i + 1]
                if t1 in tag_centers and t2 in tag_centers:
                    cv2.line(
                        canvas,
                        tag_centers[t1],
                        tag_centers[t2],
                        (255, 200, 0),
                        1,
                        cv2.LINE_AA,
                    )

        # 3. Top Header HUD
        cv2.rectangle(canvas, (0, 0), (w, 36), (30, 30, 30), -1)
        mode_str = "AUTO" if is_auto else "MANUAL"
        status_str = (
            f"Caps: {captured_cnt}/{self.target_caps_cnt} | "
            f"Tags: {valid_cnt}/{len(self.tag_ids)} | {mode_str}"
        )
        if calib_state == CalibState.OPTIMIZING:
            status_str += " | [OPTIMIZING...]"
        elif rms_err is not None:
            status_str += f" | RMS: {rms_err:.3f}px"

        hud_color = (
            (0, 255, 255)
            if calib_state == CalibState.OPTIMIZING
            else ((0, 255, 0) if valid_cnt >= self.min_tags_cnt else (0, 165, 255))
        )
        cv2.putText(
            canvas,
            status_str,
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            hud_color,
            1,
            cv2.LINE_AA,
        )

        # 4. Footer Controls Hint
        cv2.rectangle(canvas, (0, h - 24), (w, h), (30, 30, 30), -1)
        cv2.putText(
            canvas,
            "[SPACE] Cap | [C] Calib | [S] Save | [R] Reset | [A] Auto | [Q] Quit",
            (10, h - 7),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (220, 220, 220),
            1,
            cv2.LINE_AA,
        )

        # 5. Top-right Fading Notification Pop-up
        now = time.time()
        elapsed = now - notification.timestamp
        if elapsed < notification.duration and notification.message:
            alpha = max(0.0, min(1.0, 1.0 - elapsed / notification.duration))
            font = cv2.FONT_HERSHEY_SIMPLEX
            scale = 0.5
            thickness = 1
            (tw, th), _ = cv2.getTextSize(notification.message, font, scale, thickness)

            box_pad_x, box_pad_y = 10, 6
            box_w = tw + box_pad_x * 2
            box_h = th + box_pad_y * 2
            box_x = max(0, w - box_w - 10)
            box_y = 44  # Right below the 36px top header

            if box_x >= 0 and box_y + box_h <= h:
                overlay = canvas.copy()
                cv2.rectangle(
                    overlay,
                    (box_x, box_y),
                    (box_x + box_w, box_y + box_h),
                    (20, 20, 20),
                    -1,
                )
                cv2.rectangle(
                    overlay,
                    (box_x, box_y),
                    (box_x + box_w, box_y + box_h),
                    notification.color,
                    1,
                )
                cv2.putText(
                    overlay,
                    notification.message,
                    (box_x + box_pad_x, box_y + th + box_pad_y - 1),
                    font,
                    scale,
                    notification.color,
                    thickness,
                    cv2.LINE_AA,
                )
                cv2.addWeighted(overlay, alpha, canvas, 1.0 - alpha, 0, canvas)

        # 6. Apply Display Scaling
        if abs(self.disp_scale - 1.0) > 1e-3:
            canvas = cv2.resize(
                canvas,
                (int(w * self.disp_scale), int(h * self.disp_scale)),
            )

        return canvas

    def show(self, canvas: np.ndarray) -> int:
        """Displays canvas in OpenCV window and returns captured keycode."""
        if not self.window_initialized:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 640, 480)
            self.window_initialized = True

        cv2.imshow(self.window_name, canvas)
        return cv2.waitKey(1) & 0xFF


# ==============================================================================
# 4. STORAGE & PATH RESOLUTION UTILITIES
# ==============================================================================


def resolve_path(path_str: str) -> str:
    """Resolves package:// URI or relative paths into absolute filesystem paths."""
    if not path_str:
        return ""
    if path_str.startswith("package://"):
        stripped = path_str[len("package://") :]
        parts = stripped.split("/", 1)
        if get_package_share_directory is not None:
            pkg_share = get_package_share_directory(parts[0])
            return os.path.join(pkg_share, parts[1] if len(parts) > 1 else "")
    return os.path.abspath(os.path.expanduser(path_str))


def parse_camera_info(info_path: str) -> Tuple[np.ndarray, np.ndarray]:
    """Parses camera matrix K and distortion coefficients D from YAML."""
    resolved = resolve_path(info_path)
    if not os.path.exists(resolved):
        raise FileNotFoundError(f"Camera info file not found: {resolved}")
    with open(resolved, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    cam_mat = np.array(data["camera_matrix"]["data"], dtype=np.float64).reshape((3, 3))
    dist_coeffs = np.array(
        data["distortion_coefficients"]["data"], dtype=np.float64
    ).reshape((-1, 1))
    return cam_mat, dist_coeffs


def save_to_chessboard_yaml(
    res_data: Dict[str, Any], target_file: str, tag_ids: Optional[List[int]] = None
) -> None:
    """Writes calibrated tag positions directly to YAML using standard yaml.dump."""
    resolved = resolve_path(target_file)
    os.makedirs(os.path.dirname(resolved), exist_ok=True)

    tags = res_data["tags"]
    if tag_ids is None:
        tag_ids = list(tags.keys())

    names = [tags[tid]["name"] for tid in tag_ids]
    px = [float(round(tags[tid]["x"], 4)) for tid in tag_ids]
    py = [float(round(tags[tid]["y"], 4)) for tid in tag_ids]
    pz = [float(round(tags[tid]["z"], 4)) for tid in tag_ids]
    yaws = [float(round(tags[tid]["yaw"], 4)) for tid in tag_ids]

    now_str = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    output_dict = {
        "ids": tag_ids,
        "names": names,
        "positions_x": px,
        "positions_y": py,
        "positions_z": pz,
        "yaws": yaws,
    }

    yaml_header = f"# Calibration Date & Time: {now_str}\n"
    yaml_body = yaml.dump(
        output_dict, default_flow_style=None, sort_keys=False, indent=2
    )

    with open(resolved, "w", encoding="utf-8") as f:
        f.write(yaml_header + yaml_body)


# ==============================================================================
# 5. ROS 2 NODE COORDINATOR & THREAD SAFE WORKER
# ==============================================================================


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
            "output_yaml_path": "package://apriltag_localizer/config/calib_result.yaml",
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
