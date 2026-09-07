#!/usr/bin/env python3
"""
Chessboard AprilTag Distance & Pose Calibrator for LeKiwi.
Optimizes 3D tag coordinates (x, y, z) and yaw using multi-view Bundle Adjustment.
Standard ROS 2 Node with interactive OpenCV capture and visualization.
"""

import math
import os
import time
import yaml
import numpy as np
import cv2
from scipy.optimize import least_squares

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from apriltag_msgs.msg import AprilTagDetectionArray

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None


class ChessboardTagCalibSolver:
    """Solves optimal 3D coordinates and orientations for chessboard tags using Bundle Adjustment."""

    def __init__(
        self, tag_ids=(1, 4, 3, 6), tag_sz=0.02, nominal_dist=0.38, z_priors=None
    ):
        self.tag_ids = list(tag_ids)
        self.tag_sz = tag_sz
        self.nominal_dist = nominal_dist
        self.z_priors = list(z_priors) if z_priors is not None else [0.0] * len(tag_ids)

    def _get_local_corners(self, center_pt, sz, yaw_rad):
        h = sz / 2.0
        local_pts = np.array(
            [[-h, -h, 0.0], [h, -h, 0.0], [h, h, 0.0], [-h, h, 0.0]], dtype=np.float64
        )
        c, s = math.cos(yaw_rad), math.sin(yaw_rad)
        rot_mat = np.array(
            [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64
        )
        return (rot_mat @ local_pts.T).T + np.array(center_pt, dtype=np.float64)

    def init_params(self):
        d = self.nominal_dist
        z = self.z_priors
        init_tags = np.array(
            [
                d,
                0.0,
                z[1],
                0.0,  # Tag 1 (H1)
                d,
                d,
                z[2],
                0.0,  # Tag 2 (H8)
                0.0,
                d,
                z[3],
                0.0,  # Tag 3 (A8)
            ],
            dtype=np.float64,
        )

        lb = np.array(
            [
                d - 0.05,
                -0.05,
                z[1] - 0.005,
                -math.pi / 4.0,
                d - 0.05,
                d - 0.05,
                z[2] - 0.005,
                -math.pi / 4.0,
                -0.05,
                d - 0.05,
                z[3] - 0.005,
                -math.pi / 4.0,
            ],
            dtype=np.float64,
        )

        ub = np.array(
            [
                d + 0.05,
                0.05,
                z[1] + 0.005,
                math.pi / 4.0,
                d + 0.05,
                d + 0.05,
                z[2] + 0.005,
                math.pi / 4.0,
                0.05,
                d + 0.05,
                z[3] + 0.005,
                math.pi / 4.0,
            ],
            dtype=np.float64,
        )

        return init_tags, (lb, ub)

    def build_3d_corners(self, tag_params):
        corners_3d = {
            self.tag_ids[0]: self._get_local_corners(
                [0.0, 0.0, self.z_priors[0]], self.tag_sz, 0.0
            )
        }
        for idx in range(1, 4):
            base = (idx - 1) * 4
            px, py, pz, yaw = tag_params[base : base + 4]
            corners_3d[self.tag_ids[idx]] = self._get_local_corners(
                [px, py, pz], self.tag_sz, yaw
            )
        return corners_3d

    def solve(self, frames_dets, cam_mat, dist_coeffs):
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

        def reproj_residual_func(param_vec):
            current_3d = self.build_3d_corners(param_vec[:12])
            residuals = []
            for k_idx, current_dets in enumerate(valid_frames):
                c_offset = 12 + k_idx * 6
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

        opt_tags = res.x[:12]
        residuals = reproj_residual_func(res.x)

        tag_results = {
            self.tag_ids[0]: {
                "name": "A1",
                "x": 0.0,
                "y": 0.0,
                "z": float(self.z_priors[0]),
                "yaw": 0.0,
            }
        }
        names = ["H1", "H8", "A8"]
        for idx in range(1, 4):
            b = (idx - 1) * 4
            tag_results[self.tag_ids[idx]] = {
                "name": names[idx - 1],
                "x": float(opt_tags[b]),
                "y": float(opt_tags[b + 1]),
                "z": float(opt_tags[b + 2]),
                "yaw": float(opt_tags[b + 3]),
            }

        return {
            "tags": tag_results,
            "mean_err": float(np.mean(np.abs(residuals))),
            "rms_err": float(np.sqrt(np.mean(residuals**2))),
            "num_frames": len(valid_frames),
        }


def resolve_path(path_str):
    """Resolves package:// URI or relative paths into absolute filesystem path."""
    if not path_str:
        return ""
    if path_str.startswith("package://"):
        stripped = path_str[len("package://") :]
        parts = stripped.split("/", 1)
        if get_package_share_directory is not None:
            pkg_share = get_package_share_directory(parts[0])
            return os.path.join(pkg_share, parts[1] if len(parts) > 1 else "")
    return os.path.abspath(os.path.expanduser(path_str))


def parse_camera_info(info_path):
    """Parses camera matrix K and distortion coefficients D from camera_info yaml file."""
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


def save_to_chessboard_yaml(res_data, target_file):
    """Writes calibrated tag positions and parameters to target chessboard_tags.yaml."""
    resolved = resolve_path(target_file)
    if not os.path.exists(resolved):
        raise FileNotFoundError(f"Target YAML file not found: {resolved}")

    with open(resolved, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f)

    tags = res_data["tags"]
    tag_ids = [1, 4, 3, 6]
    coords = {
        tid: np.array([tags[tid]["x"], tags[tid]["y"], tags[tid]["z"]])
        for tid in tag_ids
    }

    d_a1_h1 = np.linalg.norm(coords[4] - coords[1])
    d_a1_a8 = np.linalg.norm(coords[6] - coords[1])
    avg_dist = round(float((d_a1_h1 + d_a1_a8) / 2.0), 4)

    params = config["/**"]["ros__parameters"]
    params["tag_distance"] = avg_dist
    params["tags"]["positions_x"] = [round(tags[tid]["x"], 4) for tid in tag_ids]
    params["tags"]["positions_y"] = [round(tags[tid]["y"], 4) for tid in tag_ids]
    params["tags"]["positions_z"] = [round(tags[tid]["z"], 4) for tid in tag_ids]
    params["tags"]["yaws"] = [round(tags[tid]["yaw"], 4) for tid in tag_ids]

    with open(resolved, "w", encoding="utf-8") as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)

    print(f"\n[SUCCESS] Updated {resolved} with calibrated parameters.")


class ChessboardTagCalibratorNode(Node):
    """ROS 2 Node providing interactive multi-view capture and Bundle Adjustment calibration."""

    def __init__(self):
        super().__init__("chessboard_tag_calibrator")

        defaults = {
            "img_topic": "/cameras/stereo_left/image_raw",
            "tag_dets_topic": "/tag_detections",
            "cam_info_path": "package://lekiwi_bringup/config/perception/camera_info/stereo_left.yaml",
            "output_yaml_path": "package://lekiwi_bringup/config/localization/chessboard_tags.yaml",
            "tag_ids": [1, 4, 3, 6],
            "tag_names": ["A1", "H1", "H8", "A8"],
            "tag_sz": 0.02,
            "nominal_dist": 0.38,
            "z_priors": [0.0, 0.0, 0.0, 0.0],
            "min_tags_cnt": 2,
            "target_caps_cnt": 50,
            "auto_cap_interval_sec": 1.0,
            "window_name": "LeKiwi Chessboard Tag Calibrator",
            "disp_scale": 1.0,
        }
        for name, val in defaults.items():
            self.declare_parameter(name, val)
            setattr(self, name, self.get_parameter(name).value)

        # Load camera intrinsics
        try:
            self.cam_mat, self.dist_coeffs = parse_camera_info(self.cam_info_path)
            self.get_logger().info(f"Loaded camera matrix from: {self.cam_info_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load camera info: {e}")
            self.cam_mat, self.dist_coeffs = None, None

        self.solver = ChessboardTagCalibSolver(
            tag_ids=self.tag_ids,
            tag_sz=self.tag_sz,
            nominal_dist=self.nominal_dist,
            z_priors=self.z_priors,
        )

        # Internal state
        self.bridge = CvBridge() if CvBridge is not None else None
        self.latest_img = None
        self.latest_dets = {}
        self.captured_frames = []
        self.auto_cap_enabled = False
        self.last_auto_cap_time = 0.0
        self.calib_res = None
        self.window_initialized = False

        # Notification pop-up state (top-right corner, 1s fadeout)
        self.notification_msg = ""
        self.notification_color = (0, 255, 0)
        self.notification_time = 0.0

        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.img_sub = self.create_subscription(
            Image, self.img_topic, self._img_cb, qos
        )
        self.tag_dets_sub = self.create_subscription(
            AprilTagDetectionArray, self.tag_dets_topic, self._tag_dets_cb, qos
        )

        self.get_logger().info(
            f"Calibrator listening to {self.img_topic} & {self.tag_dets_topic}"
        )

        # GUI timer running at ~30 Hz (Default GUI enabled, library handles any display error)
        self.gui_timer = self.create_timer(0.033, self._gui_timer_cb)

    def _img_cb(self, msg):
        try:
            if self.bridge is not None:
                self.latest_img = self.bridge.imgmsg_to_cv2(
                    msg, desired_encoding="bgr8"
                )
            else:
                data = np.frombuffer(msg.data, dtype=np.uint8)
                if msg.encoding in ("rgb8", "bgr8"):
                    img = data.reshape((msg.height, msg.width, 3))
                    self.latest_img = (
                        cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                        if msg.encoding == "rgb8"
                        else img
                    )
        except Exception as err:
            self.get_logger().warn(
                f"Image conversion error: {err}", throttle_duration_sec=5.0
            )

    def _tag_dets_cb(self, msg):
        tag_map = {}
        for det in msg.detections:
            tag_map[det.id] = np.array(
                [[pt.x, pt.y] for pt in det.corners], dtype=np.float64
            )
        self.latest_dets = tag_map

        if self.auto_cap_enabled:
            now = time.time()
            if now - self.last_auto_cap_time >= self.auto_cap_interval_sec:
                if len(self.captured_frames) >= self.target_caps_cnt:
                    self.auto_cap_enabled = False
                    self.notification_msg = f"Auto-cap done: {self.target_caps_cnt} frames"
                    self.notification_color = (0, 255, 255)
                    self.notification_time = now
                    self.get_logger().info(f"Target sample count ({self.target_caps_cnt}) reached.")
                elif (
                    sum(1 for tid in self.tag_ids if tid in tag_map)
                    >= self.min_tags_cnt
                ):
                    if self.capture_current_frame():
                        self.last_auto_cap_time = now

    def capture_current_frame(self):
        valid = {
            tid: pts.copy()
            for tid, pts in self.latest_dets.items()
            if tid in self.tag_ids
        }
        if len(valid) >= self.min_tags_cnt:
            self.captured_frames.append(valid)
            self.notification_msg = f"Captured #{len(self.captured_frames)} ({len(valid)} tags)"
            self.notification_color = (0, 255, 0)
            self.notification_time = time.time()
            self.get_logger().info(
                f"[CAPTURE] Added frame #{len(self.captured_frames)} with {len(valid)} tags"
            )
            return True
        self.notification_msg = f"Failed: need >= {self.min_tags_cnt} tags (have {len(valid)})"
        self.notification_color = (0, 100, 255)
        self.notification_time = time.time()
        self.get_logger().warn(
            f"Cannot capture: need >= {self.min_tags_cnt} tags (have {len(valid)})"
        )
        return False

    def run_calibration(self):
        if len(self.captured_frames) < 3:
            self.get_logger().error(
                f"Cannot calibrate: need >= 3 samples, have {len(self.captured_frames)}"
            )
            return
        if self.cam_mat is None or self.dist_coeffs is None:
            self.get_logger().error("Camera matrix or distortion coefficients missing.")
            return

        self.get_logger().info(
            f"Solving Bundle Adjustment on {len(self.captured_frames)} frames..."
        )
        try:
            res = self.solver.solve(
                self.captured_frames, self.cam_mat, self.dist_coeffs
            )
            self.calib_res = res
            self._print_calib_report(res)
        except Exception as e:
            self.get_logger().error(f"Optimization failed: {e}")

    def _print_calib_report(self, res):
        print("\n" + "=" * 64 + "\n          CALIBRATION RESULTS REPORT\n" + "=" * 64)
        print(
            f"Frames: {res['num_frames']} | Mean Error: {res['mean_err']:.4f}px | RMS Error: {res['rms_err']:.4f}px\n"
        )
        tags = res["tags"]
        print("ID | Name |    X (m)   |    Y (m)   |    Z (m)   |  Yaw (deg)")
        print("-" * 64)
        for tid in self.tag_ids:
            if tid in tags:
                t = tags[tid]
                print(
                    f"{tid:2d} | {t['name']:4s} | {t['x']:10.4f} | {t['y']:10.4f} | {t['z']:10.4f} | {math.degrees(t['yaw']):9.2f}"
                )

        coords = {
            tid: np.array([tags[tid]["x"], tags[tid]["y"], tags[tid]["z"]])
            for tid in self.tag_ids
        }
        edges = [("A1->H1", 1, 4), ("H1->H8", 4, 3), ("H8->A8", 3, 6), ("A8->A1", 6, 1)]
        print("\nCorner Distances:")
        for label, u, v in edges:
            dist = np.linalg.norm(coords[u] - coords[v])
            print(f"  - {label:6s}: {dist * 1000.0:6.2f} mm ({dist:.4f} m)")
        print("=" * 64 + "\n")

    def save_calibration(self):
        if self.calib_res is None:
            self.get_logger().warn("Run calibration before saving.")
            return
        try:
            save_to_chessboard_yaml(self.calib_res, self.output_yaml_path)
            self.get_logger().info(f"Saved calibration to {self.output_yaml_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save YAML: {e}")

    def _gui_timer_cb(self):
        if self.latest_img is not None:
            canvas = self.latest_img.copy()
        else:
            canvas = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(
                canvas,
                f"Waiting for {self.img_topic}...",
                (40, 240),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (200, 200, 200),
                1,
            )

        h, w = canvas.shape[:2]
        valid_cnt = 0
        tag_centers = {}

        # Draw detected tags
        colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255)]
        for tid, corners in self.latest_dets.items():
            pts = corners.astype(np.int32)
            is_board_tag = tid in self.tag_ids
            poly_color = (0, 255, 0) if is_board_tag else (0, 165, 255)
            if is_board_tag:
                valid_cnt += 1

            cv2.polylines(canvas, [pts], True, poly_color, 2)
            for k in range(4):
                cv2.circle(canvas, tuple(pts[k]), 4, colors[k], -1)

            cx, cy = int(np.mean(pts[:, 0])), int(np.mean(pts[:, 1]))
            tag_centers[tid] = (cx, cy)
            name = (
                f" ({self.tag_names[self.tag_ids.index(tid)]})" if is_board_tag else ""
            )
            cv2.putText(
                canvas,
                f"ID:{tid}{name}",
                (cx - 25, cy - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                poly_color,
                1,
                cv2.LINE_AA,
            )

        # Draw perimeter lines
        for t1, t2 in [(1, 4), (4, 3), (3, 6), (6, 1)]:
            if t1 in tag_centers and t2 in tag_centers:
                cv2.line(
                    canvas,
                    tag_centers[t1],
                    tag_centers[t2],
                    (255, 200, 0),
                    1,
                    cv2.LINE_AA,
                )

        # Header HUD
        cv2.rectangle(canvas, (0, 0), (w, 36), (30, 30, 30), -1)
        status_str = f"Caps: {len(self.captured_frames)}/{self.target_caps_cnt} | Tags: {valid_cnt}/4 | {'AUTO' if self.auto_cap_enabled else 'MANUAL'}"
        if self.calib_res is not None:
            status_str += f" | RMS: {self.calib_res['rms_err']:.3f}px"
        cv2.putText(
            canvas,
            status_str,
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0) if valid_cnt >= self.min_tags_cnt else (0, 165, 255),
            1,
            cv2.LINE_AA,
        )

        # Footer controls
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

        # Top-right pop-up notification (fades out in 1.0s)
        now = time.time()
        elapsed = now - self.notification_time
        if elapsed < 1.0 and self.notification_msg:
            alpha = max(0.0, min(1.0, 1.0 - elapsed / 1.0))
            font = cv2.FONT_HERSHEY_SIMPLEX
            scale = 0.5
            thickness = 1
            (tw, th), _ = cv2.getTextSize(self.notification_msg, font, scale, thickness)

            box_pad_x, box_pad_y = 10, 6
            box_w = tw + box_pad_x * 2
            box_h = th + box_pad_y * 2
            box_x = max(0, w - box_w - 10)
            box_y = 44  # Positioned right under the top header HUD (36px)

            if box_x >= 0 and box_y + box_h <= h:
                overlay = canvas.copy()
                cv2.rectangle(overlay, (box_x, box_y), (box_x + box_w, box_y + box_h), (20, 20, 20), -1)
                cv2.rectangle(overlay, (box_x, box_y), (box_x + box_w, box_y + box_h), self.notification_color, 1)
                cv2.putText(
                    overlay,
                    self.notification_msg,
                    (box_x + box_pad_x, box_y + th + box_pad_y - 1),
                    font,
                    scale,
                    self.notification_color,
                    thickness,
                    cv2.LINE_AA,
                )
                cv2.addWeighted(overlay, alpha, canvas, 1.0 - alpha, 0, canvas)

        # Scale display
        if abs(self.disp_scale - 1.0) > 1e-3:
            canvas = cv2.resize(
                canvas, (int(w * self.disp_scale), int(h * self.disp_scale))
            )

        # OpenCV display - hardcoded default 640x480 window size with WINDOW_NORMAL
        if not self.window_initialized:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 640, 480)
            self.window_initialized = True

        cv2.imshow(self.window_name, canvas)
        key = cv2.waitKey(1) & 0xFF
        if key == 32:  # SPACE
            self.capture_current_frame()
        elif key in (ord("c"), ord("C")):
            self.run_calibration()
        elif key in (ord("s"), ord("S")):
            self.save_calibration()
            self.notification_msg = "Saved to YAML"
            self.notification_color = (0, 255, 0)
            self.notification_time = time.time()
        elif key in (ord("r"), ord("R")):
            self.captured_frames.clear()
            self.calib_res = None
            self.notification_msg = "Samples reset"
            self.notification_color = (0, 200, 255)
            self.notification_time = time.time()
            self.get_logger().info("Reset all captured samples.")
        elif key in (ord("a"), ord("A")):
            self.auto_cap_enabled = not self.auto_cap_enabled
            self.notification_msg = f"Auto-cap: {'ON' if self.auto_cap_enabled else 'OFF'} ({self.auto_cap_interval_sec}s)"
            self.notification_color = (0, 255, 255) if self.auto_cap_enabled else (180, 180, 180)
            self.notification_time = time.time()
            self.get_logger().info(f"Auto-capture toggled: {self.auto_cap_enabled}")
        elif key in (ord("q"), ord("Q"), 27):  # Q or ESC
            cv2.destroyAllWindows()
            rclpy.shutdown()



def main(args=None):
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
