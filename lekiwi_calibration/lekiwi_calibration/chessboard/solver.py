"""Chessboard AprilTag Bundle Adjustment Solver and YAML serialization."""

import datetime
import math
import os
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
from scipy.optimize import least_squares
import yaml

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None


def resolve_path(path_str: str) -> str:
    """Resolves package:// URI or relative paths into absolute filesystem paths."""
    if not path_str:
        return ""
    if path_str.startswith("package://"):
        stripped = path_str[len("package://") :]
        parts = stripped.split("/", 1)
        pkg_name = parts[0]
        subpath = parts[1] if len(parts) > 1 else ""

        # Check source package directory in current workspace first
        ws_pkg = os.path.join("/root/docker_ws/lekiwi_ros2", pkg_name)
        if os.path.isdir(ws_pkg):
            return os.path.abspath(os.path.join(ws_pkg, subpath))

        if get_package_share_directory is not None:
            try:
                pkg_share = get_package_share_directory(pkg_name)
                return os.path.abspath(os.path.join(pkg_share, subpath))
            except Exception:
                pass
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

    def _init_frame_poses(
        self,
        frames_dets: List[Dict[int, np.ndarray]],
        init_3d: Dict[int, np.ndarray],
        cam_mat: np.ndarray,
        dist_coeffs: np.ndarray,
        max_frames: int = 50,
    ) -> Tuple[List[Dict[int, np.ndarray]], List[float]]:
        """Initializes camera extrinsic poses via PnP on frames with sufficient tags."""
        cam_poses_init: List[float] = []
        valid_frames: List[Dict[int, np.ndarray]] = []

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

            if len(valid_frames) >= max_frames:
                break

        return valid_frames, cam_poses_init

    def _build_residual_func(
        self,
        valid_frames: List[Dict[int, np.ndarray]],
        cam_mat: np.ndarray,
        dist_coeffs: np.ndarray,
    ):
        """Constructs reprojection residual callable for least_squares optimizer."""

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

        return reproj_residual_func

    def _extract_raw_tag_results(
        self, opt_tags: np.ndarray
    ) -> Dict[int, Dict[str, Any]]:
        """Extracts planar tag coordinates relative to A1 tag origin."""
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
        return tag_results

    def _shift_to_center(
        self, tags: Dict[int, Dict[str, Any]]
    ) -> Dict[int, Dict[str, Any]]:
        """Translates tag coordinates so that origin is at the geometric center of the board."""
        if not tags:
            return {}

        cx = float(np.mean([tags[tid]["x"] for tid in tags]))
        cy = float(np.mean([tags[tid]["y"] for tid in tags]))

        centered_tags = {}
        for tid, data in tags.items():
            centered_tags[tid] = {
                "name": data["name"],
                "x": float(data["x"] - cx),
                "y": float(data["y"] - cy),
                "z": float(data["z"]),
                "yaw": float(data["yaw"]),
            }
        return centered_tags

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

        valid_frames, cam_poses_init = self._init_frame_poses(
            frames_dets, init_3d, cam_mat, dist_coeffs
        )
        if len(valid_frames) < 3:
            raise ValueError(
                f"Too few frames with good PnP initialization: {len(valid_frames)}"
            )

        x0 = np.concatenate([init_tags, np.array(cam_poses_init, dtype=np.float64)])
        num_cams = len(valid_frames)
        lower_bnds = np.concatenate([lb, np.full(num_cams * 6, -np.inf)])
        upper_bnds = np.concatenate([ub, np.full(num_cams * 6, np.inf)])

        reproj_residual_func = self._build_residual_func(
            valid_frames, cam_mat, dist_coeffs
        )

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

        raw_tags = self._extract_raw_tag_results(res.x[:9])
        centered_tags = self._shift_to_center(raw_tags)
        residuals = reproj_residual_func(res.x)

        return {
            "tags": centered_tags,
            "mean_err": float(np.mean(np.abs(residuals))),
            "rms_err": float(np.sqrt(np.mean(residuals**2))),
            "num_frames": len(valid_frames),
        }


def format_calibration_report(res: Dict[str, Any], tag_ids: List[int]) -> str:
    """Formats planar Bundle Adjustment results into a readable terminal report."""
    lines = [
        "\n" + "=" * 64,
        "          CALIBRATION RESULTS REPORT (PLANAR)",
        "=" * 64,
        (
            f"Frames: {res['num_frames']} | "
            f"Mean Error: {res['mean_err']:.4f}px | "
            f"RMS Error: {res['rms_err']:.4f}px\n"
        ),
        "ID | Name |    X (m)   |    Y (m)   |    Z (m)   |  Yaw (deg)",
        "-" * 64,
    ]

    tags = res.get("tags", {})
    for tid in tag_ids:
        if tid in tags:
            t = tags[tid]
            lines.append(
                f"{tid:2d} | {t['name']:4s} | "
                f"{t['x']:10.4f} | {t['y']:10.4f} | {t['z']:10.4f} | "
                f"{math.degrees(t['yaw']):9.2f}"
            )

    coords = {
        tid: np.array([tags[tid]["x"], tags[tid]["y"], tags[tid]["z"]])
        for tid in tag_ids
        if tid in tags
    }
    if len(tag_ids) >= 4:
        edges = [
            ("A1->H1", tag_ids[0], tag_ids[1]),
            ("H1->H8", tag_ids[1], tag_ids[2]),
            ("H8->A8", tag_ids[2], tag_ids[3]),
            ("A8->A1", tag_ids[3], tag_ids[0]),
        ]
        lines.append("\nCorner Distances:")
        for label, u, v in edges:
            if u in coords and v in coords:
                dist = float(np.linalg.norm(coords[u] - coords[v]))
                lines.append(f"  - {label:6s}: {dist * 1000.0:6.2f} mm ({dist:.4f} m)")

    lines.append("=" * 64 + "\n")
    return "\n".join(lines)

