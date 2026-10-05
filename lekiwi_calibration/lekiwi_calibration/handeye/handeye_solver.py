# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
import os


class HandEyeSolver:
    """Computes Hand-Eye Calibration matrix using OpenCV calibrateHandEye."""

    METHODS = {
        "Tsai-Lenz": cv2.CALIB_HAND_EYE_TSAI,
        "Park": cv2.CALIB_HAND_EYE_PARK,
        "Horaud": cv2.CALIB_HAND_EYE_HORAUD,
        "Andreff": cv2.CALIB_HAND_EYE_ANDREFF,
        "Daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }

    def __init__(self, is_eye_in_hand: bool = False):
        """Initialize HandEyeSolver with designated configuration mode.

        Args:
            is_eye_in_hand: False for Eye-to-Hand (camera on base/world, target on gripper);
                True for Eye-in-Hand (camera on gripper, target on base/world).
        """
        self.is_eye_in_hand = is_eye_in_hand
        self.samples = []  # List of sample dictionaries containing "robot" and "tracking" SE(3) matrices

    def add_sample(self, robot_T: np.ndarray, tracking_T: np.ndarray) -> int:
        """Add a synchronized robot arm and visual tracking pose pair.

        Args:
            robot_T: 4x4 transform matrix from gripper to base (Eye-to-Hand) or
                base to gripper (Eye-in-Hand).
            tracking_T: 4x4 transform matrix from camera to target board (T_target2cam).

        Returns:
            Total count of accumulated samples.
        """
        self.samples.append({
            "robot": robot_T.copy(),
            "tracking": tracking_T.copy()
        })
        return len(self.samples)

    def remove_last_sample(self) -> int:
        """Remove the most recently recorded sample pair from buffer."""
        if self.samples:
            self.samples.pop()
        return len(self.samples)

    def clear_samples(self) -> None:
        """Reset and discard all accumulated sample pairs."""
        self.samples.clear()

    def compute(self) -> tuple[dict, str, dict]:
        """Solve Hand-Eye calibration across all supported OpenCV algorithms and select best result.

        Evaluates relative motion consistency ||A*X - X*B|| for each method:
        Tsai-Lenz, Park, Horaud, Andreff, and Daniilidis.

        Returns:
            Tuple of (results_dict, best_method_name, best_metrics_dict).

        Raises:
            ValueError: If fewer than 3 samples have been collected.
            RuntimeError: If all candidate algorithms fail to converge.
        """
        n = len(self.samples)
        if n < 3:
            raise ValueError(f"Need at least 3 samples to solve hand-eye calibration (currently have {n})")

        R_robot = []
        t_robot = []
        R_tracking = []
        t_tracking = []

        for s in self.samples:
            R_robot.append(s["robot"][:3, :3])
            t_robot.append(s["robot"][:3, 3].reshape(3, 1))
            R_tracking.append(s["tracking"][:3, :3])
            t_tracking.append(s["tracking"][:3, 3].reshape(3, 1))

        results = {}
        residuals = {}

        for name, method in self.METHODS.items():
            try:
                R, t = cv2.calibrateHandEye(
                    R_robot, t_robot,
                    R_tracking, t_tracking,
                    method=method
                )
                T = np.eye(4)
                T[:3, :3] = R
                T[:3, 3] = t.flatten()
                results[name] = T

                # Compute residual error across samples: || A * X - X * B ||
                errs = []
                for i in range(len(self.samples) - 1):
                    # Relative motions
                    A = np.linalg.inv(self.samples[i+1]["robot"]) @ self.samples[i]["robot"]
                    B = self.samples[i+1]["tracking"] @ np.linalg.inv(self.samples[i]["tracking"])
                    diff = A @ T - T @ B
                    errs.append(np.linalg.norm(diff))
                residuals[name] = float(np.mean(errs)) if errs else 0.0

            except Exception as e:
                pass

        if not results:
            raise RuntimeError("All Hand-Eye calibration algorithms failed with current samples.")

        # Pick method with smallest residual error (or Park as fallback)
        best_name = min(residuals, key=residuals.get) if residuals else "Park"
        best_T = results[best_name]

        rot = Rotation.from_matrix(best_T[:3, :3])
        quat = rot.as_quat()  # [x, y, z, w]
        rpy_deg = rot.as_euler("xyz", degrees=True)

        metrics = {
            "method": best_name,
            "translation": best_T[:3, 3].tolist(),
            "rotation_quat_xyzw": quat.tolist(),
            "rotation_rpy_deg": rpy_deg.tolist(),
            "residual_error": residuals.get(best_name, 0.0),
            "num_samples": n,
        }

        return results, best_name, metrics

    def save_yaml(
        self,
        filepath: str,
        parent_frame: str,
        child_frame: str,
        best_T: np.ndarray,
        metrics: dict,
    ) -> None:
        """Serialize optimal Hand-Eye calibration transform and metrics to YAML file.

        Args:
            filepath: Destination file path for YAML configuration.
            parent_frame: Parent coordinate frame ID.
            child_frame: Child coordinate frame ID.
            best_T: Optimal 4x4 SE(3) transformation matrix.
            metrics: Calibration metrics dictionary (residual error, Euler angles, samples count).
        """
        os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
        rot = Rotation.from_matrix(best_T[:3, :3])
        quat = rot.as_quat()  # x, y, z, w

        data = {
            "header": {
                "parent_frame": parent_frame,
                "child_frame": child_frame,
                "is_eye_in_hand": self.is_eye_in_hand,
            },
            "transform": {
                "translation": {
                    "x": float(best_T[0, 3]),
                    "y": float(best_T[1, 3]),
                    "z": float(best_T[2, 3]),
                },
                "rotation": {
                    "x": float(quat[0]),
                    "y": float(quat[1]),
                    "z": float(quat[2]),
                    "w": float(quat[3]),
                },
            },
            "metrics": metrics,
        }

        with open(filepath, "w") as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)
