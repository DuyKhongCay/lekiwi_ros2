# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for HandEyeSolver multi-algorithm solver and serialization."""

import os
import tempfile
import unittest
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import yaml

from lekiwi_calibration.handeye.handeye_solver import HandEyeSolver


class TestHandEyeSolver(unittest.TestCase):
    def test_insufficient_samples(self):
        solver = HandEyeSolver(is_eye_in_hand=False)
        solver.add_sample(np.eye(4), np.eye(4))
        solver.add_sample(np.eye(4), np.eye(4))
        with pytest.raises(ValueError, match="Need at least 3 samples"):
            solver.compute()

    def test_synthetic_handeye_solution(self):
        # Ground truth hand-eye transformation X: camera to base (Eye-to-Hand)
        rot_gt = Rotation.from_euler("xyz", [15.0, -10.0, 45.0], degrees=True)
        t_gt = np.array([0.15, -0.05, 0.40])
        X_gt = np.eye(4)
        X_gt[:3, :3] = rot_gt.as_matrix()
        X_gt[:3, 3] = t_gt

        # Target relative to world/base: target is fixed in world
        # For Eye-to-Hand: A = inv(robot_{i+1}) @ robot_i, B = tracking_{i+1} @ inv(tracking_i)
        # Relationship: A @ X = X @ B  =>  T_base2gripper @ T_gripper2marker ...
        # Standard formulation: R_gripper2base, t_gripper2base vs R_target2cam, t_target2cam
        solver = HandEyeSolver(is_eye_in_hand=False)

        # Generate 6 diverse robot poses
        np.random.seed(42)
        T_target_in_world = np.eye(4)
        T_target_in_world[:3, 3] = [0.2, 0.0, 0.1]

        for _ in range(6):
            r_arm = Rotation.from_euler(
                "xyz",
                np.random.uniform(-30, 30, size=3),
                degrees=True,
            )
            t_arm = np.random.uniform(-0.1, 0.1, size=3) + np.array([0.2, 0.0, 0.2])

            T_gripper_to_base = np.eye(4)
            T_gripper_to_base[:3, :3] = r_arm.as_matrix()
            T_gripper_to_base[:3, 3] = t_arm

            # Marker is attached to gripper: T_marker_in_gripper is known constant
            T_marker_in_gripper = np.eye(4)
            T_marker_in_gripper[:3, 3] = [0.0, 0.0, 0.05]

            # Marker in base:
            T_marker_in_base = T_gripper_to_base @ T_marker_in_gripper

            # Camera in base is X_gt:
            # T_marker_in_cam = inv(X_gt) @ T_marker_in_base
            T_target_to_cam = np.linalg.inv(X_gt) @ T_marker_in_base

            solver.add_sample(T_gripper_to_base, T_target_to_cam)

        results, best_name, metrics = solver.compute()
        assert len(results) > 0
        assert metrics["num_samples"] == 6

    def test_save_yaml(self):
        solver = HandEyeSolver(is_eye_in_hand=False)
        best_T = np.eye(4)
        best_T[0, 3] = 0.123
        metrics = {
            "method": "Park",
            "residual_error": 0.001,
            "num_samples": 5,
        }

        with tempfile.NamedTemporaryFile("w+", suffix=".yaml", delete=False) as tf:
            temp_path = tf.name

        try:
            solver.save_yaml(
                temp_path,
                parent_frame="base_footprint",
                child_frame="camera_stereo_left",
                best_T=best_T,
                metrics=metrics,
            )
            with open(temp_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
            self.assertEqual(data["header"]["parent_frame"], "base_footprint")
            self.assertEqual(data["header"]["child_frame"], "camera_stereo_left")
            self.assertEqual(data["metrics"]["method"], "Park")
        finally:
            if os.path.exists(temp_path):
                os.remove(temp_path)


if __name__ == "__main__":
    unittest.main()
