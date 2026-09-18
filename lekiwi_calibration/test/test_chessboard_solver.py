# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit test for ChessboardTagCalibSolver using synthetic multi-view data with noise."""

import math
import os
import tempfile
import unittest
import cv2
import numpy as np
import yaml

from lekiwi_calibration.chessboard.solver import (
    ChessboardTagCalibSolver,
    save_to_chessboard_yaml,
)


class TestChessboardTagCalibSolver(unittest.TestCase):
    """Validates Bundle Adjustment solver accuracy on synthetic 3D tag setups."""

    def setUp(self):
        self.cam_mat = np.array(
            [[2274.0, 0.0, 1640.0], [0.0, 2287.0, 1232.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

        # Ground truth tag positions (nominal 0.385m + slight offsets, z variations)
        self.gt_tags = {
            1: {"center": [0.0, 0.0, 0.0], "yaw": 0.0},
            4: {"center": [0.3842, 0.0012, 0.0015], "yaw": 0.005},
            3: {"center": [0.3838, 0.3845, 0.0008], "yaw": -0.003},
            6: {"center": [0.0005, 0.3840, 0.0010], "yaw": 0.004},
        }
        self.tag_sz = 0.02

    def _generate_gt_corners(self):
        h = self.tag_sz / 2.0
        local_pts = np.array(
            [[h, -h, 0.0], [-h, -h, 0.0], [-h, h, 0.0], [h, h, 0.0]], dtype=np.float64
        )

        gt_3d = {}
        for tid, data in self.gt_tags.items():
            yaw = data["yaw"]
            c = data["center"]
            cos_y = math.cos(yaw)
            sin_y = math.sin(yaw)
            rot_mat = np.array(
                [[cos_y, -sin_y, 0.0], [sin_y, cos_y, 0.0], [0.0, 0.0, 1.0]]
            )
            corners = (rot_mat @ local_pts.T).T + np.array(c)
            gt_3d[tid] = corners
        return gt_3d

    def test_bundle_adjustment_accuracy(self):
        gt_3d = self._generate_gt_corners()

        # Generate 15 synthetic camera viewpoints around board
        np.random.seed(42)
        frames_dets = []

        for _ in range(15):
            cx = 0.19 + np.random.uniform(-0.15, 0.15)
            cy = 0.19 + np.random.uniform(-0.15, 0.15)
            cz = np.random.uniform(0.45, 0.70)

            rvec = np.array(
                [
                    np.random.uniform(2.8, 3.14),
                    np.random.uniform(-0.2, 0.2),
                    np.random.uniform(-0.2, 0.2),
                ],
                dtype=np.float64,
            ).reshape((3, 1))

            tvec = np.array([cx, cy, cz], dtype=np.float64).reshape((3, 1))

            frame_det = {}
            for tid, pts_3d in gt_3d.items():
                proj_2d, _ = cv2.projectPoints(
                    pts_3d, rvec, tvec, self.cam_mat, self.dist_coeffs
                )
                proj_2d = proj_2d.reshape(-1, 2)
                # Add Gaussian pixel noise (sigma = 0.2 px)
                noise = np.random.normal(0.0, 0.2, proj_2d.shape)
                frame_det[tid] = proj_2d + noise

            frames_dets.append(frame_det)

        solver = ChessboardTagCalibSolver(
            tag_ids=(1, 4, 3, 6),
            tag_sz=self.tag_sz,
            nominal_dist=0.38,
            z_height=0.0,
        )

        res = solver.solve(frames_dets, self.cam_mat, self.dist_coeffs)

        self.assertLess(
            res["rms_err"], 1.0, "RMS reprojection error should be under 1.0 px"
        )

        tags = res["tags"]

        # 1. Verify geometric center of the 4 tags is exactly at (0.0, 0.0)
        mean_x = float(np.mean([tags[tid]["x"] for tid in (1, 4, 3, 6)]))
        mean_y = float(np.mean([tags[tid]["y"] for tid in (1, 4, 3, 6)]))
        self.assertAlmostEqual(mean_x, 0.0, delta=0.001, msg="Center X should be 0.0")
        self.assertAlmostEqual(mean_y, 0.0, delta=0.001, msg="Center Y should be 0.0")

        # 2. Verify recovered centered coordinates match ground truth minus center offset
        # Ground truth center: cx = 0.1921m, cy = 0.1924m
        recovered_a1 = tags[1]
        self.assertAlmostEqual(
            recovered_a1["x"], -0.1921, delta=0.002, msg="A1 X recovered around -0.192m"
        )
        self.assertAlmostEqual(
            recovered_a1["y"], -0.1924, delta=0.002, msg="A1 Y recovered around -0.192m"
        )

        recovered_h1 = tags[4]
        self.assertAlmostEqual(
            recovered_h1["x"], 0.1921, delta=0.002, msg="H1 X recovered around +0.192m"
        )
        self.assertAlmostEqual(
            recovered_h1["y"], -0.1912, delta=0.002, msg="H1 Y recovered around -0.191m"
        )

        recovered_h8 = tags[3]
        self.assertAlmostEqual(
            recovered_h8["x"], 0.1917, delta=0.002, msg="H8 X recovered around +0.192m"
        )
        self.assertAlmostEqual(
            recovered_h8["y"], 0.1921, delta=0.002, msg="H8 Y recovered around +0.192m"
        )

        recovered_a8 = tags[6]
        self.assertAlmostEqual(
            recovered_a8["x"], -0.1916, delta=0.002, msg="A8 X recovered around -0.192m"
        )
        self.assertAlmostEqual(
            recovered_a8["y"], 0.1916, delta=0.002, msg="A8 Y recovered around +0.192m"
        )

        # 3. Verify side length distance preservation (~0.384m between corners)
        dist_a1_h1 = math.hypot(
            recovered_h1["x"] - recovered_a1["x"], recovered_h1["y"] - recovered_a1["y"]
        )
        self.assertAlmostEqual(
            dist_a1_h1, 0.3842, delta=0.002, msg="A1-H1 distance preserved"
        )

    def test_save_to_chessboard_yaml(self):
        calib_res = {
            "tags": {
                0: {"name": "A1", "x": -0.195, "y": -0.195, "z": 0.004, "yaw": 0.0},
                1: {"name": "H1", "x": 0.195, "y": -0.195, "z": 0.004, "yaw": 0.0},
                2: {"name": "H8", "x": 0.195, "y": 0.195, "z": 0.004, "yaw": 0.0},
                3: {"name": "A8", "x": -0.195, "y": 0.195, "z": 0.004, "yaw": 0.0},
            }
        }
        with tempfile.NamedTemporaryFile("w+", suffix=".yaml", delete=False) as tf:
            temp_path = tf.name

        try:
            save_to_chessboard_yaml(calib_res, temp_path, tag_ids=[0, 1, 2, 3])
            with open(temp_path, "r", encoding="utf-8") as f:
                saved = yaml.safe_load(f)
            self.assertEqual(saved["ids"], [0, 1, 2, 3])
            self.assertEqual(saved["positions_x"], [-0.195, 0.195, 0.195, -0.195])
            self.assertEqual(saved["positions_y"], [-0.195, -0.195, 0.195, 0.195])
        finally:
            if os.path.exists(temp_path):
                os.remove(temp_path)


if __name__ == "__main__":
    unittest.main()
