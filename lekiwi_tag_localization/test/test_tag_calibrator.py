#!/usr/bin/env python3
"""
Unit test for ChessboardTagCalibSolver using synthetic multi-view data with noise.
"""

import math
import sys
import os
import unittest
import numpy as np
import cv2

# Add scripts directory to path for import
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from calibrate_chessboard_tags import ChessboardTagCalibSolver


class TestChessboardTagCalibSolver(unittest.TestCase):
    # Validates Bundle Adjustment solver accuracy on synthetic 3D tag setups.

    def setUp(self):
        # Configures ground truth tag parameters and synthetic camera intrinsics.
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
        # Generates ground truth 3D corners for each tag.
        h = self.tag_sz / 2.0
        local_pts = np.array(
            [[-h, -h, 0.0], [h, -h, 0.0], [h, h, 0.0], [-h, h, 0.0]], dtype=np.float64
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
        # Verifies that solver recovers true tag coordinates within sub-millimeter error.
        gt_3d = self._generate_gt_corners()

        # Generate 15 synthetic camera viewpoints around board
        np.random.seed(42)
        frames_dets = []

        for i in range(15):
            # Camera at z ~ 0.5 - 0.7m, looking towards board center (0.19, 0.19, 0.0)
            cx = 0.19 + np.random.uniform(-0.15, 0.15)
            cy = 0.19 + np.random.uniform(-0.15, 0.15)
            cz = np.random.uniform(0.45, 0.70)

            # Camera pointing roughly downwards with slight tilt
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

        z_priors = [0.0, 0.0015, 0.0008, 0.0010]
        solver = ChessboardTagCalibSolver(
            tag_ids=(1, 4, 3, 6),
            tag_sz=self.tag_sz,
            nominal_dist=0.38,
            z_priors=z_priors,
        )

        res = solver.solve(frames_dets, self.cam_mat, self.dist_coeffs)

        self.assertLess(
            res["rms_err"], 1.0, "RMS reprojection error should be under 1.0 px"
        )

        # Check recovered tag 4 (H1) position: ground truth x=0.3842, y=0.0012
        recovered_h1 = res["tags"][4]
        self.assertAlmostEqual(
            recovered_h1["x"], 0.3842, delta=0.002, msg="H1 X recovered within 2mm"
        )
        self.assertAlmostEqual(
            recovered_h1["y"], 0.0012, delta=0.002, msg="H1 Y recovered within 2mm"
        )

        # Check recovered tag 3 (H8) position: ground truth x=0.3838, y=0.3845
        recovered_h8 = res["tags"][3]
        self.assertAlmostEqual(
            recovered_h8["x"], 0.3838, delta=0.002, msg="H8 X recovered within 2mm"
        )
        self.assertAlmostEqual(
            recovered_h8["y"], 0.3845, delta=0.002, msg="H8 Y recovered within 2mm"
        )


if __name__ == "__main__":
    unittest.main()
