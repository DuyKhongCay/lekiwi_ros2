# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

from packaging.version import Version
import cv2
import cv2.aruco as aruco
import numpy as np

_CV_NEW_API = Version(cv2.__version__) >= Version("4.8.0")

DICT_MAP = {
    "DICT_4X4_50": aruco.DICT_4X4_50,
    "DICT_4X4_100": aruco.DICT_4X4_100,
    "DICT_4X4_250": aruco.DICT_4X4_250,
    "DICT_5X5_50": aruco.DICT_5X5_50,
    "DICT_5X5_100": aruco.DICT_5X5_100,
    "DICT_5X5_250": aruco.DICT_5X5_250,
    "DICT_6X6_250": aruco.DICT_6X6_250,
}


class CharucoDetectorHelper:
    def __init__(
        self,
        squares_x=3,
        squares_y=4,
        square_len_m=0.006,
        marker_len_m=0.0045,
        dict_name="DICT_4X4_50",
        min_markers=1,
    ):
        self.squares_x = squares_x
        self.squares_y = squares_y
        self.square_len_m = square_len_m
        self.marker_len_m = marker_len_m
        self.min_markers = min_markers

        dict_type = DICT_MAP.get(dict_name, aruco.DICT_4X4_50)
        self.aruco_dict = aruco.getPredefinedDictionary(dict_type)

        # In OpenCV < 4.8, minMarkers in interpolateCornersCharuco is the number of markers around a corner (must be 0, 1, or 2)
        self.legacy_min_markers = (
            max(0, min(int(min_markers), 2)) if not _CV_NEW_API else int(min_markers)
        )

        if _CV_NEW_API:
            self.board = aruco.CharucoBoard(
                (squares_x, squares_y), square_len_m, marker_len_m, self.aruco_dict
            )
            self.det_params = aruco.DetectorParameters()
            self.ch_params = aruco.CharucoParameters()
            self.ch_params.minMarkers = int(min_markers)
            self.ch_params.tryRefineMarkers = False
            self.detector = aruco.CharucoDetector(
                self.board, self.ch_params, self.det_params
            )
        else:
            self.board = aruco.CharucoBoard_create(
                squares_x, squares_y, square_len_m, marker_len_m, self.aruco_dict
            )
            self.det_params = aruco.DetectorParameters_create()
            self.detector = None

    def detect_and_estimate_pose(self, bgr_img, K, D, max_reproj_px=3.0):
        """
        Detects ChArUco board and calculates camera-to-board pose.
        Returns:
            (annotated_img, rvec, tvec, reproj_err, num_corners)
        """
        if bgr_img is None or K is None or D is None:
            return bgr_img, None, None, None, 0

        gray = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
        annotated = bgr_img.copy()

        char_corners = None
        char_ids = None

        if _CV_NEW_API:
            char_corners, char_ids, _, _ = self.detector.detectBoard(gray)
        else:
            corners, ids, _ = aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.det_params
            )
            if ids is not None and len(ids) > 0:
                n, char_corners, char_ids = aruco.interpolateCornersCharuco(
                    corners, ids, gray, self.board, minMarkers=self.legacy_min_markers
                )

        if char_corners is None or char_ids is None or len(char_corners) < 4:
            return annotated, None, None, None, 0

        # Draw corners
        aruco.drawDetectedCornersCharuco(annotated, char_corners, char_ids, (0, 255, 0))

        # Solve PnP
        rvec = None
        tvec = None
        reproj_err = None

        if _CV_NEW_API:
            obj_pts, img_pts = self.board.matchImagePoints(char_corners, char_ids)
            if obj_pts is not None and len(obj_pts) >= 4:
                ok, rvec, tvec = cv2.solvePnP(
                    obj_pts, img_pts, K, D, flags=cv2.SOLVEPNP_ITERATIVE
                )
                if ok:
                    proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, D)
                    reproj_err = float(
                        np.linalg.norm(
                            proj.reshape(-1, 2) - img_pts.reshape(-1, 2), axis=1
                        ).mean()
                    )
        else:
            ok, rvec, tvec = aruco.estimatePoseCharucoBoard(
                char_corners, char_ids, self.board, K, D, None, None
            )
            if ok:
                obj_pts = self.board.chessboardCorners[char_ids.flatten()]
                proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, D)
                reproj_err = float(
                    np.linalg.norm(
                        proj.reshape(-1, 2) - char_corners.reshape(-1, 2), axis=1
                    ).mean()
                )

        if rvec is not None and tvec is not None and reproj_err is not None:
            if reproj_err <= max_reproj_px:
                # Draw 3D coordinate axis on the board
                axis_length = self.square_len_m * 2.0
                cv2.drawFrameAxes(annotated, K, D, rvec, tvec, axis_length, 2)
                return annotated, rvec, tvec, reproj_err, len(char_corners)
            else:
                return annotated, None, None, reproj_err, len(char_corners)

        return annotated, None, None, None, len(char_corners)
