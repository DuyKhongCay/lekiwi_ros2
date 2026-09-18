"""OpenCV GUI visualizer, HUD overlay, and notification layer for Chessboard Calibration."""

from dataclasses import dataclass
from enum import Enum
import time
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np


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
