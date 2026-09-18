# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests validating concurrency safety and deadlock elimination in ChessboardTagCalibratorNode."""

import sys
import threading
import time
import unittest
from unittest.mock import MagicMock, patch

# Provide mock ROS 2 runtime in test environments lacking native rclpy installation
if "rclpy" not in sys.modules:
    try:
        import rclpy  # noqa: F401
    except ImportError:
        mock_rclpy = MagicMock()
        mock_node_mod = MagicMock()
        mock_qos_mod = MagicMock()

        class _MockNode:
            """Minimal Mock Node for testing without ROS 2 binary middleware."""

            def __init__(self, node_name: str = "test_node", *args, **kwargs):
                self._node_name = node_name
                self._logger = MagicMock()
                self._params = {}

            def get_name(self):
                return self._node_name

            def get_logger(self):
                return self._logger

            def declare_parameter(self, name, val):
                class _Param:
                    def __init__(self, v):
                        self.value = v

                self._params[name] = _Param(val)

            def get_parameter(self, name):
                class _Param:
                    def __init__(self, v):
                        self.value = v

                return self._params.get(name, _Param(None))

            def create_subscription(self, *args, **kwargs):
                return MagicMock()

            def create_timer(self, *args, **kwargs):
                return MagicMock()

            def destroy_node(self):
                pass

        mock_node_mod.Node = _MockNode
        mock_qos_mod.QoSProfile = MagicMock
        mock_qos_mod.ReliabilityPolicy = MagicMock()
        mock_qos_mod.HistoryPolicy = MagicMock()

        sys.modules["rclpy"] = mock_rclpy
        sys.modules["rclpy.node"] = mock_node_mod
        sys.modules["rclpy.qos"] = mock_qos_mod

        for mod_name in [
            "sensor_msgs",
            "sensor_msgs.msg",
            "apriltag_msgs",
            "apriltag_msgs.msg",
            "cv_bridge",
        ]:
            if mod_name not in sys.modules:
                sys.modules[mod_name] = MagicMock()

import numpy as np

from lekiwi_calibration.chessboard.calibrator_node import (
    ChessboardTagCalibratorNode,
    format_calibration_report,
)
from lekiwi_calibration.chessboard.visualizer import CalibState


class TestChessboardCalibratorNodeDeadlockSafety(unittest.TestCase):
    """Poka-yoke test suite verifying zero-deadlock guarantees and re-entrancy safety."""

    def setUp(self):
        """Constructs calibrator node with mocked ROS subscriptions and parameters."""
        self.node = ChessboardTagCalibratorNode()
        self.mock_logger = MagicMock()
        self.node.get_logger = lambda: self.mock_logger

        # Provide dummy camera intrinsics
        self.node.cam_mat = np.eye(3, dtype=np.float64)
        self.node.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

    def _run_with_timeout(self, target_func, *args, timeout_sec=3.0, **kwargs):
        """Helper to run a function in a thread with a hard timeout to catch deadlocks."""
        result = [None]
        exception = [None]

        def runner():
            try:
                result[0] = target_func(*args, **kwargs)
            except Exception as e:
                exception[0] = e

        t = threading.Thread(target=runner, daemon=True)
        t.start()
        t.join(timeout=timeout_sec)
        if t.is_alive():
            self.fail(
                f"Deadlock detected! Function {target_func.__name__} did not finish within {timeout_sec}s"
            )
        if exception[0] is not None:
            raise exception[0]
        return result[0]

    def test_format_calibration_report_pure_function(self):
        """Verifies format_calibration_report produces expected text without side-effects."""
        fake_res = {
            "num_frames": 10,
            "mean_err": 0.05,
            "rms_err": 0.08,
            "tags": {
                0: {"name": "A1", "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
                1: {"name": "H1", "x": 0.38, "y": 0.0, "z": 0.0, "yaw": 0.0},
                2: {"name": "H8", "x": 0.38, "y": 0.38, "z": 0.0, "yaw": 0.0},
                3: {"name": "A8", "x": 0.0, "y": 0.38, "z": 0.0, "yaw": 0.0},
            },
        }
        report = format_calibration_report(fake_res, [0, 1, 2, 3])
        self.assertIn("CALIBRATION RESULTS REPORT (PLANAR)", report)
        self.assertIn("A1", report)
        self.assertIn("H8", report)
        self.assertIn("Corner Distances:", report)

    def test_data_lock_is_reentrant(self):
        """Verifies _data_lock is an RLock and permits recursive acquisition."""
        self.assertTrue(
            isinstance(self.node._data_lock, type(threading.RLock())),
            "Node _data_lock must be an RLock to prevent recursive self-deadlocks.",
        )
        acquired_twice = False
        with self.node._data_lock:
            with self.node._data_lock:
                acquired_twice = True
        self.assertTrue(acquired_twice, "Failed to re-entrantly acquire _data_lock.")

    def test_capture_current_frame_success_no_deadlock(self):
        """Verifies capture_current_frame does not deadlock when saving a frame."""
        corners = np.array([[10, 10], [20, 10], [20, 20], [10, 20]], dtype=np.float64)
        self.node.latest_dets = {0: corners.copy(), 1: corners.copy()}

        success = self._run_with_timeout(self.node.capture_current_frame)
        self.assertTrue(success)
        self.assertEqual(len(self.node.captured_frames), 1)
        self.assertIn("Captured #1", self.node.notification.message)

    def test_capture_current_frame_failure_no_deadlock(self):
        """Verifies capture_current_frame does not deadlock when tag count is insufficient."""
        self.node.latest_dets = {}

        success = self._run_with_timeout(self.node.capture_current_frame)
        self.assertFalse(success)
        self.assertEqual(len(self.node.captured_frames), 0)
        self.assertIn("Failed: need >=", self.node.notification.message)

    def test_auto_cap_completion_no_deadlock(self):
        """Verifies auto-capture threshold completion triggers notifications without deadlock."""
        self.node.auto_cap_enabled = True
        self.node.target_caps_cnt = 2
        self.node.captured_frames = [{}, {}]
        self.node.last_auto_cap_time = 0.0

        mock_msg = MagicMock()
        mock_msg.detections = []

        self._run_with_timeout(self.node._tag_dets_cb, mock_msg)
        self.assertFalse(self.node.auto_cap_enabled)
        self.assertIn("Auto-cap done", self.node.notification.message)

    def test_run_calibration_async_prerequisites_failures_no_deadlock(self):
        """Verifies run_calibration_async preconditions exit safely without deadlocking."""
        # Case 1: Less than 3 frames captured
        self.node.captured_frames = [{}]
        self._run_with_timeout(self.node.run_calibration_async)
        self.assertNotEqual(self.node.calib_state, CalibState.OPTIMIZING)
        self.assertIn("Need >= 3 captured frames", self.node.notification.message)

        # Case 2: Missing camera intrinsics
        self.node.captured_frames = [{}, {}, {}]
        self.node.cam_mat = None
        self._run_with_timeout(self.node.run_calibration_async)
        self.assertNotEqual(self.node.calib_state, CalibState.OPTIMIZING)
        self.assertIn("Camera intrinsics missing", self.node.notification.message)

        # Case 3: Already optimizing
        self.node.cam_mat = np.eye(3, dtype=np.float64)
        self.node.calib_state = CalibState.OPTIMIZING
        self._run_with_timeout(self.node.run_calibration_async)
        self.assertIn(
            "Optimization already in progress", self.node.notification.message
        )

    def test_save_calibration_without_result_no_deadlock(self):
        """Verifies save_calibration handles uncalibrated state without deadlocking."""
        self.node.calib_res = None
        self._run_with_timeout(self.node.save_calibration)
        self.assertIn("Calibrate before saving", self.node.notification.message)

    def test_save_calibration_success_no_deadlock(self):
        """Verifies save_calibration executes cleanly with valid calibration result."""
        self.node.calib_res = {
            "num_frames": 3,
            "mean_err": 0.1,
            "rms_err": 0.15,
            "tags": {
                0: {"name": "A1", "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
                1: {"name": "H1", "x": 0.38, "y": 0.0, "z": 0.0, "yaw": 0.0},
                2: {"name": "H8", "x": 0.38, "y": 0.38, "z": 0.0, "yaw": 0.0},
                3: {"name": "A8", "x": 0.0, "y": 0.38, "z": 0.0, "yaw": 0.0},
            },
        }
        with patch(
            "lekiwi_calibration.chessboard.calibrator_node.save_to_chessboard_yaml"
        ) as mock_save:
            self._run_with_timeout(self.node.save_calibration)
            mock_save.assert_called_once()
            self.assertIn("Saved calibration to YAML", self.node.notification.message)

    def test_reset_invalidates_in_flight_worker(self):
        """Poka-yoke test: verify epoch invalidation prevents stale worker results from corrupting state."""
        self.node.captured_frames = [{}, {}, {}]
        self.node.calib_state = CalibState.OPTIMIZING

        self.node._solve_epoch = 1
        worker_epoch = 1

        # User presses reset ('r')
        self._run_with_timeout(self.node._reset_captured_samples)
        self.assertEqual(self.node.calib_state, CalibState.IDLE)
        self.assertEqual(len(self.node.captured_frames), 0)
        self.assertGreater(self.node._solve_epoch, worker_epoch)

        # Worker finishes with old worker_epoch
        fake_result = {"rms_err": 0.05, "tags": {}, "num_frames": 3, "mean_err": 0.04}
        with patch.object(self.node.solver, "solve", return_value=fake_result):
            self._run_with_timeout(
                self.node._solve_worker,
                frames=[],
                cam_mat=self.node.cam_mat,
                dist_coeffs=self.node.dist_coeffs,
                epoch=worker_epoch,
            )

        # calib_res should STILL be None and state should STILL be IDLE
        self.assertIsNone(
            self.node.calib_res,
            "Outdated worker should NOT overwrite calib_res after reset.",
        )
        self.assertEqual(
            self.node.calib_state,
            CalibState.IDLE,
            "State should remain IDLE after reset.",
        )

    def test_concurrent_threads_stress(self):
        """Stress-tests the node under high multi-threaded contention to ensure zero deadlocks."""
        corners = np.array([[10, 10], [20, 10], [20, 20], [10, 20]], dtype=np.float64)
        stop_event = threading.Event()
        errors = []

        def worker_capture():
            while not stop_event.is_set():
                try:
                    self.node.latest_dets = {0: corners.copy(), 1: corners.copy()}
                    self.node.capture_current_frame()
                    time.sleep(0.001)
                except Exception as e:
                    errors.append(e)

        def worker_notify():
            while not stop_event.is_set():
                try:
                    self.node._set_notification("Concurrent notification", (255, 0, 0))
                    time.sleep(0.001)
                except Exception as e:
                    errors.append(e)

        def worker_reset():
            while not stop_event.is_set():
                try:
                    self.node._reset_captured_samples()
                    time.sleep(0.003)
                except Exception as e:
                    errors.append(e)

        def worker_toggle():
            while not stop_event.is_set():
                try:
                    self.node._toggle_auto_capture()
                    time.sleep(0.002)
                except Exception as e:
                    errors.append(e)

        threads = [
            threading.Thread(target=worker_capture),
            threading.Thread(target=worker_notify),
            threading.Thread(target=worker_reset),
            threading.Thread(target=worker_toggle),
        ]

        for t in threads:
            t.start()

        time.sleep(0.5)
        stop_event.set()

        for t in threads:
            t.join(timeout=2.0)
            self.assertFalse(
                t.is_alive(), "A worker thread deadlocked under concurrent stress!"
            )

        self.assertEqual(
            len(errors), 0, f"Errors occurred during concurrent stress: {errors}"
        )


if __name__ == "__main__":
    unittest.main()
