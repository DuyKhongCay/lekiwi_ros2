"""Hand-Eye Calibration Sub-module."""

from lekiwi_calibration.handeye.charuco_detector import CharucoDetectorHelper
from lekiwi_calibration.handeye.handeye_solver import HandEyeSolver

try:
    from lekiwi_calibration.handeye.generate_charuco import (
        main as generate_charuco_main,
    )
    from lekiwi_calibration.handeye.handeye_calibration_node import (
        HandEyeCalibrationNode,
        main as handeye_calibration_main,
    )
    from lekiwi_calibration.handeye.robot_controller_client import RobotArmManager
except ImportError:
    HandEyeCalibrationNode = None
    RobotArmManager = None
    generate_charuco_main = None
    handeye_calibration_main = None

__all__ = [
    "CharucoDetectorHelper",
    "HandEyeCalibrationNode",
    "HandEyeSolver",
    "RobotArmManager",
    "handeye_calibration_main",
    "generate_charuco_main",
]
