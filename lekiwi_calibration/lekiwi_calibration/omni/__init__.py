"""Omni-Wheel Base Kinematic Calibration Sub-module."""

from lekiwi_calibration.omni.kinematics_calib import (
    compute_robot_radius_calib,
    compute_square_umbmark,
    compute_wheel_radius_calib,
    normalize_angle,
)

try:
    from lekiwi_calibration.omni.omni_base_calibrator_node import (
        OmniBaseCalibratorNode,
        main,
    )
except ImportError:
    OmniBaseCalibratorNode = None
    main = None

__all__ = [
    "OmniBaseCalibratorNode",
    "compute_wheel_radius_calib",
    "compute_robot_radius_calib",
    "compute_square_umbmark",
    "normalize_angle",
    "main",
]
