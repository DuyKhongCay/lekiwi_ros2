# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

# Vision pipeline gating & Hailo-8 NPU context
uint8 IDLE_STANDBY             = 0  # GStreamer valves closed (0% CPU/NPU)
uint8 TF_TRACKING_AND_NAV      = 1  # stereo_left + AprilTag PnP, Hailo OFF
uint8 BOARD_STATE_SCAN         = 2  # stereo_left + chessboard PnP + Hailo FEN ON
uint8 MANIPULATION_ACTOR       = 3  # usb_wrist active for LeRobot VLA, Hailo OFF
uint8 POST_MOVE_VERIFY         = 4  # stereo_left 1-shot piece placement verify
uint8 CALIBRATION_STREAM       = 5  # Raw stream for intrinsic/hand-eye calib

uint8 value                         # Active perception context
