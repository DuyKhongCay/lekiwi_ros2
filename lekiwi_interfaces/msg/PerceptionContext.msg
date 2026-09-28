# Copyright 2026 LeKiwi Labs
# Perception Context for dynamic vision, Hailo NPU gating, and robot arm servicing

uint8 IDLE_STANDBY             = 0  # All GStreamer valves closed (0% CPU/NPU)
uint8 TF_TRACKING_AND_NAV      = 1  # stereo_left OPEN, Chessboard AprilTag PnP OPEN, Hailo NPU OFF
uint8 BOARD_STATE_SCAN         = 2  # stereo_left OPEN, Chessboard PnP OPEN, Hailo FEN inference OPEN
uint8 MANIPULATION_ACTOR       = 3  # usb_wrist OPEN (LeRobot VLA policy), Hailo NPU OFF
uint8 POST_MOVE_VERIFY         = 4  # stereo_left OPEN, Hailo 1-shot verify piece placement
uint8 CALIBRATION_STREAM       = 5  # Raw uncompressed stream for hand-eye / camera calibration

uint8 value
