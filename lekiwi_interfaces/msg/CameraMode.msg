# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

# Camera streaming & pipeline modes
uint8 STANDBY=0                # Idle state, pipelines & NPU off (0% CPU/NPU)
uint8 NAVIGATING=1             # Forward stereo navigation (stereo_left)
uint8 CHESS_THINKING=2         # Chess board perception & Hailo FEN inference
uint8 MANIPULATION_LEROBOT=3   # Wrist/side cameras for LeRobot VLA manipulation

uint8 value                    # Current camera operating mode
