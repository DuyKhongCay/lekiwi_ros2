# Copyright 2026 LeKiwi Labs

uint8 PIPELINE_STOPPED=0
uint8 PIPELINE_STARTING=1
uint8 PIPELINE_RUNNING=2
uint8 PIPELINE_STOPPING=3
uint8 PIPELINE_ERROR=4
std_msgs/Header header
CameraMode effective_mode
CameraMode prev_mode
uint8 hailo_pipe_state
uint8 lerobot_pipe_state
string last_error

float32 hailo_fps
float32 stereo_right_fps
float32 usb_wrist_fps
float32 usb_side_fps
float32 hailo_publish_lat_ms
float32 stereo_right_publish_lat_ms
float32 usb_wrist_publish_lat_ms
float32 usb_side_publish_lat_ms
float32 lerobot_skew_p50_ms
float32 lerobot_skew_p95_ms
uint64 lerobot_triplet_match_count
uint64 lerobot_triplet_discarded_frame_count
# -1.0 means no successful mode transition has completed yet.
float32 set_mode_delay_ms
