# Copyright 2026 LeKiwi Labs

uint8 PIPELINE_STOPPED=0
uint8 PIPELINE_STARTING=1
uint8 PIPELINE_RUNNING=2
uint8 PIPELINE_STOPPING=3
uint8 PIPELINE_ERROR=4

std_msgs/Header header
uint8 pipeline_state
string last_error
float32 fps

