# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

# Telemetry is ordered as left, back, right base wheels.
std_msgs/Header header
uint8[3] id
int32[3] raw_position
int32[3] raw_velocity
int32[3] raw_load
float64[3] position_rad
float64[3] velocity_rad_s
float32[3] current_ma
float32[3] voltage_v
uint8[3] temperature_c
uint8[3] status
bool[3] moving
uint8[3] servo_error
bool[3] online
