# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

# Omnidirectional base wheel telemetry: [0]=Left, [1]=Back, [2]=Right
std_msgs/Header header
uint8[3] id                     # Hardware servo IDs
int32[3] raw_position           # Raw 12-bit magnetic encoder ticks (0..4095)
int32[3] raw_velocity           # Raw velocity register value
int32[3] raw_load               # Raw torque load register value
float64[3] position_rad         # Calibrated joint position [rad]
float64[3] velocity_rad_s       # Calibrated joint angular velocity [rad/s]
float32[3] current_ma           # Motor phase current [mA]
float32[3] voltage_v            # Power rail voltage [V]
uint8[3] temperature_c          # Motor temperature [deg C]
uint8[3] status                 # Operating status register bitmask
bool[3] moving                  # Physical movement indicator
uint8[3] servo_error            # Hardware alarm error bitmask
bool[3] online                  # Serial bus heartbeat status
