# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

# Operating states
uint8 UNKNOWN=0
uint8 INACTIVE=1
uint8 ACTIVE=2
uint8 ERROR=3

std_msgs/Header header
uint8 state                     # Current driver lifecycle state
bool bus_connected              # Serial bus link state (/dev/lekiwi_serial)
bool drive_enabled              # Motor drive torque enable status
bool watchdog_expired           # True if cmd_vel timed out (> 100 ms)
uint32 serial_error_count       # Cumulative CRC or framing error count
uint32 consecutive_error_count  # Consecutive failed serial transactions
uint32 retry_count              # Total bus recovery retry attempts
float32 command_age_sec         # Elapsed time since last valid cmd_vel [s]
string last_error               # Diagnostic description of last error
