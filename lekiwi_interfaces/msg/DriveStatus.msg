# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

uint8 UNKNOWN=0
uint8 INACTIVE=1
uint8 ACTIVE=2
uint8 ERROR=3

std_msgs/Header header
uint8 state
bool bus_connected
bool drive_enabled
bool watchdog_expired
uint32 serial_error_count
uint32 consecutive_error_count
uint32 retry_count
float32 command_age_sec
string last_error
