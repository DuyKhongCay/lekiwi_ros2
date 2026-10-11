# `teleop_zhongli_servo_hw`

High-performance real-time driver, interactive midpoint calibration CLI, kinematic remapping engine, and ROS 2 publisher for the **uArm / Zhongli Serial Bus Leader Arm (1 Mbps)** used in teleoperation and demonstration recording for the LeKiwi robot.

---

## 🚀 Key Architectural Improvements (1 Mbps & Real-time I/O)

- **1 Mbps High-Speed Serial Bus**: Operates strictly at 1,000,000 bps with 300 µs inter-command spacing. Full 7-servo query cycle takes $< 3.5\text{ ms}$, effortlessly sustaining 50 Hz teleoperation without bus saturation.
- **Asynchronous Background Worker (`ZhongliBusWorker`)**: Serial I/O is completely decoupled from the ROS 2 executor thread. Background worker runs continuous drift-free polling via `std::this_thread::sleep_until`.
- **Wait-Free Real-time Buffer (`realtime_tools::RealtimeBuffer`)**: ROS 2 publishing loop retrieves fresh kinematic states in $< 50\text{ ns}$ without ever blocking on UART transactions.
- **Interleaved Telemetry & Diagnostics**: Servo voltage and temperature are sampled round-robin inside the worker loop, eliminating timer contention and serial thread collisions.
- **Zero-Allocation Hot Path**: Scratch buffers and message packets are pre-allocated during initialization (MISRA C++ Rule 18-0-1 compliance).
- **3-Tier Fault Escalation**:
  - *Tier 1*: Instant input buffer flush upon single-frame drops.
  - *Tier 2*: Telemetry invalidation after 3 consecutive failures ($60\text{ ms}$).
  - *Tier 3*: Non-blocking automatic reconnect in the background if connection is severed ($> 1\text{ s}$).
- **Defensive Packet Parsing**: Robust token-based ASCII parser handling both 3-digit ($< 1000$) and 4-digit PWM ranges ($500 .. 2500$) without brittle string length checks.

---

## 📦 Package Structure

```text
teleop_zhongli_servo_hw/
├── include/teleop_zhongli_servo_hw/
│   ├── follower_dispatcher.hpp       # Dispatches commands to ros2_control controllers
│   ├── kinematic_remapper.hpp        # Translates leader to follower kinematics
│   ├── teleop_uarm_node.hpp          # Main ROS 2 teleop publishing node
│   ├── zhongli_bus_worker.hpp        # Background 1 Mbps I/O worker and RealtimeBuffer
│   ├── zhongli_protocol.hpp          # Low-level serial protocol encoder/decoder
│   └── zhongli_types.hpp             # Kinematic and snapshot data structures
├── src/
│   ├── calibrate_uarm_node.cpp       # Standalone interactive calibration CLI (1 Mbps)
│   ├── follower_dispatcher.cpp       # Dispatcher implementation
│   ├── kinematic_remapper.cpp        # Remapping logic implementation
│   ├── teleop_uarm_node.cpp          # Node lifecycle and wait-free publishing
│   ├── zhongli_bus_worker.cpp        # 50 Hz background loop & fault escalation
│   └── zhongli_protocol.cpp          # Serial framing, mutex protection & parsing
├── test/
│   ├── test_bus_worker.cpp           # RealtimeBuffer and worker lifecycle tests
│   ├── test_command_format.cpp       # Protocol packet verification tests (1 Mbps)
│   ├── test_kinematic_conversion.cpp # Remapper math and clamping unit tests
│   └── test_kinematic_remapping.cpp  # Kinematic range mapping tests
├── CMakeLists.txt
└── package.xml
```

---

## ⚙️ Servo Baud Rate Configuration

If the servos on your leader arm are configured to the factory default 115,200 baud, send the Zhongli baud rate configuration command once before running the high-rate driver:
```text
#000PBD5!   (Sets all servos on the bus to 1,000,000 bps)
```

---

## 🛠️ Usage

### 1. Calibrate Leader Arm (1 Mbps)
Run the interactive calibration tool:
```bash
ros2 run teleop_zhongli_servo_hw calibrate_uarm_node --port /dev/uarm_leader --baud 1000000
```

### 2. Launch Leader Arm Teleoperation (50 Hz)
```bash
ros2 launch teleop_zhongli_servo_hw teleop_uarm.launch.py port:=/dev/uarm_leader arm_mode:=forward_position
```
