# `teleop_zhongli_servo_hw`

Driver, interactive midpoint calibration CLI, kinematic remapping engine, and ROS 2 publisher for the **uArm / Zhongli Serial Bus Leader Arm** used in teleoperation and demonstration recording for the LeKiwi robot.

---

## 🚀 Features

- **High-Frequency Kinematic Polling (50 Hz)**: Reads raw angles from 6 unpowered Zhongli / Feetech leader arm servos via `zhongli_protocol` over serial (`/dev/uarm_leader`).
- **Midpoint-First Interactive Calibration CLI**:
  - Interactive CLI tool (`calibrate_uarm_node`) guiding zero-lock (`#<ID>PSCK!`) and range of motion recording.
  - Automatically exports clean, pure `uarm_teleop_calib.yaml` configuration without manual formula calculations.
- **Dynamic Kinematic Remapping**:
  - Remaps raw leader joint coordinates to follower arm radian limits with direction inversion, offset, and scale factors.
  - Supports multiple follower arm dispatch modes:
    - `joint_trajectory`: Publishes to `/arm_trajectory_controller/joint_trajectory` (`trajectory_msgs/msg/JointTrajectory`).
    - `forward_position`: Publishes to `/arm_forward_controller/commands` (`std_msgs/msg/Float64MultiArray`).
    - `joint_states_only`: Only publishes leader `sensor_msgs/msg/JointState` to `/leader/joint_states`.
- **Hardware Diagnostics (1 Hz)**:
  - Publishes packet health, latency, frequency, and servo telemetry to `diagnostic_updater` (`/diagnostics`).

---

## 📦 Package Structure

```text
teleop_zhongli_servo_hw/
├── include/teleop_zhongli_servo_hw/
│   ├── follower_dispatcher.hpp       # Dispatches commands to ros2_control controllers
│   ├── kinematic_remapper.hpp        # Translates leader to follower kinematics
│   ├── teleop_uarm_node.hpp          # Main ROS 2 teleop publishing node
│   └── zhongli_protocol.hpp          # Low-level serial protocol encoder/decoder
├── src/
│   ├── calibrate_uarm_node.cpp       # Standalone interactive calibration CLI
│   ├── follower_dispatcher.cpp       # Dispatcher implementation
│   ├── kinematic_remapper.cpp        # Remapping logic implementation
│   ├── teleop_uarm_node.cpp          # Node lifecycle, timers, and diagnostics
│   └── zhongli_protocol.cpp          # Serial framing and packet parsing
├── test/
│   ├── test_command_format.cpp       # Protocol packet verification tests
│   └── test_kinematic_conversion.cpp # Remapper math and clamping unit tests
├── CMakeLists.txt
└── package.xml
```

---

## 🛠️ Usage

### 1. Calibrate Leader Arm
Run the interactive calibration tool:
```bash
ros2 run teleop_zhongli_servo_hw calibrate_uarm_node
```

### 2. Launch Leader Arm Teleoperation
```bash
ros2 launch lekiwi_bringup teleop_uarm.launch.py port:=/dev/uarm_leader arm_mode:=joint_trajectory
```
