# `teleop_hardware`

Dedicated teleoperation hardware drivers and leader-arm interface package for the LeKiwi robot (Roadmap / In Development).

---

## 🎮 Planned Scope

- **Leader Arm Teleoperation**: Reading joint positions from an unpowered leader arm (SO-100 / LeRobot leader) and mirroring motions to the LeKiwi follower arm in real-time.
- **Haptic & Custom Input Devices**: Interfacing custom gamepad pedals, micro-controllers (ESP32 / Arduino), and USB HID controllers for multi-modal teleoperation.
- **Zero-Latency Trajectory Streaming**: Direct velocity/position feedforward to `ros2_control` without network overhead.

---

## 📁 Target Structure

```text
teleop_hardware/
├── include/teleop_hardware/ # C++ driver headers
├── src/                     # Device polling and serial workers
├── config/                  # Device mapping and scale YAMLs
├── launch/                  # teleop_hardware.launch.py
├── CMakeLists.txt
└── package.xml
```

