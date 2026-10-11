# How-To: Teleoperation Arm Setup & Calibration

This guide details how to configure the Zhongli / uArm leader arm for manual teleoperation and demonstration recording.

---

## Hardware Connection
Connect the Zhongli leader arm via USB. Ensure the persistent symlink `/dev/uarm_leader` is active as configured in [Udev Rules Setup](udev-and-serial-setup.md).

---

## 1. Zero-Position Calibration

Place the leader arm into its mechanical homing cradle, then run the calibration node:

```bash
ros2 run teleop_zhongli_servo_hw calibrate_uarm_node
```

The calibration node reads optical encoders and writes offset biases for all leader joints into configuration memory.

---

## 2. Launch Leader Arm Teleoperation Driver

Launch the teleoperation driver:

```bash
source ~/lekiwi_ws/install/setup.bash
ros2 launch teleop_zhongli_servo_hw teleop_uarm.launch.py
```

Verify published leader joint states:

```bash
ros2 topic echo /leader/joint_states --once
```

---

## 3. Enable Mirroring Mode (Follower Tracking)

When master bringup is running (`ros2 launch lekiwi_bringup robot.launch.py uarm_teleop:=true`), the follower arm automatically tracks leader arm motions in real time.

> [!WARNING]
> Ensure the LeKiwi follower arm workspace is clear before moving the leader arm. Keep your hand near the emergency stop button on the gamepad.
