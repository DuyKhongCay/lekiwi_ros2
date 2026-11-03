# How-To: Teleoperation & Demonstration Control Setup

This guide details teleoperation setup for manual demonstration recording and robot control on LeKiwi.

---

## 🎮 Primary Workflow: USB Gamepad Teleoperation

LeKiwi natively integrates USB joystick gamepad control for base navigation, Cartesian jogging, and episode demonstration recording:

### 1. Connect and Verify Controller

Connect a standard USB gamepad (e.g. Logitech F710, Xbox, or PS4 controller). Verify the Linux joystick device is recognized:

```bash
ls -l /dev/input/js0
```

### 2. Launch Gamepad Teleoperation Node

Launch the gamepad teleoperation stack:

```bash
ros2 launch lekiwi_bringup teleop_gamepad.launch.py
```

### 3. Gamepad Button Mapping

* **Left Stick ($X/Y$)**: Omni-directional translation ($v_x, v_y$).
* **Right Stick ($X$)**: In-place base rotation ($\omega_z$).
* **D-Pad UP**: Trigger start of demonstration recording (`/episode_recorder_node/start_episode`).
* **D-Pad DOWN**: Save recorded demonstration episode as success (`/episode_recorder_node/save_episode`).
* **D-Pad LEFT / RIGHT**: Discard invalid trial (`/episode_recorder_node/discard_episode`).
* **L1 / R1**: Emergency software disarm / E-stop.

---

## 🦾 Bilateral Leader Arm Teleoperation (Hardware Extension)

> [!NOTE]
> The Zhongli / uArm bilateral leader arm interface operates via the external `teleop_zhongli_servo_hw` driver extension. For standard demonstration collection, the gamepad workflow above is recommended.

When using a physical bilateral leader arm connected to `/dev/uarm_leader`:

1. Calibrate zero-offsets with the leader in its mechanical cradle.
2. The leader arm publishes joint angles on `/leader/joint_states`.
3. The follower arm tracks the mirrored joint trajectories via `ros2_control` in real time.
