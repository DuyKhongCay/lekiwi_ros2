# How-To: Record MCAP Demonstration Episodes

This guide shows how to operate `chess_episode_recorder` to collect demonstration datasets with synchronized vision and joint trajectories using a Gamepad controller.

---

## Prerequisites
- Gamepad USB controller connected (`/dev/gamepad`).
- Teleoperation leader arm or gamepad teleop running.
- Stereo and wrist camera topics streaming at 30 Hz.

---

## 1. Start Episode Recorder

Launch the master episode recorder suite:

```bash
source ~/lekiwi_ws/install/setup.bash
ros2 launch chess_episode_recorder recorder.launch.py
```

To supply a custom configuration file with specific storage directories and topic remappings:

```bash
ros2 launch chess_episode_recorder recorder.launch.py \
    params_file:=$(ros2 pkg prefix chess_episode_recorder)/share/chess_episode_recorder/config/episode_recorder_params.yaml
```

The node automatically subscribes to:
- `/cameras/stereo_left/image_raw/compressed`
- `/cameras/usb_wrist/image_raw/compressed`
- `/joint_states`
- `/arm_forward_controller/commands`
- `/gamepad/joy`

---

## 2. Teleoperation Recording Workflow

Use the Gamepad D-Pad buttons:

| Button | Action | State Transition & Status LED |
| :--- | :--- | :--- |
| **D-Pad UP** | Start recording episode | Opens new MCAP file; Status: `RECORDING` |
| **D-Pad DOWN** | Save successful episode | Writes metadata file; Status: `SUCCESS` |
| **D-Pad LEFT** | Discard current episode | Deletes temporary MCAP bag; Status: `DISCARDED` |

---

## 3. Verify Recorded Bag Integrity

Inspect the resulting MCAP bag file:

```bash
ros2 bag info ~/lekiwi_ws/src/lekiwi_ros2/ros_bag/session_chess_demo/episode_000001
```

Confirm that image frames and joint states have zero dropped frames and matching timestamps.
