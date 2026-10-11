# Tutorial 1: Workspace Setup & Build

This tutorial guides you through configuring your host system, installing dependencies, configuring hardware udev permissions, and successfully building the complete `lekiwi_ros2` workspace.

---

## Prerequisites

Before starting, verify that your environment satisfies these system requirements:

- **Operating System**: Ubuntu 24.04 LTS (Noble Numbat) or Ubuntu 22.04 LTS in container.
- **ROS 2 Distribution**: ROS 2 Jazzy Jalisco (or Humble Hawksbill).
- **Build Tools**: `colcon`, `rosdep`, `git`, `cmake` (3.22+).
- **Python Version**: Python 3.10+ (Jazzy uses Python 3.12).

> [!TIP]
> **Running on Raspberry Pi 5 with Docker Container?**
> If you are deploying directly onto the physical robot (Raspberry Pi 5 with Hailo-8 NPU, CSI stereo cameras, and Feetech serial bus), you can bypass manual host toolchain installation and run the isolated containerized stack. Refer directly to [**Distributed Pi 5 Docker & PC Zenoh Setup**](../how-to-guides/deployment/distributed-pi5-docker-and-pc-zenoh-setup.md).

---

## Step 1: Clone the Repository

Clone the LeKiwi repository with all submodules into your active development workspace:

```bash
mkdir -p ~/lekiwi_ws/src
cd ~/lekiwi_ws/src
git clone --recurse-submodules https://github.com/lekiwi/lekiwi_ros2.git
```

Verify your directory structure:

```bash
ls ~/lekiwi_ws/src/lekiwi_ros2
```

You should see 15 packages including `lekiwi_bringup`, `lekiwi_ftservo_hardware`, `lekiwi_orchestrator`, and `lekiwi_interfaces`.

---

## Step 2: Install ROS 2 System Dependencies

Navigate to the workspace root and use `rosdep` to pull required system and ROS packages:

```bash
cd ~/lekiwi_ws
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Install additional Python dependencies required by the Physical-AI and policy bridge modules:

```bash
pip3 install pyzmq msgpack numpy opencv-python pyyaml
```

---

## Step 3: Install Device Udev Rules

The LeKiwi hardware communicates across dedicated serial devices requiring persistent permissions:

```bash
cd ~/lekiwi_ws/src/lekiwi_ros2
sudo cp lekiwi_bringup/udev/99-lekiwi.rules /etc/udev/rules.d/
sudo cp teleop_zhongli_servo_hw/udev/99-teleop-lekiwi.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

This ensures `/dev/lekiwi_serial` (Feetech STS3215 bus) and `/dev/uarm_leader` (teleop arm) are accessible without root credentials.

---

## Step 4: Compile the Workspace

Build the workspace using `colcon`. On embedded platforms like the Raspberry Pi 5, limit parallel jobs to conserve memory:

```bash
cd ~/lekiwi_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 4
```

> [!NOTE]
> If building specifically for simulation or development without physical Hailo hardware, the build will gracefully omit optional HailoRT C++ targets.

---

## Step 5: Verification

Source the workspace overlay:

```bash
source ~/lekiwi_ws/install/setup.bash
```

Confirm that the LeKiwi packages and custom interfaces are registered:

```bash
ros2 pkg list | grep lekiwi
```

**Expected Output:**
```text
lekiwi_bringup
lekiwi_calibration
lekiwi_chess_master
lekiwi_description
lekiwi_ftservo_hardware
lekiwi_icm20948_hardware
lekiwi_inference
lekiwi_interfaces
lekiwi_motion
lekiwi_orchestrator
lekiwi_perception
```

Verify that the custom message interface exists:

```bash
ros2 interface show lekiwi_interfaces/action/ComputeBestMove
```

**Next Step:** Proceed to [Tutorial 2: Robot Bringup & Mock Simulation](02-robot-bringup-simulation.md) to launch and visualize the system.

