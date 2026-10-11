# How-To Guides: Operational Problem-Solving

How-To Guides are **task-oriented** recipes that escort developers through specific operational procedures. They assume foundational knowledge of LeKiwi and focus directly on achieving practical outcomes.

---

## 🛠️ Guide Categories

### Deployment & Networking
- [**Distributed Pi 5 Docker & PC Zenoh Setup**](deployment/distributed-pi5-docker-and-pc-zenoh-setup.md): Deploy the robot runtime stack inside Docker on Raspberry Pi 5 and establish zero-multicast Zenoh connection with remote PC workstations for RViz2 monitoring.

### Hardware & Teleoperation
- [**Udev Rules & Serial Bus Setup**](hardware/udev-and-serial-setup.md): Configure stable device symlinks and access rights for Feetech servos and IMUs.
- [**Motor Bus Fault Recovery**](hardware/motor-bus-troubleshooting.md): Identify servo communication timeouts, clear hardware faults, and reset motor bus torque.
- [**Teleoperation Arm Setup**](hardware/teleop-arm-setup.md): Calibrate and configure the Zhongli leader arm for bilateral or unilateral demonstration teaching.

### Sensor Calibration
- [**Stereo Camera & Hand-Eye Calibration**](calibration/stereo-and-handeye-calibration.md): Compute camera intrinsic parameters, stereo rectification matrices, and the robot base-to-camera transformation.

### Physical-AI & Policy Serving
- [**Record MCAP Demonstration Episodes**](physical-ai/record-mcap-episodes.md): Collect structured episodes with automated labeling and metadata via gamepad D-Pad.
- [**Convert MCAP to LeRobot Dataset**](physical-ai/convert-rosbag-to-lerobot.md): Export rosbags into Hugging Face LeRobot format for policy training.
- [**Deploy ZeroMQ Policy Server**](physical-ai/deploy-zeromq-policy-server.md): Set up low-latency binary streaming between GPU workstations and LeKiwi on-board compute.

### Navigation & Localization
- [**Tune Nav2 for Omni-Base**](navigation/tune-nav2-omni-base.md): Optimize controller plugins, costmaps, and omnidirectional kinematics parameters for the 3-wheel chassis.

