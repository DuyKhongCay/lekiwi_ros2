# Codebase Concerns

## Core Sections (Required)

### 1) Top Risks (Prioritized)

| Severity | Concern | Evidence | Impact | Suggested action |
|----------|---------|----------|--------|------------------|
| **High** | **Dual Serial Bus Contention & Device Drift**: System communicates over two distinct UART interfaces (`/dev/lekiwi_serial` for robot servos and `/dev/uarm_leader` for teleop). | [`lekiwi_ftservo_hardware/src/lekiwi_feetech_hardware.cpp:89`](file:///root/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware/src/lekiwi_feetech_hardware.cpp#L89), [`teleop_zhongli_servo_hw/src/teleop_uarm_node.cpp:36`](file:///root/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/src/teleop_uarm_node.cpp#L36) | In the absence of strict udev rule symlinks, device enumeration on Linux may invert USB serial ports upon reboot, directing commands to the wrong hardware. | Maintain and enforce udev rules in `lekiwi_bringup/udev/99-lekiwi.rules & 99-teleop-lekiwi.rules` matching device serial numbers and vendor IDs. |
| **Medium** | **High Disk Footprint in Deprecated Modules & Core Dumps**: The workspace root contains core dump binaries (`core.123666`, `core.135965`, `core.144070`, `core.154512`, `core.365785`) totaling over 700 MB. | [`lekiwi_ros2/`](file:///root/docker_ws/lekiwi_ros2), [`deprecated/COLCON_IGNORE`](file:///root/docker_ws/lekiwi_ros2/deprecated/COLCON_IGNORE) | Unnecessary repository bloat, potential git commit accidents, and disk usage on embedded devices. | Delete root core dumps (`rm core.*`) and keep `deprecated/` isolated under `COLCON_IGNORE` and `.gitignore`. |
| **Medium** | **Chessboard Localization Boundary Drift**: If AprilTag markers become occluded or poorly lit, SolvePnP pose estimation may fail or jitter. | [`lekiwi_tag_localization/src/chessboard_pose_estimator.cpp:210`](file:///root/docker_ws/lekiwi_ros2/lekiwi_tag_localization/src/chessboard_pose_estimator.cpp#L210) | Could cause sudden coordinate jumps in the chessboard TF frame, leading to inaccurate Nav2 costmap keepout positioning. | Enforce low-pass outlier filtering and minimum detected tag thresholds before broadcasting chessboard TF. |
| **Low** | **Multi-Camera CPU Utilization**: Running 4 GStreamer camera decoding pipelines simultaneously on Raspberry Pi 5. | [`lekiwi_bringup/config/perception/gscam_cameras.yaml`](file:///root/docker_ws/lekiwi_ros2/lekiwi_bringup/config/perception/gscam_cameras.yaml) | CPU starvation if dynamic valve gating is bypassed. | Ensure `CameraMode` FSM strictly opens valves only for the active mode cameras. |

---

### 2) Technical Debt

List the most important debt items only.

| Debt item | Why it exists | Where | Risk if ignored | Suggested fix |
|-----------|---------------|-------|-----------------|---------------|
| **Legacy Parameter Aliases in Task Orchestrator** | Retained for backward compatibility during node migration (`camera_hub_node` -> `cam_hub_node`). | [`lekiwi_control/lekiwi_control/task_orchestrator.py:104-112`](file:///root/docker_ws/lekiwi_ros2/lekiwi_control/lekiwi_control/task_orchestrator.py#L104-L112) | Minor runtime parameter confusion; deprecation warnings emitted to rosout. | Clean up fallback aliases once launch files and orchestration configs are finalized. |
| **Root Level Core Dump Artifacts** | Leftover crash dumps from development debugging of hardware / perception nodes. | [`core.*`](file:///root/docker_ws/lekiwi_ros2) | Takes ~780 MB disk space and clutters workspace root. | Clean up core dumps and configure `ulimit -c 0` or move core dumps to a dedicated debug location. |
| **Archived Legacy Perception Packages** | Preserved during architectural migration to the lifecycle component model. | [`deprecated/hailo_perception_node/`](file:///root/docker_ws/lekiwi_ros2/deprecated/hailo_perception_node), [`deprecated/lekiwi_perception_bak/`](file:///root/docker_ws/lekiwi_ros2/deprecated/lekiwi_perception_bak) | Repo size bloat; outdated code may be accidentally referenced. | Keep `COLCON_IGNORE` active and archive in git tags when stable. |

---

### 3) Security Concerns

| Risk | OWASP category (if applicable) | Evidence | Current mitigation | Gap |
|------|--------------------------------|----------|--------------------|-----|
| **Physical Actuation Safety on Real Hardware** | A04:2021 - Insecure Design (Robotics Physical Safety) | [`lekiwi_bringup/launch/robot.launch.py:18`](file:///root/docker_ws/lekiwi_ros2/lekiwi_bringup/launch/robot.launch.py#L18), [`lekiwi_navigation/config/twist_mux.yaml`](file:///root/docker_ws/lekiwi_ros2/lekiwi_navigation/config/twist_mux.yaml) | Launch default is `hardware_type:=mock`. `twist_mux` priority 255 E-Stop locks motor output on `/safety/estop_active`. | When switching to `hardware_type:=real`, uncalibrated limits in `lekiwi_arm_calib.yaml` can cause sudden wheel spin or joint collision. |
| **Device File Permissions** | A05:2021 - Security Misconfiguration | [`lekiwi_bringup/udev/99-lekiwi.rules & 99-teleop-lekiwi.rules`](file:///root/docker_ws/lekiwi_ros2/lekiwi_bringup/udev/99-lekiwi.rules) | Udev rules grant `MODE="0666"` for serial, teleop, and camera devices. | Any local process can access raw serial motor control without authentication. Ensure network isolation of the host Raspberry Pi. |

---

### 4) Performance and Scaling Concerns

| Concern | Evidence | Current symptom | Scaling risk | Suggested improvement |
|---------|----------|-----------------|-------------|-----------------------|
| **CPU Saturation during Multi-Camera Video Decoding** | [`lekiwi_bringup/config/perception/gscam_cameras.yaml:20,37`](file:///root/docker_ws/lekiwi_ros2/lekiwi_bringup/config/perception/gscam_cameras.yaml#L20) | High CPU utilization if all 4 camera streams are decoded simultaneously. | Frame drop and increased inference latency on Raspberry Pi 5. | Ensure dynamic GStreamer valve gating (`active_modes`) is strictly applied so only active cameras run decode/scale pipelines. |
| **Serial Bus Polling Period** | [`lekiwi_ftservo_hardware/include/lekiwi_ftservo_hardware/lekiwi_feetech_hardware.hpp:108`](file:///root/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware/include/lekiwi_ftservo_hardware/lekiwi_feetech_hardware.hpp#L108) | Sequential sync read packets take ~15-20 ms for 9 servos. | Cannot scale to > 50 Hz control rate without bus optimization or splitting into two UART channels. | Utilize sync-read / broadcast read commands where possible to minimize turnaround latency. |

---

### 5) Fragile/High-Churn Areas

| Area | Why fragile | Churn signal | Safe change strategy |
|------|-------------|-------------|----------------------|
| [`lekiwi_bringup/launch/robot.launch.py`](file:///root/docker_ws/lekiwi_ros2/lekiwi_bringup/launch/robot.launch.py) | Central orchestration hub connecting all subsystems; changes affect overall boot order and default arguments. | Top 1 churn file (8 commits in 90 days) | Test changes using individual launch files (`teleop_uarm.launch.py`, `cameras.launch.py`, `navigation.launch.py`) before modifying the top-level launch. |
| [`lekiwi_navigation/launch/navigation.launch.py`](file:///root/docker_ws/lekiwi_ros2/lekiwi_navigation/launch/navigation.launch.py) | Configures Nav2 lifecycle transitions and `twist_mux` arbitration for holonomic base. | Core navigation bringup | Validate with static map visualization in RViz2 before live robot deployment. |
| [`lekiwi_tag_localization/src/chessboard_pose_estimator.cpp`](file:///root/docker_ws/lekiwi_ros2/lekiwi_tag_localization/src/chessboard_pose_estimator.cpp) | Direct OpenCV SolvePnP pose estimation and TF broadcasting. | Active localization module | Verify with `test_pose_solver` GTest suite prior to integration testing. |

---

### 6) `[ASK USER]` Questions

1. **`[ASK USER]`** Dọn dẹp các file core dump ở thư mục gốc (`core.123666`, `core.135965`, `core.144070`, `core.154512`, `core.365785` - chiếm ~780 MB): Bạn có muốn dọn dẹp các file core dump này để giải phóng dung lượng đĩa không?

---

### 7) Evidence

- [`.codebase-scan.txt`](file:///root/docker_ws/lekiwi_ros2/docs/codebase/.codebase-scan.txt)
- [`lekiwi_bringup/launch/robot.launch.py`](file:///root/docker_ws/lekiwi_ros2/lekiwi_bringup/launch/robot.launch.py)
- [`lekiwi_tag_localization/src/chessboard_pose_estimator.cpp`](file:///root/docker_ws/lekiwi_ros2/lekiwi_tag_localization/src/chessboard_pose_estimator.cpp)
- [`lekiwi_navigation/launch/navigation.launch.py`](file:///root/docker_ws/lekiwi_ros2/lekiwi_navigation/launch/navigation.launch.py)
