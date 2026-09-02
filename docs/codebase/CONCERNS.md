# Codebase Concerns

## Core Sections (Required)

### 1) Top Risks (Prioritized)

| Severity | Concern | Evidence | Impact | Suggested action |
|----------|---------|----------|--------|------------------|
| **High** | **Dual Serial Bus Contention & Device Drift**: System communicates over two distinct UART interfaces (`/dev/lekiwi_serial` for robot servos and `/dev/uarm_leader` for teleop). | [`lekiwi_ftservo_hardware/src/lekiwi_feetech_hardware.cpp:89`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware/src/lekiwi_feetech_hardware.cpp#L89), [`teleop_zhongli_servo_hw/src/teleop_uarm_node.cpp:36`](file://$HOME/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/src/teleop_uarm_node.cpp#L36) | In the absence of strict udev rule symlinks, device enumeration on Linux may invert USB serial ports upon reboot, directing commands to the wrong hardware. | Maintain and enforce udev rules in `lekiwi_bringup/udev/99-lekiwi.rules & 99-teleop-lekiwi.rules` matching device serial numbers and vendor IDs. |
| **Medium** | **High Disk Footprint in Deprecated Modules & Core Dumps**: The workspace root contains core dump binaries (`core.610991`, `core.622922`) and archived `deprecated/` models totaling over 250 MB. | [`lekiwi_ros2/`](file://$HOME/docker_ws/lekiwi_ros2), [`deprecated/COLCON_IGNORE`](file://$HOME/docker_ws/lekiwi_ros2/deprecated/COLCON_IGNORE) | Unnecessary repository bloat, potential git commit accidents, and disk usage on embedded devices. | Delete root core dumps (`rm core.*`) and keep `deprecated/` isolated under `COLCON_IGNORE` and `.gitignore`. |
| **Medium** | **Empty Subsystem README Files**: Package READMEs for `lekiwi_perception` and `lekiwi_control` were previously empty or minimal. | [`lekiwi_perception/README.md`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_perception/README.md), [`lekiwi_control/README.md`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_control/README.md) | Lack of package-level onboarding guidance for parameter tuning, topic lists, and launch commands. | Maintain comprehensive README files across all workspace packages aligned with `docs/codebase/`. |
| **Low** | **Roadmap Packages in Early Development**: `lekiwi_navigation/` contains skeleton README documentation for Nav2. | [`lekiwi_navigation/README.md`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_navigation/README.md) | Developers might assume Nav2 is fully functional when it is still in the roadmap stage. | Keep roadmap flags clear in `STRUCTURE.md` and `README.md`. |

---

### 2) Technical Debt

List the most important debt items only.

| Debt item | Why it exists | Where | Risk if ignored | Suggested fix |
|-----------|---------------|-------|-----------------|---------------|
| **Legacy Parameter Aliases in Task Orchestrator** | Retained for backward compatibility during node migration (`camera_hub_node` -> `cam_hub_node`). | [`lekiwi_control/lekiwi_control/task_orchestrator.py:104-112`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_control/lekiwi_control/task_orchestrator.py#L104-L112) | Minor runtime parameter confusion; deprecation warnings emitted to rosout. | Clean up fallback aliases once launch files and orchestration configs are finalized. |
| **Root Level Core Dump Artifacts** | Leftover crash dumps from development debugging of hardware / perception nodes. | [`core.610991`, `core.622922`](file://$HOME/docker_ws/lekiwi_ros2) | Takes ~260 MB disk space and clutters workspace root. | Clean up core dumps and configure `ulimit -c 0` or move core dumps to a dedicated debug location. |
| **Archived Legacy Perception Packages** | Preserved during architectural migration to the lifecycle component model. | [`deprecated/hailo_perception_node/`](file://$HOME/docker_ws/lekiwi_ros2/deprecated/hailo_perception_node), [`deprecated/lekiwi_perception_bak/`](file://$HOME/docker_ws/lekiwi_ros2/deprecated/lekiwi_perception_bak) | Repo size bloat; outdated code may be accidentally referenced. | Archive legacy branches in git history and remove the physical directory once validated. |

---

### 3) Security Concerns

| Risk | OWASP category (if applicable) | Evidence | Current mitigation | Gap |
|------|--------------------------------|----------|--------------------|-----|
| **Physical Actuation Safety on Real Hardware** | A04:2021 - Insecure Design (Robotics Physical Safety) | [`lekiwi_bringup/launch/robot.launch.py:18`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_bringup/launch/robot.launch.py#L18), [`lekiwi_bringup/config/servos/lekiwi_arm_calib.yaml:76-78`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_bringup/config/servos/lekiwi_arm_calib.yaml#L76-L78) | Launch default is `hardware_type:=mock` (no serial packets sent). | When switching to `hardware_type:=real`, wrong velocity direction or uncalibrated limits in `lekiwi_arm_calib.yaml` can cause sudden wheel spin or joint collision. |
| **Device File Permissions** | A05:2021 - Security Misconfiguration | [`lekiwi_bringup/udev/99-lekiwi.rules & 99-teleop-lekiwi.rules`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_bringup/udev/99-lekiwi.rules & 99-teleop-lekiwi.rules) | Udev rules grant `MODE="0666"` for serial, teleop, and camera devices. | Any local process can access raw serial motor control without authentication. Ensure network isolation of the host Raspberry Pi. |

---

### 4) Performance and Scaling Concerns

| Concern | Evidence | Current symptom | Scaling risk | Suggested improvement |
|---------|----------|-----------------|-------------|-----------------------|
| **CPU Saturation during Multi-Camera Video Decoding** | [`lekiwi_bringup/config/perception/gscam_cameras.yaml:20,37`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_bringup/config/perception/gscam_cameras.yaml#L20) | High CPU utilization if all 4 camera streams are decoded simultaneously. | Frame drop and increased inference latency on Raspberry Pi 5. | Ensure dynamic GStreamer valve gating (`active_modes`) is strictly applied so only active cameras run decode/scale pipelines. |
| **Serial Bus Polling Period** | [`lekiwi_ftservo_hardware/include/lekiwi_ftservo_hardware/lekiwi_feetech_hardware.hpp:108`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware/include/lekiwi_ftservo_hardware/lekiwi_feetech_hardware.hpp#L108) | Sequential sync read packets take ~15-20 ms for 9 servos. | Cannot scale to > 50 Hz control rate without bus optimization or splitting into two UART channels. | Utilize sync-read / broadcast read commands where possible to minimize turnaround latency. |

---

### 5) Fragile/High-Churn Areas

| Area | Why fragile | Churn signal | Safe change strategy |
|------|-------------|-------------|----------------------|
| [`lekiwi_bringup/launch/robot.launch.py`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_bringup/launch/robot.launch.py) | Central orchestration hub connecting all subsystems; changes affect overall boot order and default arguments. | Top 1 churn file (7 commits in 90 days) | Test changes using individual launch files (`teleop_uarm.launch.py`, `cameras.launch.py`, `control.launch.py`) before modifying the top-level launch. |
| [`teleop_zhongli_servo_hw/src/teleop_uarm_node.cpp`](file://$HOME/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/src/teleop_uarm_node.cpp) | Direct serial protocol communication with physical leader arm servos and real-time trajectory dispatch. | Actively developed new package | Validate via `test_kinematic_conversion` and run in standalone launch mode before integrated testing. |
| [`lekiwi_perception/src/camera_streamer_component.cpp`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_perception/src/camera_streamer_component.cpp) | GStreamer C-API integration, thread synchronization, and ROS lifecycle management. | High churn (4 commits) | Verify with `test_camera_streamer_component` GTest suite using `videotestsrc` before testing on hardware. |
| [`lekiwi_description/urdf/ros2_control.xacro`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_description/urdf/ros2_control.xacro) | Bridges URDF kinematic joints with ros2_control hardware plugins and parameter files. | 3 commits | Validate with `check_urdf` and launch `description.launch.py` in mock mode. |

---

### 6) `[ASK USER]` Questions (Resolved)

1. **`[ASK USER]`** Thư mục `deprecated/` (chứa các package cũ và model HEF ~30 MB):
   - **Quyết định của người dùng**: Giữ nguyên thư mục `deprecated/`, không xóa để phục vụ đối chiếu và lưu trữ lịch sử phát triển.
2. **`[ASK USER]`** Thư mục `lekiwi_navigation/` và `teleop_zhongli_servo_hw/`:
   - **Quyết định của người dùng**: Đã triển khai `teleop_zhongli_servo_hw` hoàn chỉnh; `lekiwi_navigation` giữ nguyên cấu trúc skeleton cho Nav2 roadmap.
3. **`[ASK USER]`** Đường dẫn liên kết trong tài liệu codebase:
   - **Quyết định của người dùng**: Đã thống nhất chuẩn hóa toàn bộ các đường dẫn tài liệu thành `file://$HOME/docker_ws/lekiwi_ros2/...`.

---

### 7) Evidence

- [`docs/codebase/.codebase-scan.txt`](file://$HOME/docker_ws/lekiwi_ros2/docs/codebase/.codebase-scan.txt)
- [`lekiwi_ftservo_hardware/src/lekiwi_feetech_hardware.cpp`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware/src/lekiwi_feetech_hardware.cpp)
- [`teleop_zhongli_servo_hw/src/teleop_uarm_node.cpp`](file://$HOME/docker_ws/lekiwi_ros2/teleop_zhongli_servo_hw/src/teleop_uarm_node.cpp)
- [`lekiwi_perception/README.md`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_perception/README.md)
- [`lekiwi_control/README.md`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_control/README.md)
- [`lekiwi_bringup/launch/robot.launch.py`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_bringup/launch/robot.launch.py)
- [`lekiwi_bringup/udev/99-lekiwi.rules & 99-teleop-lekiwi.rules`](file://$HOME/docker_ws/lekiwi_ros2/lekiwi_bringup/udev/99-lekiwi.rules & 99-teleop-lekiwi.rules)
