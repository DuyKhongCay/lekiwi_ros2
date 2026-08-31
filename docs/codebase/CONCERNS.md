# Codebase Concerns

## Core Sections (Required)

### 1) Top Risks (Prioritized)

| Severity | Concern | Evidence | Impact | Suggested action |
|----------|---------|----------|--------|------------------|
| **High** | **Single Serial Bus Contention & Latency**: All 9 Feetech STS servos (6 arm joints + 3 omni wheels) share one UART serial bus (`/dev/lekiwi_serial`) at 1 Mbps. | [`lekiwi_ftservo_hardware/src/lekiwi_feetech_hardware.cpp:89`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware/src/lekiwi_feetech_hardware.cpp#L89) | A single malformed packet, physical wire disconnection, or unresponsive servo ID stalls sequential polling and impacts the entire robot's motion. | Maintain the async I/O worker thread pattern; enforce strict packet timeouts (e.g. 20 ms), and configure watchdog alerts on `/lekiwi_base/drive_status`. |
| **Medium** | **High Disk Footprint in Deprecated Modules**: The `deprecated/` folder contains archived packages with duplicate compiled Hailo HEF model files (`yolo11n.hef`, `yolov8n-seg.hef`) totaling over 30 MB. | [`docs/codebase/.codebase-scan.txt:39-49`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/docs/codebase/.codebase-scan.txt#L39-L49), [`deprecated/COLCON_IGNORE`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/deprecated/COLCON_IGNORE) | Unnecessary repository bloat and potential confusion regarding active vs deprecated model binaries. | Safely purge `deprecated/` packages once current modular perception and hardware interfaces are validated. |
| **Medium** | **Empty Subsystem README Files**: Package READMEs for `lekiwi_perception` and `lekiwi_control` are completely empty (0 bytes). | [`lekiwi_perception/README.md`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_perception/README.md), [`lekiwi_control/README.md`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_control/README.md) | Lack of package-level onboarding guidance for parameter tuning, topic lists, and launch commands. | Populate both README files with usage documentation based on `docs/codebase/`. |
| **Low** | **Empty Placeholder Directories**: `lekiwi_navigation/` and `teleop_hardware/` exist in the workspace root but contain no files. | [`lekiwi_navigation/`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_navigation), [`teleop_hardware/`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/teleop_hardware) | Workspace clutter with no active package manifest or code. | Either initialize the packages with proper `package.xml`/`CMakeLists.txt` or remove the empty directories. |

---

### 2) Technical Debt

List the most important debt items only.

| Debt item | Why it exists | Where | Risk if ignored | Suggested fix |
|-----------|---------------|-------|-----------------|---------------|
| **Legacy Parameter Aliases in Task Orchestrator** | Retained for backward compatibility during node migration (`camera_hub_node` -> `cam_hub_node`). | [`lekiwi_control/lekiwi_control/task_orchestrator.py:104-112`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_control/lekiwi_control/task_orchestrator.py#L104-L112) | Minor runtime parameter confusion; deprecation warnings emitted to rosout. | Clean up fallback aliases once launch files and orchestration configs are finalized. |
| **Archived Legacy Perception Packages** | Preserved during architectural migration to the lifecycle component model. | [`deprecated/hailo_perception_node/`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/deprecated/hailo_perception_node), [`deprecated/lekiwi_perception_bak/`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/deprecated/lekiwi_perception_bak) | Repo size bloat; outdated code may be accidentally referenced. | Archive legacy branches in git history and remove the physical directory. |

---

### 3) Security Concerns

| Risk | OWASP category (if applicable) | Evidence | Current mitigation | Gap |
|------|--------------------------------|----------|--------------------|-----|
| **Physical Actuation Safety on Real Hardware** | A04:2021 - Insecure Design (Robotics Physical Safety) | [`lekiwi_bringup/launch/robot.launch.py:18`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_bringup/launch/robot.launch.py#L18), [`lekiwi_bringup/config/hardware/lekiwi_joints.yaml:76-78`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_bringup/config/hardware/lekiwi_joints.yaml#L76-L78) | Launch default is `hardware_type:=mock` (no serial packets sent). | When switching to `hardware_type:=real`, wrong velocity direction or uncalibrated limits in `lekiwi_joints.yaml` can cause sudden wheel spin or joint collision. |
| **Device File Permissions** | A05:2021 - Security Misconfiguration | [`lekiwi_bringup/udev/99-lekiwi.rules`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_bringup/udev/99-lekiwi.rules) | Udev rules grant `MODE="0666"` for serial and camera devices. | Any local process can access raw serial motor control without authentication. Ensure network isolation of the host Raspberry Pi. |

---

### 4) Performance and Scaling Concerns

| Concern | Evidence | Current symptom | Scaling risk | Suggested improvement |
|---------|----------|-----------------|-------------|-----------------------|
| **CPU Saturation during Multi-Camera Video Decoding** | [`lekiwi_bringup/config/perception/gscam_cameras.yaml:20,37`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_bringup/config/perception/gscam_cameras.yaml#L20) | High CPU utilization if all 4 camera streams are decoded simultaneously. | Frame drop and increased inference latency on Raspberry Pi 5. | Ensure dynamic GStreamer valve gating (`active_modes`) is strictly applied so only active cameras run decode/scale pipelines. |
| **Serial Bus Polling Period** | [`lekiwi_ftservo_hardware/include/lekiwi_ftservo_hardware/lekiwi_feetech_hardware.hpp:108`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware/include/lekiwi_ftservo_hardware/lekiwi_feetech_hardware.hpp#L108) | Sequential sync read packets take ~15-20 ms for 9 servos. | Cannot scale to > 50 Hz control rate without bus optimization or splitting into two UART channels. | Utilize sync-read / broadcast read commands where possible to minimize turnaround latency. |

---

### 5) Fragile/High-Churn Areas

| Area | Why fragile | Churn signal | Safe change strategy |
|------|-------------|-------------|----------------------|
| [`lekiwi_bringup/launch/robot.launch.py`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_bringup/launch/robot.launch.py) | Central orchestration hub connecting all subsystems; changes affect overall boot order and default arguments. | Top 1 churn file (6 commits in 90 days) | Test changes using individual launch files (`cameras.launch.py`, `control.launch.py`) before modifying the top-level launch. |
| [`lekiwi_perception/src/camera_streamer_component.cpp`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_perception/src/camera_streamer_component.cpp) | GStreamer C-API integration, thread synchronization, and ROS lifecycle management. | High churn (4 commits) | Verify with `test_camera_streamer_component` GTest suite using `videotestsrc` before testing on hardware. |
| [`lekiwi_description/urdf/ros2_control.xacro`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_description/urdf/ros2_control.xacro) | Bridges URDF kinematic joints with ros2_control hardware plugins and parameter files. | 3 commits | Validate with `check_urdf` and launch `description.launch.py` in mock mode. |

---

### 6) `[ASK USER]` Questions (Resolved)

1. **`[ASK USER]`** Thư mục `deprecated/` (chứa các package cũ và model HEF ~30 MB):
   - **Quyết định của người dùng**: Giữ nguyên thư mục `deprecated/`, không xóa để phục vụ đối chiếu và lưu trữ lịch sử phát triển.
2. **`[ASK USER]`** Hai thư mục `lekiwi_navigation/` và `teleop_hardware/`:
   - **Quyết định của người dùng**: Giữ lại vì nằm trong kế hoạch phát triển tiếp theo (tích hợp Nav2 và hardware teleop); đã bổ sung tài liệu roadmap `README.md` cho cả 2 thư mục.
3. **`[ASK USER]`** Tài liệu hướng dẫn README cho từng package và file tổng `lekiwi_ros2/README.md`:
   - **Quyết định của người dùng**: Đã tự động tạo và cập nhật trọn bộ `README.md` cho root project và toàn bộ 9 package trong workspace.

---

### 7) Evidence

- [`docs/codebase/.codebase-scan.txt`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/docs/codebase/.codebase-scan.txt)
- [`lekiwi_ftservo_hardware/src/lekiwi_feetech_hardware.cpp`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_ftservo_hardware/src/lekiwi_feetech_hardware.cpp)
- [`lekiwi_perception/README.md`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_perception/README.md)
- [`lekiwi_control/README.md`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_control/README.md)
- [`lekiwi_bringup/launch/robot.launch.py`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_bringup/launch/robot.launch.py)
- [`lekiwi_bringup/udev/99-lekiwi.rules`](file:///home/duykhongcay/docker_ws/lekiwi_ros2/lekiwi_bringup/udev/99-lekiwi.rules)

