# Repository Guidelines & Agent Specification

## 1. Project Overview & Architecture

This repository contains the production ROS 2 workspace for the **LeKiwi Robot** — an autonomous physical-AI robotic platform equipped with:
- **Mobility**: 3-wheel holonomic omnidirectional mobile base driven by Feetech STS3215 bus servos (`lekiwi_motion`, Nav2).
- **Manipulation**: 6-DoF robotic arm with gripper (`lekiwi_manipulation`, `teleop_zhongli_servo_hw`).
- **Perception**: Dual stereo camera pipelines accelerated by Hailo-8/8L NPU for real-time YOLO object detection, chessboard pose estimation, and FEN state generation (`lekiwi_perception`).
- **Hardware Abstraction**: Real-time `ros2_control` hardware & sensor plugins running asynchronous I/O threads on Raspberry Pi 5 (`lekiwi_ftservo_hardware`, `lekiwi_icm20948_hardware`).
- **Physical AI & Learning**: Integrated HuggingFace LeRobot policy streaming server (`lerobot_policy_server`), episode recording, and rosbag-to-dataset conversion pipeline (`chess_episode_recorder`, `chess_rosbag_to_lerobot`).
- **Task Orchestration**: Central Finite-State Machine orchestrator coordinating camera modes and mission lifecycles (`lekiwi_orchestrator`).

Keep generated `build/`, `install/`, and `log/` directories located directly within the `lekiwi_ros2/` project root (`/root/docker_ws/lekiwi_ros2/{build,install,log}`), and strictly keep their contents out of git commits / source changes.

---

## 2. Project Structure & Module Organization

| Package | Language / Framework | Role & Scope |
| :--- | :---: | :--- |
| `lekiwi_interfaces/` | ROS 2 IDL | Custom msgs (`CameraMode`, `DriveStatus`, `ServoTelemetry`) and srvs (`SetCamMode`, `ResetMotorBus`). |
| `lekiwi_description/` | URDF / Xacro | Robot kinematics, 3D meshes (STL), transmissions, and `ros2_control` tags. |
| `lekiwi_bringup/` | Launch & Configs | System compose launch files (`robot.launch.py`), YAML parameter files, and udev rules. |
| `lekiwi_perception/` | C++17 / GStreamer / HailoRT | Zero-copy lifecycle camera streamer, Hailo-8 NPU inference, chess vision components, GTests. |
| `lekiwi_ftservo_hardware/`| C++17 / `ros2_control` | `SystemInterface` plugin for 9 Feetech STS3215 servos over 1 Mbps serial bus with async I/O worker. |
| `lekiwi_icm20948_hardware/`| C++17 / `ros2_control` | `SensorInterface` plugin for ICM-20948 9-DoF IMU communicating over I2C (`/dev/i2c-1`). |
| `lekiwi_motion/` | Python / C++ / Nav2 | 3-wheel omni drive kinematics, twist multiplexing, and holonomic local planner navigation. |
| `lekiwi_manipulation/` | Python / `rclpy` | 6-DoF arm forward/inverse kinematics, joint trajectory execution, and LeRobot action bridge. |
| `lekiwi_orchestrator/` | Python / `rclpy` | Multi-mode FSM task manager (`STANDBY`, `NAV_EXPLORE`, `CHESS_THINKING`, `ARM_MANIPULATION`). |
| `lekiwi_calibration/` | Python / OpenCV | Native OpenCV GUI hand-eye calibration (Eye-to-Hand & Eye-in-Hand) and ChArUco target detection. |
| `lekiwi_chess_master/` | Python / Chess Engine | Chessboard FEN state analysis, legal move validation, and game loop decision engine. |
| `lerobot_policy_server/` | Python (PEP 621) / ZMQ / PyTorch | Standalone async LeRobot inference server communicating with ROS 2 client via ZeroMQ. |
| `chess_episode_recorder/` | Python / ROS 2 | Real-time synchronized time-series recorder for stereo camera frames and servo angles. |
| `chess_rosbag_to_lerobot/`| Python / LeRobot | Converter parsing ROS 2 MCAP/db3 rosbags into HuggingFace `LeRobotDataset` format. |
| `teleop_zhongli_servo_hw/`| C++ / Driver | Teleoperation master arm driver, interactive calibration CLI, and joint state publisher. |
| `scripts/` | Python Utilities | Developer diagnostics, trace log analysis, and camera image capture tools. |
| `deprecated/` | Legacy Code | Shelved packages marked with `COLCON_IGNORE`. Do not edit or reference in new features. |

---

## 3. Agent Operating Rules & Guardrails

- **Language Policy**: Always respond to users in **Vietnamese**; write code, comments, docstrings, and commit messages strictly in **English**.
- **Visuals**: Always use **Mermaid** syntax (` ```mermaid `) whenever generating diagrams, state machine transitions, dataflow pipelines, or architectural layouts.
- **Strict Protection of Agent Skills**: The agent is strictly **FORBIDDEN** from modifying, deleting, overwriting, creating, or running commands that mutate files within `.agents/skills/`, `.agents/`, or `skills-lock.json`. The agent may ONLY read skill files. Any installation, modification, or deletion of skills must be done manually by the user.
- **File Editing Protocol**:
  - Always use the dedicated editing tool (`replace_file_content` for existing files) to ensure git diffs are transparent.
  - **Never** use shell commands (`cat << EOF`, `echo >`, `sed`, `tee`, python scripts) to write or overwrite source code files.
  - Keep edits surgical, minimal, and limited strictly to requested files. Verify changes with build/test commands.
- **No Mocking of Missing Hardware**: In unit tests, never attempt to open `/dev/tty*`, `/dev/i2c*`, or connect to physical Hailo NPU. Use mocking and deterministic synthetic inputs.

---

## 4. Build, Development & Hardware Setup

### Centralized Workspace Build
All ROS packages (whether built from inside or outside the project, e.g. third-party or submodules) MUST always output their build artifacts directly into the `lekiwi_ros2/` project root's `build/`, `install/`, and `log/` directories (`/root/docker_ws/lekiwi_ros2/build`, `/root/docker_ws/lekiwi_ros2/install`, `/root/docker_ws/lekiwi_ros2/log`). Never allow nested or separate `build/`/`install/` directories inside individual package subfolders or out in the parent workspace. Always run `colcon build` from within the `lekiwi_ros2/` project root (or explicitly specify `--build-base /root/docker_ws/lekiwi_ros2/build --install-base /root/docker_ws/lekiwi_ros2/install --log-base /root/docker_ws/lekiwi_ros2/log`).

```bash
# Navigate to the lekiwi_ros2 project root
cd /root/docker_ws/lekiwi_ros2

# Source ROS 2 base environment
source /opt/ros/$ROS_DISTRO/setup.bash

# Build entire workspace with Ninja generator (Fast & Multi-threaded)
colcon build --symlink-install --cmake-args -GNinja
source install/setup.bash

# Build a focused single package
colcon build --symlink-install --packages-select lekiwi_perception --cmake-args -GNinja

# Build Python policy server package
pip install -e lerobot_policy_server/
```

### Launch Commands
```bash
# Full system launch
ros2 launch lekiwi_bringup robot.launch.py

# Launch perception stack only
ros2 launch lekiwi_bringup perception.launch.py

# Run standalone LeRobot Policy Server (GPU host)
policy-server --config config/server_config.yaml
```

### Hardware Devices & Permissions
- **Feetech Bus Servos**: Port `/dev/lekiwi_serial` (symlinked via udev rule from `/dev/ttyAMA*` or `/dev/ttyUSB*` at 1,000,000 baud).
- **ICM-20948 IMU**: I2C bus `/dev/i2c-1` (Default address `0x68` or `0x69`).
- **Hailo-8 NPU**: PCIe interface checked via `hailortcli fw-control identify`.

---

## 5. Coding Style & Naming Conventions

Use four spaces in Python and two spaces in CMake. Match nearby C++ formatting: two-space indentation, braces on their own line, `snake_case` functions/variables, `PascalCase` types, and `.hpp` headers. Keep ROS package and topic/config names lowercase with underscores. C++ targets compile with `-Wall -Wextra -Wpedantic`; resolve new warnings before submitting.

The agent must enforce a strict default rule where every generated class and function or method includes exactly one to two lines of comments. These default comments must concisely explain the purpose or "why" behind the code instead of stating obvious actions. For example, a default Python implementation should look like this:

```python
class DataProcessor:
    # Handles parsing and sanitizing raw input string streams.
    def clean_timestamp(self, raw_time):
        # Standardizes raw text time formats into a uniform ISO string.
        return processed_time
```

Similarly, a default C++ implementation without explicit style requests must follow the exact same structure:

```cpp
class DataProcessor {
    // Handles parsing and sanitizing raw input string streams.
    void cleanTimestamp(std::string rawTime) {
        // Standardizes raw text time formats into a uniform ISO string.
    }
};
```

However, when the user explicitly requests strict convention compliance, the agent must shift to standard documentation structures. For Python, it must strictly apply PEP 8 for comments and PEP 257 for docstrings, formatted as follows:

```python
class DataProcessor:
    """Manages the full data sanitization lifecycle for incoming logs."""

    def clean_timestamp(self, raw_time: str) -> str:
        """Standardize a raw datetime string into ISO 8601 format.

        Args:
            raw_time: The unformatted input timestamp string.
        """
        return processed_time
```

For C++, the agent must strictly implement the Doxygen standard using specialized tag blocks for classes and methods like this:

```cpp
/**
 * @brief Manages the full data sanitization lifecycle for incoming logs.
 */class DataProcessor {public:
    /**
     * @brief Standardize a raw datetime string into ISO 8601 format.
     * @param rawTime The unformatted input timestamp string.
     */
    std::string cleanTimestamp(std::string rawTime) {
        return processedTime;
    }
};
```

Naming & Abbreviation Conventions: For variables, functions, and namespaces consisting of two or more combined words that include the following terms, apply these standard abbreviations:

- buffer -> buff
- width -> w, height -> h
- acknowledge -> ack
- description -> desc
- destination -> dest
- srouce -> src, srouces->srcs
- diagnostic -> diag
- calibration -> calib
- capture -> cap, captures -> caps
- rotation -> rot
- pieces -> pcs
- point -> pt, points -> pts
- geometry -> geom
- camera -> cam, cameras -> cams
- left -> l, right -> r
- confidence -> conf
- matrix -> mat
- generation -> gen
- detection -> det, detections -> dets
- iteration -> iter, iterations -> iters
- result -> res
- return -> ret
- distance -> dist
- class -> cls
- count -> cnt
- argument -> arg, arguments -> args
- runtime -> rt
- error -> err
- fuction -> fuct
- parameter -> param, parameters -> params
- image -> img, images -> imgs
- information -> info
- latency -> lat
- multiply -> mul
- transformation -> trans
- configuration -> configs
- display -> disp
- statistic -> stats
- previous -> prev

---

## 6. Testing Instructions

Always run targeted package tests before executing the full suite:

```bash
# Run all tests in the workspace
colcon test --event-handlers console_direct+
colcon test-result --verbose

# Run unit tests for C++ perception component (GoogleTest)
colcon test --packages-select lekiwi_perception --event-handlers console_direct+

# Run unit tests for Python modules with pytest & coverage
colcon test --packages-select lekiwi_orchestrator --pytest-args -v

# Run pytest directly on lerobot_policy_server
pytest lerobot_policy_server/tests/ -v --cov=lerobot_policy_server
```

- **C++ Tests**: Add tests as `<package>/test/test_<feature>.cpp` using GoogleTest and register them in `CMakeLists.txt` using `ament_add_gtest`.
- **Python Tests**: Add tests as `<package>/test/test_<feature>.py` or `<package>/tests/`, registered via `ament_cmake_pytest` or standard `pytest`.
- **Deterministic Isolation**: Prefer deterministic unit tests that avoid physical cameras, Feetech servo serial ports, Hailo hardware, or external network services.

---

## 7. Diagnostics & Troubleshooting Patterns

When debugging robot behavior or communication failures:
1. **ROS 2 Communication**: Check topic availability and publication rates:
   ```bash
   ros2 node list
   ros2 topic hz /camera/color/image_raw
   ros2 topic echo /lekiwi/servo_telemetry --once
   ```
2. **Hardware Serial Bus**: Check port presence and permissions:
   ```bash
   ls -la /dev/lekiwi_serial
   # If access denied: sudo usermod -a -G dialout $USER
   ```
3. **Hailo NPU Runtime**: Verify PCIe enumeration and firmware status:
   ```bash
   hailortcli fw-control identify
   ```
4. **TF Coordinate Frame Tree**:
   ```bash
   ros2 run tf2_tools view_frames
   # Generates frames.pdf showing transforms between base_link, camera_link, and arm joints
   ```

---

## 8. Commit & Pull Request Guidelines

- Use concise, imperative Conventional Commit-style subjects (`feat:`, `fix:`, `refactor:`, `test:`, `docs:`, `chore:`), e.g. `feat(perception): add Hailo-8 YOLO v8 pose estimation` or `refactor(motion): optimize omni drive kinematics`.
- Keep commits scoped to a single concern.
- Pull requests should describe behavior and configuration changes, link related issues when applicable, list validation commands, and include visual artifacts (Mermaid diagrams, logs, or RViz screenshots) for launch, visualization, or camera-pipeline changes.
- Do not commit device-specific paths, secrets, model checkpoint weights, or generated trace/rosbag output.
