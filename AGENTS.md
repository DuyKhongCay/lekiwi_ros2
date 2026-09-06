# Repository Guidelines

## Project Structure & Module Organization

This is a ROS 2 workspace built with `ament_cmake` and `colcon`.

- `lekiwi_interfaces/`: ROS messages and services shared between packages.
- `lekiwi_control/`: Python control finite-state machine and ROS executables; tests are in `test/`.
- `lekiwi_perception/`: C++ GStreamer/Hailo camera and chess-vision components. Public headers live in `include/`, implementations in `src/`, runtime models/images in `resources/`, and GTests in `test/`.
- `lekiwi_bringup/`: launch files, runtime YAML configuration, and udev rules.
- `lekiwi_description/`: URDF robot model and CAD assets.
- `scripts/`: standalone diagnostic and image utilities.

Keep generated `build/`, `install/`, and `log/` directories out of source changes.

## File editing

- Always use the IDE's dedicated editing tools/APIs (`replace_file_content` for editing existing code) to ensure diff tracking is visible directly in chat.
- **Never** use shell commands (`cat << EOF`, `echo >`, `sed`, `tee`, `python script`, etc.) to write or overwrite source code files.
- Keep every edit focused and limited strictly to the requested files.
- Inspect the modified file and verify changes with the appropriate build or test command after editing.

## Agent-Specific Instructions

- Respond to users in Vietnamese; write code comments in English.
- Always use **Mermaid** syntax (` ```mermaid `) whenever drawing diagrams, charts, pipelines, or architectural graphs.
- **Strict Protection of Agent Skills**: The agent is strictly **FORBIDDEN** from modifying, deleting, overwriting, creating, or running commands that mutate files within `.agents/skills/`, `.agents/`, or `skills-lock.json`. The agent may ONLY read skill files. Any installation, modification, or deletion of skills must be done manually by the user.

## Build, Test, and Development Commands

- **Centralized Build Outputs**: All ROS packages (whether built from inside or outside the project, e.g. third-party or submodules) MUST always output their build artifacts directly into the project workspace root's `build/`, `install/`, and `log/` directories (`/root/docker_ws/build`, `/root/docker_ws/install`, `/root/docker_ws/log`). Never allow nested or separate `build/`/`install/` directories inside individual package subfolders. Always use `--build-base /root/docker_ws/build --install-base /root/docker_ws/install --log-base /root/docker_ws/log` (or run `colcon build` directly from `/root/docker_ws`).

From the workspace root, source your ROS 2 installation, then use:

```bash
colcon build --symlink-install --cmake-args -GNinja
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Always prioritize using the **Ninja** build generator (`--cmake-args -GNinja`) when building packages for faster and more efficient builds.

Build a focused package with `colcon build --symlink-install --packages-select lekiwi_perception --cmake-args -GNinja`. Run a robot stack after building with `ros2 launch lekiwi_bringup robot.launch.py`; use the individual launch files in `lekiwi_bringup/launch/` for narrower debugging.

## Coding Style & Naming Conventions

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

## Testing Guidelines

Add C++ tests as `test/test_<feature>.cpp` using GoogleTest and register them in `lekiwi_perception/CMakeLists.txt` with `ament_add_gtest`. Add control tests as `test/test_<feature>.py` for `ament_cmake_pytest`. Prefer deterministic unit tests that avoid physical cameras, Hailo hardware, or network services. Run the focused package tests before the full workspace suite.

## Commit & Pull Request Guidelines

Use concise, imperative Conventional Commit-style subjects, as in `feat: initialize robot description package` or `refactor: migrate control modules`. Keep commits scoped to one concern. Pull requests should describe behavior and configuration changes, link related issues when applicable, list validation commands, and include logs or screenshots for launch, visualization, or camera-pipeline changes. Do not commit device-specific paths, secrets, or generated trace output.
