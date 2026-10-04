# `lekiwi_manipulation`

Mock manipulation action server for the LeKiwi chess robot.

---

## 🏛️ Architecture & Nodes

`lekiwi_manipulation` provides a lightweight, pure-software simulation server for manipulation tasks:

- **`mock_policy_server` (`MockPolicyServer`)**:
  - Implements the `/manipulation/execute_chess_move` Action Server (`lekiwi_interfaces/action/ExecuteChessMove`).
  - Simulates the sequential atomic phases:
    `APPROACH_PICK` $\to$ `DESCEND_PICK` $\to$ `GRASP` $\to$ `LIFT` $\to$ `TRANSIT_PLACE` $\to$ `DESCEND_PLACE` $\to$ `RELEASE` $\to$ `RETRACT_STOW`.
  - Supports incremental sleep with preemption / graceful cancel handling and periodic progress feedback.
  - Publishes diagnostic status on `/diagnostics`.

---

## 📡 Interfaces & Communication

### Action Servers
- `/manipulation/execute_chess_move` (`lekiwi_interfaces/action/ExecuteChessMove`):
  - **Goal**: `instruction`, `from_square`, `to_square`, `is_capture`, `pick_ik_hint`, `place_ik_hint`.
  - **Result**: `success`, `message`, `execution_time_sec`.
  - **Feedback**: `current_phase`, `progress_percent`.

### Topics
- `/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`): Telemetry on manipulation health and active phase.

---

## 🚀 Launch & Usage

```bash
# Launch mock policy server with default parameters
ros2 launch lekiwi_manipulation manipulation.launch.py
```

---

## 🧪 Testing

Run pytest automated unit tests:

```bash
colcon test --packages-select lekiwi_manipulation --event-handlers console_cohesion+
colcon test-result --test-result-base build/lekiwi_manipulation --verbose
```

