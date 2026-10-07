# `lekiwi_manipulation`
 
Manipulation subsystem for LeKiwi: provides high-level AI policy interfaces and mock policy server. Hardware motion planning and trajectory execution are handled deterministically in C++ within `lekiwi_motion`.
 
---
 
## 🏛️ Architecture & Nodes
 
`lekiwi_manipulation` provides high-level policy interfaces:
 
- **`mock_policy_server` (`MockPolicyServer`)**:
  - Implements the `/manipulation/execute_chess_move` Action Server (`lekiwi_interfaces/action/ExecuteChessMove`).
  - Simulates the sequential atomic phases for rapid testing without physical robot motion:
    `APPROACH_PICK` $\to$ `DESCEND_PICK` $\to$ `GRASP` $\to$ `LIFT` $\to$ `TRANSIT_PLACE` $\to$ `DESCEND_PLACE` $\to$ `RELEASE` $\to$ `RETRACT_STOW`.
  - Supports incremental sleep with preemption / graceful cancel handling and periodic progress feedback.
  - Publishes diagnostic status on `/diagnostics`.
- **Real Hardware Execution**:
  - Delegated to `lekiwi_motion::ManipulationActionServer` and `lekiwi_motion::CartesianServiceNode` (C++). Launchable via `ros2 launch lekiwi_bringup orchestrator.launch.py use_mock:=false`.

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

Manipulation is unified under `lekiwi_bringup`:

```bash
# Launch orchestrator with mock manipulation server (testing / simulation)
ros2 launch lekiwi_bringup orchestrator.launch.py use_mock:=true

# Launch orchestrator with real hardware C++ manipulation nodes
ros2 launch lekiwi_bringup orchestrator.launch.py use_mock:=false
```

---

## 🧪 Testing

Run pytest automated unit tests:

```bash
colcon test --packages-select lekiwi_manipulation --event-handlers console_cohesion+
colcon test-result --test-result-base build/lekiwi_manipulation --verbose
```

