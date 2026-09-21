# `lekiwi_orchestrator`

High-level autonomous chess mission orchestrator, readiness-gated navigation startup, and camera operational mode state machine for LeKiwi mobile manipulation robot.

---

## 🏛️ Architecture & Design Patterns (GoF & SOLID)

`lekiwi_orchestrator` is designed adhering to Clean Code and GoF Design Patterns:

1. **Mediator & Facade Pattern (`ChessMissionOrchestrator`)**:
   - Central conductor coordinating perception (`lekiwi_perception`), FIDE referee & Stockfish (`lekiwi_chess_master`), reachability kinematics (`lekiwi_control`), base navigation (`Nav2`), and arm execution (`lekiwi_manipulation`).
   - Keeps subsystems loosely coupled with zero direct cyclic dependencies.
2. **State Pattern (`MissionState` & `ChessFsmPolicy` in `fsm.py`)**:
   - Enforces legal transitions across the autonomous game lifecycle:
     `BOOT_INITIALIZING` $\to$ `WAITING_FOR_TF_READY` $\to$ `WAITING_FOR_PLAYER_MOVE` $\to$ `EVALUATING_BEST_MOVE` $\to$ `CHECKING_REACHABILITY` $\to$ `NAVIGATING_TO_STANDOFF` $\to$ `EXECUTING_MANIPULATION` $\to$ `TURN_COMPLETED` $\to$ `GAME_OVER`.
3. **Single Responsibility Principle (SRP) (`ChessboardCoordinateMapper`)**:
   - Pure mathematical and geometry translation module mapping FIDE notation ("a1".."h8") to Cartesian 3D $(x, y, z)$ on `chessboard_frame`.
   - 100% testable without ROS graph.
4. **Readiness consumption**:
   - The C++ gatekeeper belongs to `lekiwi_control`. Mission dispatch expires its readiness heartbeat after `readiness_timeout_sec`.
   - `NavigationStartup` starts Nav2 once after fresh readiness when task orchestration receives `start_navigation:=true`; service attempts have bounded timeouts and retry backoff.

---

## 🧩 Nodes & Executables

| Executable | Class | Purpose |
|---|---|---|
| `chess_mission_orchestrator` | `ChessMissionOrchestrator` | End-to-end mission loop manager |
| `task_orchestrator` | `TaskOrchestratorNode` | Bootstraps cameras, CameraMode FSM, and optional readiness-gated Nav2 startup |

---

## 📡 Interfaces & Communication

### Subscriptions
- `/system/tf_ready` (`std_msgs/msg/Bool`): Readiness heartbeat from `lekiwi_control`; consumers use a receipt-time expiry.
- `/chess/game_status` (`lekiwi_interfaces/msg/ChessGameStatus`): Match state, FIDE legality, and Stockfish `best_move`.

### Clients (Services & Actions)
- `/workspace/check_move_feasibility` (`lekiwi_interfaces/srv/CheckMoveFeasibility`): Kinematics reachability & standoff pose query.
- `/navigate_to_pose` (`nav2_msgs/action/NavigateToPose`): Nav2 standoff docking.
- `/manipulation/execute_chess_move` (`lekiwi_interfaces/action/ExecuteChessMove`): Arm pick and place execution.
- `/orchestrator/set_mode` (`lekiwi_interfaces/srv/SetCamMode`): Camera operational mode switching.

### Publications
- `/camera_mode` (`lekiwi_interfaces/msg/CameraMode`): Latched current vision pipeline mode (`STANDBY`, `NAVIGATING`, `CHESS_THINKING`, `MANIPULATION_LEROBOT`).
- `/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`): Telemetry on mission state and TF health.

---

## 🚀 Launch & Usage

```bash
# Launch full orchestration layer
ros2 launch lekiwi_orchestrator orchestrator.launch.py
```

---

## 🧪 Testing

Run automated pytest unit tests:

```bash
colcon test --packages-select lekiwi_orchestrator --event-handlers console_cohesion+
colcon test-result --verbose
```
