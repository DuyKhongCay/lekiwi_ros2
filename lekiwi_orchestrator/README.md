# `lekiwi_orchestrator`

High-level autonomous chess mission orchestrator, readiness-gated navigation startup, and camera operational mode state machine for LeKiwi mobile manipulation robot.

---

## 🏛️ Architecture & Design Patterns (GoF & SOLID)

`lekiwi_orchestrator` is designed adhering to Clean Code and GoF Design Patterns:

1. **Mediator & Facade Pattern (`ChessMissionOrchestrator`)**:
   - Central conductor coordinating perception (`lekiwi_perception`), FIDE referee & Stockfish (`lekiwi_chess_master`), reachability kinematics (`lekiwi_motion`), base navigation (`Nav2`), and arm execution (`lekiwi_manipulation`).
   - Delegates stage decomposition to `StagePipelineBuilder`, runtime health/lease/recovery to `NodeHealthMonitor`, and action client management to `ActionDispatcher`.
2. **Pure Domain Stage Pipeline (`StagePipelineBuilder`)**:
   - Pure domain service mapping `CheckMoveFeasibility.Response` and `ChessMoveGoal` to atomic `ExecutionStage` steps (`CLEAR`, `PICK`, `PLACE`, `MOVE`) for quiet, single/dual/triple-base, and capture maneuvers.
3. **Consolidated Health & Self-Healing (`NodeHealthMonitor`)**:
   - Manages TF readiness heartbeat lease (`readiness_timeout_sec`), critical vs un-gated state tracking, auto-recovery timer/service (`/orchestrator/recover`), and telemetry diagnostics.
4. **State Pattern (`MissionState` & `ChessFsmPolicy` in `fsm.py`)**:
   - Enforces legal transitions across the autonomous game lifecycle:
     `BOOT_INITIALIZING` $\to$ `WAITING_FOR_TF_READY` $\to$ `WAITING_FOR_PLAYER_MOVE` $\to$ `EVALUATING_BEST_MOVE` $\to$ `CHECKING_REACHABILITY` $\to$ `NAVIGATING_TO_STANDOFF` $\to$ `EXECUTING_MANIPULATION` $\to$ `TURN_COMPLETED` $\to$ `GAME_OVER`.
5. **Spatial kinematics delegation**:
   - All 3D metric coordinate conversions from algebraic squares and workspace reachability checks are handled centrally by `lekiwi_motion`.

---

## 🧩 Nodes & Executables

| Executable | Class | Purpose |
|---|---|---|
| `chess_mission_orchestrator` | `ChessMissionOrchestrator` | End-to-end mission loop manager, direct owner of `/camera_mode`, self-healing recovery, and optional navigation startup |

---

## 📡 Interfaces & Communication

### Subscriptions
- `/system/tf_ready` (`std_msgs/msg/Bool`): Readiness heartbeat from `lekiwi_motion`; consumers use a receipt-time expiry.
- `/chess/game_status` (`lekiwi_interfaces/msg/ChessGameStatus`): Match state, FIDE legality, and Stockfish `best_move`.

### Services & Actions
- `/workspace/check_move_feasibility` (`lekiwi_interfaces/srv/CheckMoveFeasibility`): Kinematics reachability & standoff pose query.
- `/orchestrator/recover` (`std_srvs/srv/Trigger`): Manual self-healing recovery service resetting `ERROR_FALLBACK`.
- `/navigate_to_pose` (`nav2_msgs/action/NavigateToPose`): Nav2 standoff docking via ActionDispatcher.
- `/manipulation/execute_chess_move` (`lekiwi_interfaces/action/ExecuteChessMove`): Arm pick and place execution via ActionDispatcher.

### Publications
- `/camera_mode` (`lekiwi_interfaces/msg/CameraMode`): Latched current vision pipeline mode (`STANDBY`, `NAVIGATING`, `CHESS_THINKING`, `MANIPULATION_LEROBOT`), published directly on state transitions.
- `/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`): Telemetry on mission state, TF health, and recovery attempts.

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
