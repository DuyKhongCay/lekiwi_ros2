# `lekiwi_orchestrator`

High-level autonomous chess mission orchestrator, readiness-gated navigation startup, and camera operational mode state machine for LeKiwi mobile manipulation robot.

---

## 🏛️ Architecture & Design Patterns (GoF & SOLID)

`lekiwi_orchestrator` is designed adhering to Clean Code and GoF Design Patterns:

1. **Mediator Pattern (`ChessMissionOrchestrator`)**:
   - Central conductor coordinating referee signals, localization gating, and specialized domain subsystems.
   - Cleanly decoupled into 3 cohesive pillars: `move_pipeline.py`, `perception_manager.py`, and `motion_dispatcher.py`, alongside `health_monitor.py`.
2. **Move Pipeline & Micro Motion FSM (`move_pipeline.py`)**:
   - Pure domain staging (`StagePipelineBuilder`): maps `CheckMoveFeasibility.Response` and `ChessMoveGoal` to atomic `ExecutionStage` steps (`CLEAR`, `PICK`, `PLACE`, `MOVE`).
   - Move execution engine (`MovePipelineExecutor`): drives the Level 2 Micro Motion Sub-FSM, synchronizing Nav2 standoffs, arm manipulation actions, and perception context transitions.
3. **Perception Context & Viewpoint Geometry (`perception_manager.py`)**:
   - Hardware-aware context coordinator (`PerceptionContextCoordinator`): manages latched publication (`/perception_context`) and service `/orchestrator/set_perception_context`.
   - FOV & observation geometry calculations (`compute_observation_pose`, `generate_candidate_observation_poses`).
4. **Motion Dispatching, Nav2 Lifecycle & Active Observation (`motion_dispatcher.py`)**:
   - Action dispatching abstraction (`ActionDispatcherInterface`, `RosActionDispatcher`, `SimulatedActionDispatcher`): handles Nav2 goal dispatching, manipulation action execution, and watchdog monitoring with full test mockability.
   - Action dispatching abstraction (`ActionDispatcherInterface`, `RosActionDispatcher`): handles Nav2 goal dispatching (with `mock_nav2` tabletop bypass), manipulation action execution, and watchdog monitoring with full test mockability.
   - Nav2 lifecycle management: handles readiness-gated Nav2 lifecycle startup (`/lifecycle_manager_navigation/manage_nodes`), heartbeat timeout expiration, and backoff retries.
   - Active observation navigator (`ActiveObservationNavigator`): repositions robot base to alternative candidate viewpoints when board scanning is occluded or timed out.
5. **Consolidated Health & Self-Healing (`health_monitor.py`)**:
   - Manages TF readiness heartbeat lease (`readiness_timeout_sec`), critical vs un-gated state tracking, auto-recovery timer/service (`/orchestrator/recover`), and telemetry diagnostics.
6. **2-Level Hierarchical State Pattern (`fsm.py`)**:
   - **Level 1 (Macro FSM - `MacroMissionState`)**: High-level game turn ownership, localization gating, and match status:
     `BOOT_INITIALIZING` $\to$ `WAITING_FOR_TF_READY` $\to$ `WAITING_FOR_PLAYER_MOVE` $\to$ `EVALUATING_BEST_MOVE` $\to$ `CHECKING_REACHABILITY` $\to$ `EXECUTING_MOVE_PIPELINE` $\to$ `TURN_COMPLETED` $\to$ `GAME_OVER`.
   - **Level 2 (Micro Motion Sub-FSM - `MotionExecutionState`)**: Atomic stage progression inside `EXECUTING_MOVE_PIPELINE`:
     `IDLE` $\to$ `NAV_TO_CLEAR` $\to$ `CLEARING_PIECE` $\to$ `NAV_TO_PICK` $\to$ `PICKING_PIECE` $\to$ `NAV_TO_PLACE` $\to$ `PLACING_PIECE` $\to$ `POST_MOVE_VERIFYING` $\to$ `IDLE`.
7. **PerceptionContext Hardware Gating (`lekiwi_interfaces/msg/PerceptionContext`)**:
   - Hardware-aware vision gating for GStreamer valves, Hailo-8 NPU, and LeRobot VLA:
     * `IDLE_STANDBY (0)`: Low power, valves closed.
     * `TF_TRACKING_AND_NAV (1)`: `stereo_left` open for AprilTag/TF tracking during navigation; Hailo-8 inference OFF (saving compute and avoiding motion-blur artifacts).
     * `BOARD_STATE_SCAN (2)`: `stereo_left` active with Hailo-8 full ROI YOLO chess detection.
     * `MANIPULATION_ACTOR (3)`: `usb_wrist` camera open for LeRobot SmolVLA; Hailo-8 inference OFF.
     * `POST_MOVE_VERIFY (4)`: One-shot board scan verifying piece placement before concluding turn.
     * `CALIBRATION_STREAM (5)`: Raw camera streaming for extrinsics/intrinsics calibration.

---

## 🧩 Nodes & Executables

| Executable | Class | Purpose |
|---|---|---|
| `chess_mission_orchestrator` | `ChessMissionOrchestrator` | End-to-end mission loop manager, direct owner of `/perception_context`, self-healing recovery, and optional navigation startup |

---

## 📡 Interfaces & Communication

### Subscriptions
- `/system/nav_ready` (`std_msgs/msg/Bool`): Base navigation readiness heartbeat from `lekiwi_motion` (evaluated with lease expiry).
- `/system/grasp_ready` (`std_msgs/msg/Bool`): Arm manipulation precision readiness heartbeat from `lekiwi_motion`.
- `/chess/game_status` (`lekiwi_interfaces/msg/ChessGameStatus`): Match state, FIDE legality, and Stockfish `best_move`.

### Services & Actions
- `/workspace/check_move_feasibility` (`lekiwi_interfaces/srv/CheckMoveFeasibility`): Kinematics reachability & standoff pose query.
- `/orchestrator/recover` (`std_srvs/srv/Trigger`): Manual self-healing recovery service resetting `ERROR_FALLBACK`.
- `/orchestrator/set_perception_context` (`lekiwi_interfaces/srv/SetPerceptionContext`): Programmatic override of perception context.
- `/navigate_to_pose` (`nav2_msgs/action/NavigateToPose`): Nav2 standoff docking via ActionDispatcher.
- `/manipulation/execute_chess_move` (`lekiwi_interfaces/action/ExecuteChessMove`): Arm pick and place execution via ActionDispatcher.

### Publications
- `/perception_context` (`lekiwi_interfaces/msg/PerceptionContext`): Latched hardware-aware perception operational mode.
- `/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`): Telemetry on mission state, Nav & Grasp readiness, and recovery attempts.
- `~/status_markers` (`visualization_msgs/msg/MarkerArray`): 3D HUD billboard and mission status text in RViz.

---

## 🚀 Launch & Usage

The orchestration and readiness subsystem launch files and YAML configurations are centralized in `lekiwi_bringup`:

```bash
# Launch full orchestration layer from lekiwi_bringup
ros2 launch lekiwi_bringup orchestrator.launch.py

# Launch with autonomous mission execution enabled
ros2 launch lekiwi_bringup orchestrator.launch.py start_mission:=true
```

---

## 🧪 Testing

Run automated pytest unit tests:

```bash
colcon test --packages-select lekiwi_orchestrator --event-handlers console_cohesion+
colcon test-result --verbose
```
