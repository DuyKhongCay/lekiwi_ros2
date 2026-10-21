---
post_title: 'lekiwi_orchestrator'
author1: 'duykhongcay'
post_slug: 'lekiwi-orchestrator'
microsoft_alias: 'duykhongcay'
featured_image: ''
categories:
  - robotics
tags:
  - ros2
  - orchestrator
  - state-machine
  - active-perception
  - mobile-manipulation
  - clean-code
ai_note: 'Documentation updated following technical writer standards'
summary: 'High-level autonomous chess mission orchestrator, readiness-gated navigation startup, active observation, and perception state machine for LeKiwi robot'
post_date: '2026-10-05'
---

## Overview

The `lekiwi_orchestrator` package serves as the central mission coordinator and
subsystem mediator for the LeKiwi Autonomous Chess Playing Robot. It orchestrates
FIDE game referee signals, localization gating, camera perception contexts,
active observation repositioning, and coordinated mobile manipulation.

The package implements the **Mediator** and **Facade** design patterns within a
strictly decoupled, acyclic, and high-cohesion 4-layer architecture:
- `core`: Pure domain models, hierarchical state machines, parameter schemas, and SE(2) / time utilities.
- `planning`: Board geometry calculation, move decomposition, and active geodesic observation navigation.
- `execution`: ROS 2 action/service motion clients, multi-stage step sequencers, and turn workflows.
- `supervision`: Health lease supervision, 3D HUD markers, perception lifecycle gating, and top-level orchestrator node.

All modules are maintained with **Cognitive Complexity $\le 8$**, adhering to Clean
Code, SOLID principles, and ROS 2 asynchronous execution standards.

---

## System Architecture and Design Patterns

`ChessMissionOrchestrator` coordinates specialized subsystems across the 4 architectural layers:

```mermaid
flowchart TD
    subgraph CoreLayer ["1. Core Layer (Pure Domain & Utilities)"]
        MODELS["mission_types.py<br/>(Contracts & Goals)"]
        FSM["fsm.py<br/>(3-Tier HFSM Matrix)"]
        PARAMS["parameters.py<br/>(Type-safe Parameters)"]
        UTILS["utils.py<br/>(SE(2) Math & Time Helpers)"]
    end

    subgraph PlanningLayer ["2. Planning Layer"]
        GEOM["board_geometry.py<br/>(Polar Coords & Geodesic Ranking)"]
        PLAN["move_planner.py<br/>(Atomic MoveStep Builder)"]
        OBS["obs_navigator.py<br/>(Viewpoint Tour Scheduler)"]
    end

    subgraph ExecutionLayer ["3. Execution Layer"]
        MOTION["motion_client.py<br/>(Nav2 & Arm Action Gateways)"]
        SEQ["move_sequencer.py<br/>(Pipeline Execution & Self-Healing)"]
        WORKFLOW["turn_workflow.py<br/>(Referee & Verification Host)"]
    end

    subgraph SupervisionLayer ["4. Supervision Layer"]
        HEALTH["health_supervisor.py<br/>(Health Leases & 3D HUD)"]
        PERC["perception_context.py<br/>(Hardware Gating & Latched Topic)"]
        NODE["orchestrator_node.py<br/>(ChessMissionOrchestrator Facade)"]
    end

    CoreLayer --> PlanningLayer & ExecutionLayer & SupervisionLayer
    PlanningLayer --> ExecutionLayer & SupervisionLayer
    ExecutionLayer --> SupervisionLayer
```

### Concurrency and Thread-Safety Model

The orchestrator operates within a `MultiThreadedExecutor` with rigorous
thread-safety guarantees:

- **State Synchronization (`_state_lock`)**: All macro FSM transitions,
  move goal caches, and recovery checkpoints are guarded by a reentrant mutex
  (`threading.RLock`).
- **Callback Group Partitioning**:
  - `_cb_group_sub` (`MutuallyExclusiveCallbackGroup`): Serializes fast topic
    callbacks (`/system/nav_ready`, `/system/grasp_ready`, `/chess/game_status`).
  - `_cb_group_client` (`ReentrantCallbackGroup`): Drives asynchronous action
    clients and service invocations concurrently, eliminating deadlock during
    nested ROS 2 calls.
- **Open Call Pattern**: State transitions release `_state_lock` before
  invoking long-running side effects (such as `_move_sequencer.cancel()` or
  recovery timer scheduling) to prevent lock contention with active ROS 2
  action feedback threads.

---

## Hierarchical Finite State Machine (HFSM)

The mission lifecycle is structured into a 3-tier hierarchical state machine
coupled with hardware-aware camera perception gating:

### 1. Macro Mission Lifecycle (Level 1 FSM & Auto-Recovery)

```mermaid
flowchart TD
    BOOT(["<b>BOOT_INITIALIZING</b> (0)<br/>System bootstrap & ROS 2 interface init"])
    WAIT_TF["<b>WAITING_FOR_TF_READY</b> (1)<br/><i>Perception: TF_TRACKING_AND_NAV</i><br/>Dual-odometry covariance audit & TF chain stabilization"]
    WAIT_PLAYER["<b>WAITING_FOR_PLAYER_MOVE</b> (2)<br/><i>Perception: BOARD_STATE_SCAN</i><br/>Awaiting referee signal or opponent move detection"]
    EVAL["<b>EVALUATING_BEST_MOVE</b> (3)<br/><i>Perception: BOARD_STATE_SCAN</i><br/>Stockfish / LeRobot next move recommendation"]
    CHK_REACH["<b>CHECKING_REACHABILITY</b> (4)<br/><i>Service: /workspace/check_move_feasibility</i><br/>5-DoF analytical IK reachability & standoff query"]
    EXEC_PIPE["<b>EXECUTING_MOVE_PIPELINE</b> (5)<br/><i>Perception: MANIPULATION_ACTOR</i><br/>Sequencing Nav2 docking & arm trajectory phases"]
    VERIFY["<b>POST_MOVE_VERIFYING</b> (6)<br/><i>Perception: POST_MOVE_VERIFY</i><br/>Overhead vision confirmation; triggers active observation if occluded"]
    TURN_DONE["<b>TURN_COMPLETED</b> (7)<br/>Turn token handoff back to match arbiter"]
    GAME_OVER(["<b>GAME_OVER</b> (8)<br/>Checkmate / Draw terminal state"])
    ERR(["<b>ERROR_FALLBACK</b> (9)<br/>Preserves pending move; halts actuator motion"])

    BOOT --> WAIT_TF
    WAIT_TF --> WAIT_PLAYER
    WAIT_PLAYER --> EVAL
    EVAL --> CHK_REACH
    CHK_REACH --> EXEC_PIPE
    EXEC_PIPE --> VERIFY
    VERIFY --> TURN_DONE
    TURN_DONE --> WAIT_PLAYER
    TURN_DONE --> GAME_OVER

    %% Failures and Auto-Recovery
    WAIT_TF & WAIT_PLAYER & EVAL & CHK_REACH & EXEC_PIPE & VERIFY -.->|"Timeout / TF Drop / Grasp Loss"| ERR
    ERR -.->|"Auto-Recovery Retry (Attempts &le; 3)"| WAIT_TF
```

### 2. Motion Execution & Step Decomposition (Level 2 & Level 3 Pipeline)

```mermaid
flowchart TD
    subgraph Level2 ["Level 2: Motion Execution Sub-FSM (move_sequencer.py)"]
        direction TB
        M_IDLE["<b>IDLE</b><br/>Awaiting atomic move dispatch"]
        M_NAV_CLEAR["<b>NAV_TO_CLEAR</b><br/>Nav2 base docking at captured piece standoff"]
        M_CLEAR["<b>CLEARING_PIECE</b><br/>Arm pick & discard opponent piece"]
        M_NAV_PICK["<b>NAV_TO_PICK</b><br/>Nav2 base docking at source square standoff"]
        M_PICK["<b>PICKING_PIECE</b><br/>Arm approach, grasp & lift player piece"]
        M_NAV_PLACE["<b>NAV_TO_PLACE</b><br/>Nav2 base docking at target square standoff"]
        M_PLACE["<b>PLACING_PIECE</b><br/>Arm descend, insert piece & gripper retract"]
        M_NAV_OBS["<b>NAV_TO_OBS</b><br/>Nav2 base retreat to standoff observation perimeter"]

        M_IDLE --> M_NAV_CLEAR
        M_NAV_CLEAR --> M_CLEAR
        M_CLEAR --> M_NAV_PICK
        M_IDLE --> M_NAV_PICK
        M_NAV_PICK --> M_PICK
        M_PICK --> M_NAV_PLACE
        M_NAV_PLACE --> M_PLACE
        M_PLACE --> M_NAV_OBS
        M_NAV_OBS --> M_IDLE
    end

    subgraph Level3 ["Level 3: Micro Move Steps (move_planner.py &bull; StepKind)"]
        direction TB
        S_CLEAR["<b>CLEAR Step</b>: Approach &rarr; Grip capture &rarr; Discard bin transit"]
        S_PICK["<b>PICK Step</b>: Pre-grasp standoff &rarr; Grasp pose &rarr; Vertical lift"]
        S_PLACE["<b>PLACE Step</b>: Lower to board &rarr; Release gripper &rarr; Stow pose"]
        S_MOVE["<b>MOVE Step</b>: Monolithic pick-and-place without intermediate stow"]
        S_OBS["<b>OBSERVATION Step</b>: Geodesic S^1 active vantage repositioning"]
    end

    M_CLEAR -.->|"Realizes"| S_CLEAR
    M_PICK -.->|"Realizes"| S_PICK
    M_PLACE -.->|"Realizes"| S_PLACE
    M_NAV_OBS -.->|"Realizes"| S_OBS
```

### 3. Hardware-Aware Perception Context Gating (perception_context.py)

```mermaid
flowchart TD
    subgraph Modes ["Operational Vision Modes (lekiwi_interfaces/msg/PerceptionContext)"]
        direction TB
        P0["<b>Mode 0: IDLE_STANDBY</b><br/>GStreamer valves closed &bull; 0% CPU/NPU consumption"]
        P1["<b>Mode 1: TF_TRACKING_AND_NAV</b><br/>Stereo Left camera + AprilTag PnP &bull; Hailo NPU OFF"]
        P2["<b>Mode 2: BOARD_STATE_SCAN</b><br/>Overhead camera + Chessboard PnP &bull; Hailo-8 FEN ON"]
        P3["<b>Mode 3: MANIPULATION_ACTOR</b><br/>USB Wrist camera active &bull; LeRobot VLA policy execution"]
        P4["<b>Mode 4: POST_MOVE_VERIFY</b><br/>Stereo Left camera 1-shot piece placement verification"]
        P5["<b>Mode 5: CALIBRATION_STREAM</b><br/>Raw unrectified image streams for hand-eye calibration"]

        P0 --> P1
        P1 --> P2
        P2 --> P3
        P3 --> P4
        P4 --> P2
        P1 --> P5
    end
```

---

## Active Observation Repositioning Strategy

When visual occlusions or board state ambiguities occur, `ObsNavigator` executes
a **Direct Geodesic Viewpoint Scheduling Strategy** along the circular standoff boundary
($S^1$):

1. **Direct Geodesic Viewpoint Positioning**:
   - Eliminates uncalibrated intermediate radial waypoints, ensuring every observation
     pose corresponds to a tested, occlusion-free sensor viewing angle.
2. **Azimuth Open-Tour Scheduling (`rank_by_azimuth`)**:
   - Computes geodesic angular distances along the circle from calibrated candidate offsets
     ($\Delta \theta \in [0^\circ, \pm 18^\circ]$ for relocalization;
     $\Delta \theta \in [0^\circ, \pm 18^\circ, \pm 32^\circ]$ for post-move verification).
   - Solves the global minimal arc open tour from the robot's current polar coordinates,
     preventing back-and-forth oscillations across viewpoints.

---

## Subsystem Modules

| Module | Primary Classes / Symbols | Architectural Responsibility |
| :--- | :--- | :--- |
| `core/mission_types.py` | `ChessMoveGoal`, `ActionResult`, `ObservationIntent` | Immutable data contracts, factory parsers, and execution results. |
| `core/fsm.py` | `MacroMissionState`, `MotionExecutionState`, transitions | 3-tier state enums, state validation matrix, and illegal transition guards. |
| `core/parameters.py` | `OrchestratorParameters` | Node parameter declaration, strict validation, and cached configuration. |
| `core/utils.py` | `create_pose2d_stamped`, `normalize_angle`, timestamps | Planar SE(2) mathematics, timestamp normalization, and header conversions. |
| `planning/board_geometry.py` | `RankedViewpoint`, `rank_by_azimuth` | Polar coordinate resolution and minimal open-tour viewpoint scheduling. |
| `planning/move_planner.py` | `MovePlanBuilder`, `MoveStep`, `StepKind` | Kinematic stage generation (`CLEAR`, `PICK`, `PLACE`, `MOVE`, `OBSERVATION`). |
| `planning/obs_navigator.py` | `ObsNavigator` | Autonomous multi-viewpoint direct active observation repositioning. |
| `execution/motion_client.py` | `RosMotionClient`, `MotionClient` | Nav2 & arm manipulation action clients with watchdog timeouts. |
| `execution/move_sequencer.py` | `MoveSequencer` | Sequential stage advancement, watchdog enforcement, and grasp self-healing. |
| `execution/turn_workflow.py` | `GameStatusHandler`, `MoveWorkflow`, `PostMoveVerifier` | Referee processing, turn lifecycle orchestration, and verification. |
| `supervision/perception_context.py` | `PerceptionContextManager` | GStreamer valve & Hailo-8 camera operational context switching. |
| `supervision/health_supervisor.py` | `HealthSupervisor`, `StatusMarkerBuilder` | Readiness heartbeat lease tracking, auto-recovery timers, and 3D HUD. |
| `supervision/orchestrator_node.py` | `ChessMissionOrchestrator` | Top-level node facade, dependency wiring, and ROS 2 lifecycle coordination. |

---

## ROS 2 Interfaces

### Subscribed Topics

| Topic | Message Type | QoS | Semantic Purpose |
| :--- | :--- | :--- | :--- |
| `/system/nav_ready` | `std_msgs/msg/Bool` | Transient Local, Depth 1 | Navigation readiness heartbeat lease from `lekiwi_navigation`. |
| `/system/grasp_ready` | `std_msgs/msg/Bool` | Transient Local, Depth 1 | Arm IK and visual grasp readiness lease from `lekiwi_arm_plan`. |
| `/chess/game_status` | `lekiwi_interfaces/msg/ChessGameStatus` | Volatile, Depth 10 | Referee match status, active turn, checkmate flags, and best move. |

### Published Topics

| Topic | Message Type | QoS | Semantic Purpose |
| :--- | :--- | :--- | :--- |
| `/perception_context` | `lekiwi_interfaces/msg/PerceptionContext` | Transient Local, Depth 1 | Active vision pipeline mode (gating GStreamer / Hailo-8 streams). |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Volatile, Depth 10 | Node health telemetry, readiness status, and recovery metrics. |
| `~/status_markers` | `visualization_msgs/msg/MarkerArray` | Volatile, Depth 10 | RViz 3D text HUD billboard anchored above `base_footprint`. |

### Services and Actions

| Name | Interface Type | Role | Purpose |
| :--- | :--- | :--- | :--- |
| `/workspace/check_move_feasibility` | `lekiwi_interfaces/srv/CheckMoveFeasibility` | Client | Queries workspace reachability and IK feasibility before motion. |
| `/orchestrator/recover` | `std_srvs/srv/Trigger` | Server | Manual recovery endpoint to clear `ERROR_FALLBACK` state. |
| `/orchestrator/set_perception_context` | `lekiwi_interfaces/srv/SetPerceptionContext` | Server | Programmatic override of active perception operational context. |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Client | Dispatches radial entry or azimuth repositioning goals to Nav2. |
| `/manipulation/execute_chess_move` | `lekiwi_interfaces/action/ExecuteChessMove` | Client | Dispatches trajectory and gripper goals to arm manipulation server. |

---

## Node Parameters

Parameters can be overridden via YAML configuration files or launch arguments:

| Parameter | Type | Default | Unit / Description |
| :--- | :--- | :--- | :--- |
| `robot_color` | `string` | `"b"` | Robot player color (`"w"` for White, `"b"` for Black). |
| `board_frame` | `string` | `"chessboard_frame"` | Coordinate frame ID of the chessboard origin. |
| `map_frame` | `string` | `"map"` | Global map coordinate frame ID. |
| `navigation` | `bool` | `true` | Enables real Nav2 action dispatching (false activates mock). |
| `feasibility_timeout_sec` | `double` | `5.0` | Timeout in seconds for reachability service response. |
| `action_timeout_sec` | `double` | `60.0` | Watchdog timeout for Nav2 and manipulation action completion. |
| `readiness_timeout_sec` | `double` | `1.0` | Heartbeat expiration threshold for navigation and grasp readiness. |
| `chassis_settle_timeout_sec` | `double` | `2.0` | Stabilization pause duration between base docking/retreat and arm/vision operations. |
| `recovery.auto_recovery_enabled` | `bool` | `true` | Enables automatic recovery retry timer upon `ERROR_FALLBACK`. |
| `recovery.auto_recovery_timeout_sec` | `double` | `5.0` | Delay before triggering auto-recovery transition. |
| `recovery.max_recovery_attempts` | `int` | `3` | Maximum consecutive self-healing attempts before lockout. |
| `observation.standoff_distance` | `double` | `0.65` | Target radial observation distance $R$ from board center (meters). |
| `observation.scan_timeout_sec` | `double` | `6.0` | Timeout allowed for overhead board FEN confirmation. |
| `observation.angle_offsets.relocalize` | `double[]` | `[0.0, -0.314, 0.314]` | Candidate azimuth offsets (rad) for visual relocalization. |
| `observation.angle_offsets.post_move_verify` | `double[]` | `[0.0, -0.314, 0.314, -0.558, 0.558]` | Candidate azimuth offsets (rad) for post-move verification. |
| `visualization.enabled` | `bool` | `true` | Enables RViz 3D status HUD billboard marker publishing. |
| `visualization.topic` | `string` | `"~/status_markers"` | Topic name for visualization markers. |
| `visualization.hud_z_offset` | `double` | `0.35` | Height offset (meters) of HUD billboard above `robot_frame`. |
| `visualization.robot_frame` | `string` | `"base_footprint"` | Reference frame for status HUD anchoring. |

---

## Fault Tolerance and Self-Healing

The orchestrator incorporates automated self-healing mechanisms:

1. **TF Readiness Heartbeat Guard**: If TF tracking drops or `/system/nav_ready`
   times out (> 1.0 s), the orchestrator immediately transitions to
   `ERROR_FALLBACK` to halt mechanical motion.
2. **Pending Move Preservation**: When transitioning into `ERROR_FALLBACK`, any
   unfulfilled chess move is preserved in `_pending_recovery_move`. Once Nav2
   readiness is re-confirmed, the orchestrator automatically resumes the pending
   move without human intervention.
3. **Escalated Auto-Recovery**: An asynchronous recovery timer triggers up to
   `max_recovery_attempts` (3 cycles). Recovery resets action goals, clears
   pipeline sequencers, sets perception to `TF_TRACKING_AND_NAV`, and returns to
   `WAITING_FOR_TF_READY`.

---

## Build, Testing, and Execution

### Build Package

```bash
cd ~/docker_ws
colcon build --packages-select lekiwi_orchestrator --symlink-install
```

### Architectural and Unit Verification

```bash
# Run unit test suite across modular layers (core, execution, planning, supervision)
pytest lekiwi_ros2/lekiwi_orchestrator/test/ -v

# Run tests by specific layer
pytest lekiwi_ros2/lekiwi_orchestrator/test/core/ -v
pytest lekiwi_ros2/lekiwi_orchestrator/test/planning/ -v
pytest lekiwi_ros2/lekiwi_orchestrator/test/execution/ -v
pytest lekiwi_ros2/lekiwi_orchestrator/test/supervision/ -v

# Run entire test suite via colcon
colcon test --packages-select lekiwi_orchestrator --event-handlers console_cohesion+
colcon test-result --verbose
```

### Launch Subsystem

The orchestration layer is launched via `lekiwi_bringup`:

```bash
# Launch orchestrator node with default parameters
ros2 launch lekiwi_bringup orchestrator.launch.py

# Launch with autonomous mission execution enabled for White player
ros2 launch lekiwi_bringup orchestrator.launch.py robot_color:=w start_mission:=true
```
