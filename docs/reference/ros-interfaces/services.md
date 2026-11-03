# ROS 2 Services Reference

This document defines custom ROS 2 Service contracts provided by `lekiwi_interfaces` and used across LeKiwi subsystems.

---

## 1. `CheckMoveFeasibility.srv`

- **Server Node**: `lekiwi_motion` (Service: `/motion/check_move_feasibility` or `/check_move_feasibility`)
- **Client Nodes**: `lekiwi_orchestrator`
- **Purpose**: Evaluates workspace reachability, analytical inverse kinematics, obstacle clearance, and plans required mobile base repositioning poses before commanding physical manipulation.

### Definition (`lekiwi_interfaces/srv/CheckMoveFeasibility.srv`):

```protobuf
# Request
lekiwi_interfaces/ChessMoveDetails move  # Target chess move entity
---
# Navigation & manipulation plan strategies
uint8 PLAN_ZERO_NAV            = 0  # 0 Nav2: Stationary pick & place (Quiet move)
uint8 PLAN_SINGLE_BASE         = 1  # 1 Nav2: Move to shared base pose -> Pick & Place
uint8 PLAN_DUAL_BASE           = 2  # 2 Nav2: B_pick -> Pick -> B_place -> Place
uint8 PLAN_CAPTURE_ZERO_NAV    = 3  # 0 Nav2: Clear -> Pick -> Place
uint8 PLAN_CAPTURE_SINGLE_BASE = 4  # 1 Nav2: Move to shared pose -> Clear -> Pick -> Place
uint8 PLAN_CAPTURE_DUAL_BASE   = 5  # 2 Nav2: Two intermediate base poses for capture
uint8 PLAN_CAPTURE_TRIPLE_BASE = 6  # 3 Nav2: B_clear -> Clear -> B_pick -> Pick -> B_place -> Place

uint8 plan_type                     # Selected strategy (PLAN_*)
bool feasible                       # True if inverse kinematics & path exist

# Mobile base target poses in map frame
geometry_msgs/PoseStamped clear_base_pose  # Base pose to clear captured piece
geometry_msgs/PoseStamped pick_base_pose   # Base pose to pick moving piece
geometry_msgs/PoseStamped place_base_pose  # Base pose to place moving piece

# 3D grasp/place coordinates in chessboard_frame [m]
geometry_msgs/Point clear_point            # Grasp point for captured piece
geometry_msgs/Point pick_point             # Grasp point for moving piece
geometry_msgs/Point place_point            # Release point for moving piece

string message                             # Feasibility assessment details
```

---

## 2. `SetTorqueEnabled.srv`

- **Server Node**: `lekiwi_motion` (TorqueManagerNode, Service: `/set_torque_enabled`)
- **Client Nodes**: CLI utilities, Calibration nodes (`lekiwi_calibration`), Teleoperation
- **Purpose**: Enables or disables motor bus holding torque for gravity compliance or manual lead-through teaching.

### Definition (`lekiwi_interfaces/srv/SetTorqueEnabled.srv`):

```protobuf
# Constants
uint8 TARGET_ALL=0             # All robot joints (arm + base)
uint8 TARGET_ARM=1             # Arm servos only (IDs 1..6)
uint8 TARGET_BASE=2            # Base wheel servos only (IDs 7..9)

# Request
uint8 target                   # Target subsystem (TARGET_ALL, TARGET_ARM, TARGET_BASE)
bool enabled                   # Target torque state: true=enable, false=disable (lead-through)
bool toggle false              # If true, flips current torque state regardless of enabled
---
# Response
bool success                   # True if command acknowledged by servos
string message                 # Diagnostic message or failure cause
```

---

## 3. `SetPerceptionContext.srv`

- **Server Node**: `lekiwi_perception` (Service: `/perception/set_context`)
- **Client Nodes**: `lekiwi_orchestrator`
- **Purpose**: Dynamically adjusts visual perception gating and target context (e.g. searching for chessboard vs. evaluating piece occupancy).

### Definition (`lekiwi_interfaces/srv/SetPerceptionContext.srv`):

```protobuf
# Request
PerceptionContext requested_context  # Desired perception pipeline context
---
# Response
bool success                         # True if pipeline gating succeeded
PerceptionContext applied_context    # Active perception context after transition
string message                       # Diagnostic status or failure cause
```

---

## 4. `GoToNamedPose.srv`

- **Server Node**: `lekiwi_motion` (CartesianJogServerNode, Service: `/cartesian_jog/go_to_named_pose`)
- **Client Nodes**: `lekiwi_orchestrator`, CLI diagnostic utilities
- **Purpose**: Synchronously transitions the 6-DoF arm to calibrated landmark poses (`"approach"`, `"stow"`, `"clear_left"`, `"clear_right"`).

### Definition (`lekiwi_interfaces/srv/GoToNamedPose.srv`):

```protobuf
# Request
string pose_name                     # Target named pose ("approach", "stow", "clear_left", "clear_right")
---
# Response
bool success                         # True if trajectory reached target pose
string message                       # Diagnostic detail or error message
```

---

## 5. `SetFen.srv`

- **Server Node**: `lekiwi_chess_master` (GameStateTracker, Service: `/chess/set_fen`)
- **Client Nodes**: `lekiwi_orchestrator`, referee debug scripts
- **Purpose**: Injects or updates the canonical FIDE board state string for game tracking and Stockfish engine synchronization.

### Definition (`lekiwi_interfaces/srv/SetFen.srv`):

```protobuf
# Request
string fen                           # Standard 6-field FEN or 1-field piece placement string
---
# Response
bool success                         # True if FEN syntax is valid and accepted
string canonical_fen                 # Canonical normalized FEN string
string active_color                  # Active player color ("w" or "b")
string message                       # Validation status or syntax error details
```


