---
post_title: 'lekiwi_interfaces'
author1: 'duykhongcay'
post_slug: 'lekiwi-interfaces'
microsoft_alias: 'duykhongcay'
featured_image: ''
categories:
  - robotics
tags:
  - ros2
  - interfaces
  - messages
  - services
  - actions
ai_note: 'Documentation updated following technical writer standards'
summary: 'ROS 2 message, service, and action definitions for LeKiwi robot perception, navigation, manipulation, and chess playing'
post_date: '2026-10-05'
---

## Overview

The `lekiwi_interfaces` package contains all shared ROS 2 interface definitions
(`.msg`, `.srv`, `.action`) for the LeKiwi autonomous mobile manipulator robot.
It serves as the decoupled API contract between perception, mobile base control,
arm manipulation, chess decision-making, and high-level task orchestration.

All spatial coordinates and kinematics follow ROS REP-103 conventions (meters,
radians, SI units), with explicit timestamping via `std_msgs/Header` on
spatial, sensor, and status messages.

---

## Message Definitions (`msg/`)

### 1. `CameraMode.msg`

Defines high-level camera streaming and pipeline operating modes for the robot
finite-state machine:

| Mode Constant | Value | Description | Active Camera Pipelines |
| :--- | :--- | :--- | :--- |
| `STANDBY` | `0` | Idle state; all streams and NPU models gated off | None (0% CPU/NPU) |
| `NAVIGATING` | `1` | Mobile base visual SLAM and obstacle detection | `stereo_left` |
| `CHESS_THINKING`| `2` | Chess perception mode; Hailo board FEN inference | `stereo_left` + Hailo NPU |
| `MANIPULATION_LEROBOT` | `3` | Robotic arm manipulation and VLA policy rollouts | `usb_wrist`, `usb_side`, `stereo_right` |

- **Field**: `uint8 value`

### 2. `PerceptionContext.msg`

Configures dynamic vision pipeline gating, GStreamer valve states, and Hailo-8
NPU inference tasks:

| Context Constant | Value | GStreamer Valve / Pipeline Configuration |
| :--- | :--- | :--- |
| `IDLE_STANDBY` | `0` | All GStreamer valves closed to conserve CPU/NPU |
| `TF_TRACKING_AND_NAV` | `1` | `stereo_left` open, AprilTag PnP active, Hailo NPU off |
| `BOARD_STATE_SCAN` | `2` | `stereo_left` open, chessboard PnP active, Hailo FEN inference active |
| `MANIPULATION_ACTOR` | `3` | `usb_wrist` active for LeRobot VLA policy execution, Hailo NPU off |
| `POST_MOVE_VERIFY` | `4` | `stereo_left` open for single-shot piece placement verification |
| `CALIBRATION_STREAM` | `5` | Raw uncompressed video stream for intrinsic and hand-eye calibration |

- **Field**: `uint8 value`

### 3. `ChessMoveDetails.msg`

Encapsulates algebraic, kinematic, and semantic attributes of a single FIDE
chess move:

| Field | Type | Description |
| :--- | :--- | :--- |
| `uci` | `string` | Standard UCI move notation (e.g. `"e2e4"`, `"e7e8q"`, `"e1g1"`) |
| `piece_type` | `string` | Moving piece, or `"moving:captured"` if capture (e.g. `"pawn"`, `"pawn:knight"`) |
| `piece_color` | `string` | Color to move: `"w"` (White) or `"b"` (Black) |
| `promotion_piece`| `string` | Promoted piece type upon reaching rank 8/1 (`"queen"`, etc.) |
| `is_capture` | `bool` | True if this move captures an opposing piece |
| `is_en_passant` | `bool` | Indicates an en-passant pawn capture |
| `is_castling` | `bool` | Indicates a kingside or queenside castling move |

### 4. `ChessGameStatus.msg`

Overall game state, board evaluation, and match orchestration phase:

| Field | Type | Description |
| :--- | :--- | :--- |
| `header` | `std_msgs/Header` | Timestamp of latest board state update |
| `full_fen` | `string` | Standard 6-field Forsyth-Edwards Notation (FEN) |
| `last_move_details` | `ChessMoveDetails` | Semantic details of the latest executed move |
| `active_color` | `string` | Color to move: `"w"` (White) or `"b"` (Black) |
| `is_board_synced` | `bool` | True when physical board layout matches active session FEN |
| `is_legal_move` | `bool` | True if detected move conforms to FIDE chess rules |
| `is_check` | `bool` | True if active player's King is in check |
| `is_checkmate` | `bool` | True if active player has no legal moves and is in check |
| `is_draw` | `bool` | True if draw condition reached (stalemate, repetition) |

---

## Service Definitions (`srv/`)

### 1. `SetPerceptionContext.srv`

Configures the active perception context, gating GStreamer feeds and Hailo-8 NPU:
- **Request**: `PerceptionContext requested_context`
- **Response**: `bool success`, `PerceptionContext applied_context`, `string message`

### 2. `SetTorqueEnabled.srv`

Controls motor torque power for robot joint subsystems (arm vs base wheels):
- **Constants**:
  - `uint8 TARGET_ALL = 0`
  - `uint8 TARGET_ARM = 1`
  - `uint8 TARGET_BASE = 2`
- **Request**: `uint8 target`, `bool enabled`, `bool toggle` (default `false`)
- **Response**: `bool success`, `string message`

### 3. `CheckMoveFeasibility.srv`

Evaluates reachability and calculates multi-segment navigation plans for chess moves:
- **Request**: `lekiwi_interfaces/ChessMoveDetails move`
- **Response**:
  - `uint8 plan_type` (`PLAN_ZERO_NAV=0`, `PLAN_SINGLE_BASE=1`, `PLAN_DUAL_BASE=2`,
    `PLAN_CAPTURE_ZERO_NAV=3`, `PLAN_CAPTURE_SINGLE_BASE=4`,
    `PLAN_CAPTURE_DUAL_BASE=5`, `PLAN_CAPTURE_TRIPLE_BASE=6`)
  - `bool feasible`
  - `geometry_msgs/PoseStamped clear_base_pose` (map frame)
  - `geometry_msgs/PoseStamped pick_base_pose` (map frame)
  - `geometry_msgs/PoseStamped place_base_pose` (map frame)
  - `geometry_msgs/Point clear_point` (chessboard_frame, meters)
  - `geometry_msgs/Point pick_point` (chessboard_frame, meters)
  - `geometry_msgs/Point place_point` (chessboard_frame, meters)
  - `string message`

---

## Action Definitions (`action/`)

### 1. `ComputeBestMove.action`

Computes the optimal chess move from an input FEN position using Stockfish:
- **Goal**:
  - `string fen`
  - `uint32 think_time_ms` [ms]
  - `uint32 depth` [plies]
- **Result**:
  - `bool success`
  - `string best_move` (UCI format)
  - `string ponder_move`
  - `int32 eval_centipawns`
  - `bool is_mate`
  - `int32 mate_in_moves`
  - `string message`
- **Feedback**:
  - `uint32 current_depth`
  - `int32 current_eval`
  - `uint64 nodes_per_second` [nodes/s]
  - `uint32 elapsed_time_ms` [ms]
  - `string current_pv`

### 2. `ExecuteChessMove.action`

Coordinates physical piece pick, transport, and place operations via robot arm:
- **Goal**:
  - `uint8 operation` (`OP_MOVE`, `OP_CLEAR_LEFT`, `OP_CLEAR_RIGHT`, `OP_PICK`, `OP_PLACE`)
  - `string instruction` (Semantic SmolVLA prompt, e.g. `"Move black pawn to target"`)
  - `geometry_msgs/Point pick_point` [meters]
  - `geometry_msgs/Point place_point` [meters]
  - `string target_frame` (default: `"chessboard_frame"`)
  - `float32[6] environment_state` (6D spatial coordinates in `base_footprint`: `[src(3), tgt(3)]`)
- **Result**:
  - `bool success`
  - `string message`
  - `float32 execution_time_sec` [seconds]
- **Feedback**:
  - `string current_phase` (`"POLICY_ROLLOUT"`, `"APPROACH"`, `"GRASP"`, etc.)
  - `float32 progress_percent` [0.0 to 100.0%]
  - `string instruction`
