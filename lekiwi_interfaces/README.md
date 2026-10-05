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

### 3. `DriveStatus.msg`

Health, error counters, and communication status for the 3-wheel omnidirectional
mobile base driver:

| Field | Type | Description |
| :--- | :--- | :--- |
| `header` | `std_msgs/Header` | Timestamp and reference frame for telemetry tracking |
| `state` | `uint8` | Operating state (`UNKNOWN=0`, `INACTIVE=1`, `ACTIVE=2`, `ERROR=3`) |
| `bus_connected` | `bool` | Physical serial bus connectivity flag (`/dev/lekiwi_serial`) |
| `drive_enabled` | `bool` | Motor drive torque enable status |
| `watchdog_expired` | `bool` | True if velocity command timed out (> 100 ms) |
| `serial_error_count` | `uint32` | Cumulative count of packet CRC or framing errors |
| `consecutive_error_count` | `uint32` | Current count of consecutive failed serial transactions |
| `retry_count` | `uint32` | Total number of bus re-initialization retry attempts |
| `command_age_sec` | `float32` | Time elapsed since last valid velocity command [seconds] |
| `last_error` | `string` | Diagnostic description of the latest driver error |

### 4. `ServoTelemetry.msg`

High-frequency telemetry for the 3 omnidirectional base wheel servos. Array
indices correspond strictly to: `[0] = Left Wheel`, `[1] = Back Wheel`,
`[2] = Right Wheel`.

| Field | Type | Physical Unit | Description |
| :--- | :--- | :--- | :--- |
| `header` | `std_msgs/Header` | - | Telemetry sampling timestamp |
| `id` | `uint8[3]` | - | Hardware servo bus IDs |
| `raw_position` | `int32[3]` | ticks | Raw 12-bit encoder ticks (0..4095) |
| `raw_velocity` | `int32[3]` | ticks/s | Raw internal velocity register |
| `raw_load` | `int32[3]` | - | Raw motor torque load register |
| `position_rad` | `float64[3]` | rad | Calibrated joint angle per REP-103 |
| `velocity_rad_s` | `float64[3]` | rad/s | Calibrated joint angular velocity per REP-103 |
| `current_ma` | `float32[3]` | mA | Measured motor phase current |
| `voltage_v` | `float32[3]` | V | Servo input power supply voltage |
| `temperature_c` | `uint8[3]` | deg C | Internal microcontroller temperature |
| `status` | `uint8[3]` | - | Operating status register bitmask |
| `moving` | `bool[3]` | - | Physical movement indicator |
| `servo_error` | `uint8[3]` | - | Hardware alarm error bitmask |
| `online` | `bool[3]` | - | Serial bus heartbeat response flag |

### 5. `ChessMoveDetails.msg`

Encapsulates algebraic, kinematic, and semantic attributes of a single FIDE
chess move:

| Field | Type | Description |
| :--- | :--- | :--- |
| `uci` | `string` | Standard UCI move notation (e.g. `"e2e4"`, `"e7e8q"`, `"e1g1"`) |
| `san` | `string` | Standard FIDE notation (e.g. `"e4"`, `"Nf3"`, `"O-O"`, `"e8=Q#"`) |
| `from_square` | `string` | Origin square on chessboard grid (e.g. `"e2"`) |
| `to_square` | `string` | Destination square on chessboard grid (e.g. `"e4"`) |
| `piece_type` | `string` | Piece type: `"pawn"`, `"knight"`, `"bishop"`, `"rook"`, `"queen"`, `"king"` |
| `piece_color` | `string` | Color to move: `"w"` (White) or `"b"` (Black) |
| `promotion_piece`| `string` | Promoted piece type upon reaching rank 8/1 |
| `is_capture` | `bool` | True if this move captures an opposing piece |
| `captured_piece_type` | `string` | Type of piece removed during capture |
| `captured_square` | `string` | Square of captured piece (differs from `to_square` in en-passant) |
| `is_en_passant` | `bool` | Indicates an en-passant pawn capture |
| `is_castling` | `bool` | Indicates a kingside or queenside castling move |
| `castling_rook_from` | `string` | Initial square of participating castling rook (`"h1"` / `"a1"`) |
| `castling_rook_to` | `string` | Final square of participating castling rook (`"f1"` / `"d1"`) |

### 6. `ChessGameStatus.msg`

Overall game state, board evaluation, and match orchestration phase:

| Field | Type | Description |
| :--- | :--- | :--- |
| `header` | `std_msgs/Header` | Timestamp of latest board state update |
| `full_fen` | `string` | Standard 6-field Forsyth-Edwards Notation (FEN) |
| `last_move_details` | `ChessMoveDetails` | Semantic details of the latest executed move |
| `best_move_details` | `ChessMoveDetails` | Engine recommendation for the active turn |
| `eval_centipawns` | `int32` | Stockfish score (+ = White advantage, - = Black) |
| `active_color` | `string` | Color to move: `"w"` (White) or `"b"` (Black) |
| `game_phase` | `uint8` | Phase (`WAITING_PLAYER=0`, `THINKING=1`, `READY=2`, `EXECUTING=3`, `GAME_OVER=4`) |
| `is_board_stable` | `bool` | True if vision markers are clear without optical blur |
| `is_legal_move` | `bool` | True if detected move conforms to FIDE chess rules |
| `is_check` | `bool` | True if active player's King is in check |
| `is_checkmate` | `bool` | True if active player has no legal moves and is in check |
| `is_draw` | `bool` | True if draw condition reached (stalemate, repetition) |

---

## Service Definitions (`srv/`)

### 1. `SetCamMode.srv`

Requests a transition of the active camera streaming and pipeline operating mode:
- **Request**: `CameraMode requested_mode`
- **Response**: `bool success`, `CameraMode applied_mode`, `string message`

### 2. `SetPerceptionContext.srv`

Configures the active perception context, gating GStreamer feeds and Hailo-8 NPU:
- **Request**: `PerceptionContext requested_context`
- **Response**: `bool success`, `PerceptionContext applied_context`, `string message`

### 3. `SetTorqueEnabled.srv`

Controls motor torque power for robot joint subsystems (arm vs base wheels):
- **Constants**:
  - `uint8 TARGET_ALL = 0`
  - `uint8 TARGET_ARM = 1`
  - `uint8 TARGET_BASE = 2`
- **Request**: `uint8 target`, `bool enabled`, `bool toggle` (default `false`)
- **Response**: `bool success`, `string message`

### 4. `ResetMotorBus.srv`

Resets serial UART communication buffers and re-initializes bus servo controllers:
- **Request**: (empty)
- **Response**: `bool success`, `string message`

### 5. `CheckMoveFeasibility.srv`

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
  - `string instruction`
  - `string piece_type`
  - `string from_square`
  - `string to_square`
  - `geometry_msgs/Point pick_point` [meters, REP-103]
  - `geometry_msgs/Point place_point` [meters, REP-103]
  - `bool is_capture`
  - `string target_frame` (default: `"chessboard_frame"`)
  - `sensor_msgs/JointState pick_ik_hint`
  - `sensor_msgs/JointState place_ik_hint`
- **Result**:
  - `bool success`
  - `string message`
  - `float32 execution_time_sec` [seconds]
- **Feedback**:
  - `string current_phase` (`"APPROACH_PICK"`, `"GRASP"`, `"LIFT"`,
    `"APPROACH_PLACE"`, `"RELEASE"`, `"RETRACT"`)
  - `float32 progress_percent` [0.0 to 100.0%]
