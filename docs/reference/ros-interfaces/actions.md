# ROS 2 Actions Reference

This document defines the custom ROS 2 Action interfaces provided by `lekiwi_interfaces`.

---

## 1. `ComputeBestMove.action`

- **Providing Package**: `lekiwi_chess_master` (Component: `ChessEngineActionComponent`, Action Name: `/chess/compute_best_move`)
- **Consumer**: `lekiwi_orchestrator`
- **Purpose**: Requests the Stockfish 16 engine to evaluate the current board state and return the best move in UCI format.

### Definition (`lekiwi_interfaces/action/ComputeBestMove.action`):

```protobuf
# Goal
string fen                     # Standard 6-field FEN board string
uint32 think_time_ms           # Engine search time limit [ms] (0 = default)
uint32 depth                   # Max engine search depth [plies] (0 = unlimited)
---
# Result
bool success                   # True if valid move found
string best_move               # Best move in UCI format (e.g., "e2e4")
string ponder_move             # Predicted human response in UCI format
int32 eval_centipawns          # Position score (+ = White, - = Black) [cp]
bool is_mate                   # True if mate sequence detected
int32 mate_in_moves            # Moves to mate (+ = robot wins, - = robot loses)
string message                 # Engine status or error description
---
# Feedback
uint32 current_depth           # Current search depth [plies]
int32 current_eval             # Intermediate score [cp]
uint64 nodes_per_second        # Calculation rate [nodes/s]
uint32 elapsed_time_ms         # Elapsed search time [ms]
string current_pv              # Current principal variation line
```

---

## 2. `ExecuteChessMove.action`

- **Action Servers**: `lekiwi_motion` (Cartesian analytical kinematics), `lekiwi_inference` (LeRobot policy client), `chess_episode_recorder`
- **Consumer**: `lekiwi_orchestrator` (Action Client on `/manipulation/execute_chess_move`)
- **Purpose**: Commands execution of physical pick, place, transport, and capture primitives across 3D board coordinates.

### Definition (`lekiwi_interfaces/action/ExecuteChessMove.action`):

```protobuf
# Goal
uint8 OP_MOVE        = 0  # Pick at pick_point and place at place_point
uint8 OP_CLEAR_LEFT  = 1  # Pick at pick_point and discard to left onboard bin
uint8 OP_CLEAR_RIGHT = 2  # Pick at pick_point and discard to right onboard bin
uint8 OP_PICK        = 3  # Pick at pick_point and return to stow pose
uint8 OP_PLACE       = 4  # Retrieve from stow pose and place at place_point

uint8 operation                # Target atomic manipulation primitive (OP_*)
string instruction             # Active move instruction for recorder / telemetry (e.g. "e2e4")
geometry_msgs/Point pick_point # Grasp target in target_frame [m] (used in MOVE, CLEAR_*, PICK)
geometry_msgs/Point place_point# Placement target in target_frame [m] (used in MOVE, PLACE)
string target_frame            # TF frame for coordinates (default: "chessboard_frame")
float32[6] environment_state   # 6D spatial coordinates in base_footprint: [src(3), tgt(3)]
---
# Result
bool success                   # True if physical execution converged / succeeded
string message                 # Diagnostic detail or failure reason
float32 execution_time_sec     # Execution duration [s]
---
# Feedback
string current_phase           # Phase: "APPROACH", "GRASP", "LIFT", "POLICY_ROLLOUT", etc.
float32 progress_percent       # Execution progress [0.0..100.0%]
string instruction             # Active move instruction for recorder / telemetry
```

---

## 3. `ExecuteCollectionTask.action`

- **Action Server**: `lekiwi_orchestrator`
- **Consumer**: `chess_episode_recorder` (Campaign Conductor)
- **Purpose**: Dispatches automated end-to-end dataset collection episodes combining navigation, manipulation primitives, and camera post-verification.

### Definition (`lekiwi_interfaces/action/ExecuteCollectionTask.action`):

```protobuf
# Goal
lekiwi_interfaces/ChessMoveDetails move       # Move metadata (UCI, squares, piece, color, castling, en-passant)
bool force_pick_place                        # Force decomposition of single-base moves into PICK and PLACE
string clear_bin_side                        # Discard bin side: "left" or "right"
string[] bypass_steps                        # Step kinds to bypass physical execution: ["MOVE", "CLEAR", "PICK", "PLACE"]
---
# Result
bool success                                 # True if the task was successfully completed and verified
string final_fen                             # Verified board FEN from camera post-move
string message                               # Diagnostic details or error explanation
---
# Feedback
string current_stage                         # Execution phase: e.g. "NAVIGATING", "MANIPULATING", "POST_VERIFYING"
float32 progress_percent                     # Progress indicator (0.0 to 100.0)
```
