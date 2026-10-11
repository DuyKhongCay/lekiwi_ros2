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

- **Action Servers**: `chess_episode_recorder` (records physical episode and forwards trajectory), `lekiwi_motion`
- **Consumer**: `lekiwi_orchestrator` (Action Client on `/manipulation/execute_chess_move`)
- **Purpose**: Commands execution of a physical pick-and-place chess move across board coordinates with 3D pick/place points.

### Definition (`lekiwi_interfaces/action/ExecuteChessMove.action`):

```protobuf
# Goal
string instruction             # Task description (e.g., "Pick e2, place e4")
string piece_type              # Piece type (e.g., "pawn", "knight")
string from_square             # Source square (e.g., "e2")
string to_square               # Target square (e.g., "e4")
geometry_msgs/Point pick_point # Grasp point in target_frame [m]
geometry_msgs/Point place_point# Placement point in target_frame [m]
bool is_capture                # True if move captures opponent piece
string target_frame            # TF frame for points (default: "chessboard_frame")
---
# Result
bool success                   # True if physical execution succeeded
string message                 # Result message or failure reason
float32 execution_time_sec     # Trajectory duration [s]
---
# Feedback
string current_phase           # Phase: "APPROACH_PICK", "GRASP", "LIFT", "APPROACH_PLACE", "RELEASE", "RETRACT"
float32 progress_percent       # Execution progress [0.0..100.0%]
```
