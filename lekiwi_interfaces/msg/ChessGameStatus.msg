# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

# Match orchestration phases
uint8 PHASE_WAITING_PLAYER   = 0  # Waiting for human opponent move
uint8 PHASE_ROBOT_THINKING   = 1  # Stockfish engine calculating move
uint8 PHASE_ROBOT_READY      = 2  # Best move ready & kinematics verified
uint8 PHASE_ROBOT_EXECUTING  = 3  # Physical arm/base move in execution
uint8 PHASE_GAME_OVER        = 4  # Match ended (checkmate, draw, etc.)

std_msgs/Header header
string full_fen                    # Standard 6-field FEN board representation
ChessMoveDetails last_move_details # Details of last executed move
ChessMoveDetails best_move_details # Engine recommended move for active turn
int32 eval_centipawns              # Stockfish evaluation (+ = White, - = Black) [cp]
string active_color                # Turn color: "w" or "b"
uint8 game_phase                   # Current match phase (PHASE_*)

# Perception & rule validation flags
bool is_board_stable               # True if board vision markers are stable
bool is_legal_move                 # True if move conforms to FIDE rules
bool is_check                      # True if active King is in check
bool is_checkmate                  # True if checkmate reached
bool is_draw                       # True if draw condition reached
