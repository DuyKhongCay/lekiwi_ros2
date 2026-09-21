std_msgs/Header header

# Standard 6-field FEN representation of the board
string full_fen

# Last executed move in UCI format (e.g., "e2e4")
string last_move

# Engine proposed best move in UCI format (e.g., "e7e5")
string best_move

# Stockfish evaluation score in centipawns
int32 eval_centipawns

# Active turn color: "w" or "b"
string active_color

# Match & Robot execution phases
uint8 PHASE_WAITING_PLAYER   = 0
uint8 PHASE_ROBOT_THINKING   = 1
uint8 PHASE_ROBOT_READY      = 2
uint8 PHASE_ROBOT_EXECUTING  = 3
uint8 PHASE_GAME_OVER        = 4
uint8 game_phase

# Game validation flags
bool is_board_stable
bool is_legal_move
bool is_check
bool is_checkmate
bool is_draw

