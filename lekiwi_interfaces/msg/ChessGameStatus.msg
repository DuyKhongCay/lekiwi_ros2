# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

std_msgs/Header header
string full_fen                    # Standard 6-field FEN board representation
ChessMoveDetails last_move_details # Details of last executed move (Green Arrow)
string active_color                # Turn color: "w" or "b"

# Perception & rule validation flags
bool is_board_synced               # True when physical board layout matches active session FEN
bool is_legal_move                 # True if physical move conforms to FIDE rules and advanced FEN
bool is_check                      # True if active King is in check
bool is_checkmate                  # True if checkmate reached
bool is_draw                       # True if draw condition reached
