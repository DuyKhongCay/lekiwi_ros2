# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

# Move notation (Standard UCI, e.g. "e2e4", "e7e8q", "e1g1")
string uci

# Piece semantics & unified piece type ("<moving>" or "<moving>:<captured>")
string piece_type              # Moving piece, or "moving:captured" if capture (e.g. "pawn", "pawn:knight")
string piece_color             # Moving piece color: "white" or "black"
string promotion_piece         # Promoted piece: "queen", "knight", "rook", "bishop", or ""

# Physical manipulation flags
bool is_capture                # True if move captures an opponent piece
bool is_en_passant             # True for en-passant pawn capture
bool is_castling               # True for castling move
