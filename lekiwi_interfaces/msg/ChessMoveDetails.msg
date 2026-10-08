# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

# Move notation
string uci                     # UCI notation (e.g., "e2e4", "e7e8q", "e1g1")
string san                     # FIDE SAN notation (e.g., "e4", "Nf3", "O-O", "e8=Q#")

# Board coordinates & piece semantics
string from_square             # Origin square on 8x8 board (e.g., "e2")
string to_square               # Target square on 8x8 board (e.g., "e4")
string piece_type              # Moving piece: "pawn", "knight", "bishop", "rook", "queen", "king"
string piece_color             # Moving piece color: "w" or "b"
string promotion_piece         # Promoted piece: "queen", "knight", "rook", "bishop", or ""

# Physical manipulation attributes
bool is_capture                # True if move captures an opponent piece
string captured_piece_type     # Captured piece type, or "" if none
string captured_square         # Square of captured piece (differs in en-passant)

# Special move flags & kinematics
bool is_en_passant             # True for en-passant pawn capture
bool is_castling               # True for castling move
string castling_rook_from      # Castling rook start square (e.g., "h1" or "a1")
string castling_rook_to        # Castling rook target square (e.g., "f1" or "d1")
