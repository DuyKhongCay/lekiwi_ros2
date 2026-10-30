# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""
Mission Domain Types and Value Objects for LeKiwi Orchestrator.

Decouples high-level motion clients, pipeline planners, sequencers, and application nodes
to prevent cyclic dependencies and adhere to the Dependency Inversion Principle.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING, Final

if TYPE_CHECKING:
    from lekiwi_interfaces.msg import ChessMoveDetails

try:
    from lekiwi_interfaces.msg import ChessMoveDetails
except ImportError:
    ChessMoveDetails = None  # type: ignore[assignment, misc]


@dataclass(frozen=True)
class ActionResult:
    """Normalized outcome of an action execution step.

    Encapsulates completion status, execution duration, and diagnostic message.
    """

    success: bool
    message: str = ""
    execution_time_sec: float = 0.0


class ObservationIntent(str, Enum):
    """Semantic observation goal for robot base relocation.

    Differentiates between post-move board state verification and EKF relocalization.
    """

    POST_MOVE_VERIFY = "POST_MOVE_VERIFY"
    RELOCALIZE = "RELOCALIZE"


_CASTLING_ROOK_MAP: Final[dict[str, tuple[str, str]]] = {
    "e1g1": ("h1", "f1"),
    "e1c1": ("a1", "d1"),
    "e8g8": ("h8", "f8"),
    "e8c8": ("a8", "d8"),
}


def _normalize_bin_side(clear_bin_side: str) -> str:
    """Normalize discard bin side to canonical 'left' or 'right'."""
    if isinstance(clear_bin_side, str) and clear_bin_side.strip().lower() == "left":
        return "left"
    return "right"


@dataclass(frozen=True)
class ChessMoveGoal:
    """Mission-level move domain entity aligned 1:1 with ChessMoveDetails interface."""

    uci: str
    piece_type: str = "pawn"
    piece_color: str = "white"
    promotion_piece: str = ""
    is_capture: bool = False
    is_en_passant: bool = False
    is_castling: bool = False

    # Collection directives
    force_pick_place: bool = False
    clear_bin_side: str = "right"
    bypass_steps: tuple[str, ...] = ()

    # --- Domain Computed Properties (Zero State Duplication) ---

    @property
    def from_square(self) -> str:
        """Origin square derived directly from UCI notation (e.g., 'e2')."""
        return self.uci[:2]

    @property
    def to_square(self) -> str:
        """Destination square derived directly from UCI notation (e.g., 'e4')."""
        return self.uci[2:4]

    @property
    def piece(self) -> str:
        """Moving piece type (e.g., 'pawn', 'knight')."""
        return (
            self.piece_type.split(":", 1)[0]
            if ":" in self.piece_type
            else (self.piece_type or "pawn")
        )

    @property
    def captured_piece(self) -> str:
        """Target captured piece type if move is a capture, else empty string."""
        return self.piece_type.split(":", 1)[1] if ":" in self.piece_type else ""

    @property
    def captured_square(self) -> str:
        """Accurately resolved square of captured piece (handles en-passant offset)."""
        if not self.is_capture:
            return ""
        if self.is_en_passant:
            return f"{self.to_square[0]}{self.from_square[1]}"
        return self.to_square

    @property
    def castling_rook_from(self) -> str:
        """Initial square of participating castling rook, or empty string."""
        return _CASTLING_ROOK_MAP.get(self.uci, ("", ""))[0] if self.is_castling else ""

    @property
    def castling_rook_to(self) -> str:
        """Destination square of participating castling rook, or empty string."""
        return _CASTLING_ROOK_MAP.get(self.uci, ("", ""))[1] if self.is_castling else ""

    @property
    def color_name(self) -> str:
        """Natural language color string ('white' or 'black') for instruction generation."""
        return "black" if self.piece_color.strip().lower() in ("b", "black") else "white"

    @property
    def opposing_color_name(self) -> str:
        """Natural language color string of the opposing side for capture instructions."""
        return "white" if self.color_name == "black" else "black"

    def to_ros_move_details(self) -> ChessMoveDetails:
        """Convert ChessMoveGoal to ROS 2 ChessMoveDetails message."""
        msg = ChessMoveDetails()
        msg.uci = self.uci
        msg.piece_type = self.piece_type
        msg.piece_color = self.piece_color
        msg.promotion_piece = self.promotion_piece
        msg.is_capture = self.is_capture
        msg.is_en_passant = self.is_en_passant
        msg.is_castling = self.is_castling
        return msg

    @classmethod
    def from_uci_or_details(
        cls,
        uci_move: str,
        move_details: ChessMoveDetails | ChessMoveGoal | None = None,
        force_pick_place: bool = False,
        clear_bin_side: str = "right",
        bypass_steps: tuple[str, ...] | list[str] = (),
    ) -> ChessMoveGoal | None:
        """Parse clean ChessMoveGoal from UCI string or rich ChessMoveDetails message.

        Extracts source, target, piece semantics, castling waypoints, and collection directives.

        Args:
            uci_move: Standard UCI move string (e.g., 'e2e4', 'e7e8q').
            move_details: Optional ROS 2 message or existing ChessMoveGoal instance.
            force_pick_place: Whether to force 1-base move into atomic PICK and PLACE.
            clear_bin_side: Target discard bin side ("left" or "right").
            bypass_steps: Step kinds to bypass physical execution.

        Returns:
            Populated ChessMoveGoal instance, or None if UCI string length is invalid (< 4).
        """
        if isinstance(move_details, ChessMoveGoal):
            return move_details

        cleaned_move = (
            move_details.uci
            if (move_details is not None and getattr(move_details, "uci", ""))
            else uci_move
        ).strip().lower()
        if len(cleaned_move) < 4:
            return None

        norm_bypass = tuple(bypass_steps)
        norm_bin = _normalize_bin_side(clear_bin_side)

        p_type = "pawn"
        p_color = "white"
        promo = cleaned_move[4:] if len(cleaned_move) > 4 else ""
        is_cap = False
        is_ep = False
        is_castle = False

        if move_details is not None:
            raw_c = move_details.piece_color.strip().lower() if move_details.piece_color else ""
            if raw_c in ("black", "b"):
                p_color = "black"
            elif raw_c in ("white", "w"):
                p_color = "white"
            elif raw_c:
                p_color = raw_c
            else:
                p_color = "white"

            p_type = move_details.piece_type.strip().lower() if move_details.piece_type else "pawn"
            promo = move_details.promotion_piece or promo
            is_cap = bool(move_details.is_capture)
            is_ep = bool(move_details.is_en_passant)
            is_castle = bool(move_details.is_castling)

        return cls(
            uci=cleaned_move,
            piece_type=p_type,
            piece_color=p_color,
            promotion_piece=promo,
            is_capture=is_cap,
            is_en_passant=is_ep,
            is_castling=is_castle,
            force_pick_place=force_pick_place,
            clear_bin_side=norm_bin,
            bypass_steps=norm_bypass,
        )


def parse_piece_from_fen(fen: str, square: str) -> tuple[str, str] | None:
    """Parse piece type and standardized color ('white'/'black') from FEN board representation.

    Args:
        fen: Standard FEN string (at least board placement substring).
        square: 2-character square algebraic notation e.g. 'e2', 'c7'.

    Returns:
        Tuple of (piece_type, piece_color) e.g. ('pawn', 'black'), or None if square is empty or invalid.
    """
    if not fen or len(square) != 2:
        return None
    file_char = square[0].lower()
    rank_char = square[1]
    if not ("a" <= file_char <= "h") or not ("1" <= rank_char <= "8"):
        return None
    file_idx = ord(file_char) - ord("a")
    rank_idx = 8 - int(rank_char)

    board_str = fen.strip().split()[0]
    ranks = board_str.split("/")
    if len(ranks) != 8:
        return None

    expanded_rank: list[str] = []
    for ch in ranks[rank_idx]:
        if ch.isdigit():
            expanded_rank.extend(["."] * int(ch))
        else:
            expanded_rank.append(ch)

    if len(expanded_rank) != 8:
        return None

    piece_ch = expanded_rank[file_idx]
    if piece_ch == ".":
        return None

    piece_color = "white" if piece_ch.isupper() else "black"
    piece_map = {
        "p": "pawn",
        "n": "knight",
        "b": "bishop",
        "r": "rook",
        "q": "queen",
        "k": "king",
    }
    piece_type = piece_map.get(piece_ch.lower(), "pawn")
    return piece_type, piece_color
