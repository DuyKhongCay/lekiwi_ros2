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
from typing import TYPE_CHECKING, Any

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


@dataclass(frozen=True)
class ChessMoveGoal:
    """Lightweight mission-level move description.

    Holds parsed UCI coordinates, capture targets, and castling rook waypoints.
    """

    uci: str
    from_square: str
    to_square: str
    promotion: str | None = None
    is_capture: bool = False
    captured_square: str = ""
    castling_rook_from: str = ""
    castling_rook_to: str = ""

    @classmethod
    def from_uci_or_details(
        cls,
        uci_move: str,
        move_details: ChessMoveDetails | None = None,
    ) -> ChessMoveGoal | None:
        """Parse ChessMoveGoal from UCI string or rich ChessMoveDetails message.

        Extracts source, target, promotion, and castling parameters into a uniform goal.

        Args:
            uci_move: Standard UCI move string (e.g., 'e2e4', 'e7e8q').
            move_details: Optional ROS 2 message providing explicit capture/castling info.

        Returns:
            Populated ChessMoveGoal instance, or None if UCI string length is invalid (< 4).
        """
        cleaned_move = uci_move.strip().lower()
        if len(cleaned_move) < 4:
            return None

        if move_details is not None and getattr(move_details, "from_square", ""):
            is_cap = bool(getattr(move_details, "is_capture", False))
            cap_sq = getattr(move_details, "captured_square", "") or (
                move_details.to_square if is_cap else ""
            )
            promo = getattr(move_details, "promotion_piece", "") or None

            return cls(
                uci=getattr(move_details, "uci", "") or cleaned_move,
                from_square=move_details.from_square,
                to_square=move_details.to_square,
                promotion=promo,
                is_capture=is_cap,
                captured_square=cap_sq,
                castling_rook_from=getattr(move_details, "castling_rook_from", ""),
                castling_rook_to=getattr(move_details, "castling_rook_to", ""),
            )

        from_sq = cleaned_move[:2]
        to_sq = cleaned_move[2:4]
        promo = cleaned_move[4:] if len(cleaned_move) > 4 else None
        return cls(
            uci=cleaned_move,
            from_square=from_sq,
            to_square=to_sq,
            promotion=promo,
            is_capture=False,
        )
