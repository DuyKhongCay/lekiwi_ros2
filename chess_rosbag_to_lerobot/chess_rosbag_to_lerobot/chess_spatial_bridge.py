# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Chess spatial bridge: maps chess instructions to 3D metric coordinates and task metadata.

Adheres to Single Source of Truth (SSOT) matching:
- lekiwi_chess_master::domain
- lekiwi_orchestrator::MovePlanBuilder
- chessboard_pose_estimator
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from pathlib import Path
import re
from typing import Callable, List, Optional, Sequence, Tuple
import numpy as np

from chess_rosbag_to_lerobot.config import ChessboardConfig


# -----------------------------------------------------------------------------
# Domain Enums & Dataclasses
# -----------------------------------------------------------------------------

class TaskType(str, Enum):
    MOVE = "move"
    CLEAR = "clear"
    PICK = "pick"
    PLACE = "place"


class ClearBinSide(str, Enum):
    LEFT = "left"
    RIGHT = "right"


TASK_ORDER = {
    TaskType.MOVE: 0,
    TaskType.CLEAR: 1,
    TaskType.PICK: 2,
    TaskType.PLACE: 3,
}

PIECE_ORDER = {
    "pawn": 0,
    "knight": 1,
    "bishop": 2,
    "rook": 3,
    "queen": 4,
    "king": 5,
    "piece": 6,
    "unknown": 7,
}

COLOR_ORDER = {
    "white": 0,
    "black": 1,
    "unknown": 2,
}


@dataclass(frozen=True)
class SpatialEndpoint:
    """3D point and its reference frame domain flag."""
    xyz: np.ndarray  # shape (3,), dtype float32
    is_board_frame: bool


@dataclass(frozen=True)
class EpisodeSpatialPlan:
    """Pre-computed spatial plan for an episode to prevent hot-loop branching."""
    task_type: TaskType
    canonical_task: str
    source: SpatialEndpoint
    target: SpatialEndpoint
    task_one_hot: np.ndarray  # shape (4,), [is_move, is_clear, is_pick, is_place]
    stow_xyz: np.ndarray      # shape (3,), dtype float32 in base_footprint frame


@dataclass(frozen=True)
class ParsedChessTask:
    """Structured result of grammar parsing an input chess task instruction."""
    task_type: TaskType
    piece_color: Optional[str] = None
    piece_type: Optional[str] = None
    from_square: Optional[str] = None
    to_square: Optional[str] = None
    clear_bin_side: Optional[ClearBinSide] = None


# -----------------------------------------------------------------------------
# Regex Matchers for Chess Grammar (SmolVLA SSOT & Legacy Fallbacks)
# -----------------------------------------------------------------------------

# SmolVLA primary patterns (orchestrator SSOT):
# - Move <color> <piece> to target [from_sq to to_sq]  (square brackets optional)
# - Clear <color> <piece> to <left|right> bin [from_sq]  (square brackets optional)
# - Pick <color> <piece> [from_sq]  (square brackets optional)
# - Place <color> <piece> [to_sq]   (square brackets optional)

_SMOLVLA_MOVE_RE = re.compile(
    r"^\s*move\s+(white|black)\s+(pawn|knight|bishop|rook|queen|king|piece)"
    r"(?:\s+to\s+target)?"
    r"(?:\s*\[\s*([a-h][1-8])\s+to\s+([a-h][1-8])\s*\])?\s*$",
    re.IGNORECASE,
)

_SMOLVLA_CLEAR_RE = re.compile(
    r"^\s*clear\s+(white|black)\s+(pawn|knight|bishop|rook|queen|king|piece)"
    r"\s+to\s+(left|right)\s+bin"
    r"(?:\s*\[\s*([a-h][1-8])\s*\])?\s*$",
    re.IGNORECASE,
)

_SMOLVLA_PICK_RE = re.compile(
    r"^\s*pick\s+(white|black)\s+(pawn|knight|bishop|rook|queen|king|piece)"
    r"(?:\s*\[\s*([a-h][1-8])\s*\])?\s*$",
    re.IGNORECASE,
)

_SMOLVLA_PLACE_RE = re.compile(
    r"^\s*place\s+(white|black)\s+(pawn|knight|bishop|rook|queen|king|piece)"
    r"(?:\s*\[\s*([a-h][1-8])\s*\])?\s*$",
    re.IGNORECASE,
)

# Legacy Teleop / UCI Fallback patterns
UCI_REGEX = re.compile(r"\b([a-h][1-8])([a-h][1-8])([qrbn])?\b", re.IGNORECASE)
SQUARE_PAIR_REGEX = re.compile(r"([a-h][1-8]).*?([a-h][1-8])", re.IGNORECASE)

_CLEAR_EXPLICIT_RE = re.compile(
    r"^\s*clear\s+([a-h][1-8])(?:\s+to)?\s+(left|right)(?:\s+onboard)?(?:\s+bin)?\b",
    re.IGNORECASE,
)
_CLEAR_PREFIX_SIDE_RE = re.compile(
    r"^\s*clear[_\s]+(left|right)\s+([a-h][1-8])\b",
    re.IGNORECASE,
)
_CLEAR_AMBIGUOUS_RE = re.compile(
    r"^\s*clear\s+([a-h][1-8])\b",
    re.IGNORECASE,
)
_PICK_ATOMIC_RE = re.compile(
    r"^\s*pick\s+([a-h][1-8])\s*$",
    re.IGNORECASE,
)
_PLACE_ATOMIC_RE = re.compile(
    r"^\s*place\s+([a-h][1-8])\s*$",
    re.IGNORECASE,
)
_MOVE_PICK_PLACE_RE = re.compile(
    r"^\s*pick\s+([a-h][1-8]),?\s+place\s+([a-h][1-8])\s*$",
    re.IGNORECASE,
)


def square_to_metric(square: str, cfg: ChessboardConfig) -> Tuple[float, float, float]:
    """Convert an algebraic square (e.g. 'e4', 'a1', 'h8') to (x, y, z) in chessboard_frame."""
    cleaned = square.strip().lower()
    if len(cleaned) < 2 or not ("a" <= cleaned[0] <= "h" and "1" <= cleaned[1] <= "8"):
        raise ValueError(f"Invalid chess square notation: '{square}'")

    file_idx = ord(cleaned[0]) - ord("a")  # 0 to 7 (a to h)
    rank_idx = ord(cleaned[1]) - ord("1")  # 0 to 7 (1 to 8)

    # Origin is strictly at board center (between rank 4 & 5, and file d & e)
    x = (file_idx - 3.5) * cfg.square_size_m
    y = (rank_idx - 3.5) * cfg.square_size_m
    z = cfg.grasp_clearance_z_m
    return x, y, z


def parse_chess_task(task_text: str) -> ParsedChessTask:
    """Parse a chess task instruction into a structured ParsedChessTask.

    Primary (SmolVLA Orchestrator SSOT):
        1. 'Move black pawn to target [c7 to c5]' or 'Move black pawn to target'
        2. 'Clear white knight to left bin [d4]' or 'Clear white knight to left bin'
        3. 'Pick black bishop [f8]' or 'Pick black bishop'
        4. 'Place black bishop [c5]' or 'Place black bishop'

    Fallback (Legacy Teleop):
        5. 'Pick c7, place c5', 'e2e4', 'Move from e2 to e4'
        6. 'Clear d4 to left onboard bin', 'clear_left e5'
        7. 'Pick d5', 'Place e4'

    Raises:
        ValueError: if instruction is empty, ambiguous, or unparseable.
    """
    cleaned = (task_text or "").strip()
    if not cleaned:
        raise ValueError("Chess task instruction is empty")

    # 1. Try SmolVLA primary matchers
    m_smol_move = _SMOLVLA_MOVE_RE.match(cleaned)
    if m_smol_move:
        return ParsedChessTask(
            TaskType.MOVE,
            piece_color=m_smol_move.group(1).lower(),
            piece_type=m_smol_move.group(2).lower(),
            from_square=m_smol_move.group(3).lower() if m_smol_move.group(3) else None,
            to_square=m_smol_move.group(4).lower() if m_smol_move.group(4) else None,
        )

    m_smol_clear = _SMOLVLA_CLEAR_RE.match(cleaned)
    if m_smol_clear:
        side = ClearBinSide.LEFT if m_smol_clear.group(3).lower() == "left" else ClearBinSide.RIGHT
        return ParsedChessTask(
            TaskType.CLEAR,
            piece_color=m_smol_clear.group(1).lower(),
            piece_type=m_smol_clear.group(2).lower(),
            clear_bin_side=side,
            from_square=m_smol_clear.group(4).lower() if m_smol_clear.group(4) else None,
        )

    m_smol_pick = _SMOLVLA_PICK_RE.match(cleaned)
    if m_smol_pick:
        return ParsedChessTask(
            TaskType.PICK,
            piece_color=m_smol_pick.group(1).lower(),
            piece_type=m_smol_pick.group(2).lower(),
            from_square=m_smol_pick.group(3).lower() if m_smol_pick.group(3) else None,
        )

    m_smol_place = _SMOLVLA_PLACE_RE.match(cleaned)
    if m_smol_place:
        return ParsedChessTask(
            TaskType.PLACE,
            piece_color=m_smol_place.group(1).lower(),
            piece_type=m_smol_place.group(2).lower(),
            to_square=m_smol_place.group(3).lower() if m_smol_place.group(3) else None,
        )

    # 2. Try legacy explicit CLEAR
    m_explicit = _CLEAR_EXPLICIT_RE.search(cleaned)
    if m_explicit:
        sq = m_explicit.group(1).lower()
        side_str = m_explicit.group(2).lower()
        side = ClearBinSide.LEFT if side_str == "left" else ClearBinSide.RIGHT
        return ParsedChessTask(TaskType.CLEAR, from_square=sq, clear_bin_side=side)

    m_prefix = _CLEAR_PREFIX_SIDE_RE.search(cleaned)
    if m_prefix:
        side_str = m_prefix.group(1).lower()
        sq = m_prefix.group(2).lower()
        side = ClearBinSide.LEFT if side_str == "left" else ClearBinSide.RIGHT
        return ParsedChessTask(TaskType.CLEAR, from_square=sq, clear_bin_side=side)

    if _CLEAR_AMBIGUOUS_RE.search(cleaned) or (
        re.search(r"\bclear\b", cleaned, re.IGNORECASE)
        and re.search(r"\b[a-h][1-8]\b", cleaned, re.IGNORECASE)
    ):
        raise ValueError(
            f"Ambiguous CLEAR task missing left/right bin specification: '{cleaned}'"
        )

    # 3. Try legacy atomic PICK / PLACE
    m_pick = _PICK_ATOMIC_RE.match(cleaned)
    if m_pick:
        return ParsedChessTask(TaskType.PICK, from_square=m_pick.group(1).lower())

    m_place = _PLACE_ATOMIC_RE.match(cleaned)
    if m_place:
        return ParsedChessTask(TaskType.PLACE, to_square=m_place.group(1).lower())

    # 4. Try legacy MOVE
    m_move = _MOVE_PICK_PLACE_RE.match(cleaned)
    if m_move:
        return ParsedChessTask(
            TaskType.MOVE,
            from_square=m_move.group(1).lower(),
            to_square=m_move.group(2).lower(),
        )

    uci_match = UCI_REGEX.search(cleaned)
    if uci_match:
        return ParsedChessTask(
            TaskType.MOVE,
            from_square=uci_match.group(1).lower(),
            to_square=uci_match.group(2).lower(),
        )

    pair_match = SQUARE_PAIR_REGEX.search(cleaned)
    if pair_match:
        return ParsedChessTask(
            TaskType.MOVE,
            from_square=pair_match.group(1).lower(),
            to_square=pair_match.group(2).lower(),
        )

    raise ValueError(
        f"Cannot parse chess task instruction into known primitives: '{task_text}'"
    )


def resolve_episode_spatial_plan(
    task_text: str,
    cfg: ChessboardConfig,
) -> EpisodeSpatialPlan:
    """Resolve an episode's spatial conditioning plan using direct Cartesian landmarks."""
    parsed = parse_chess_task(task_text)
    stow_xyz = np.array(cfg.landmarks.stow, dtype=np.float32)

    if parsed.task_type == TaskType.MOVE:
        if parsed.from_square:
            from_xyz = np.array(square_to_metric(parsed.from_square, cfg), dtype=np.float32)
            source = SpatialEndpoint(xyz=from_xyz, is_board_frame=True)
        else:
            source = SpatialEndpoint(xyz=stow_xyz, is_board_frame=False)

        if parsed.to_square:
            to_xyz = np.array(square_to_metric(parsed.to_square, cfg), dtype=np.float32)
            target = SpatialEndpoint(xyz=to_xyz, is_board_frame=True)
        else:
            target = SpatialEndpoint(xyz=stow_xyz, is_board_frame=False)

        one_hot = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)

    elif parsed.task_type == TaskType.CLEAR:
        if parsed.from_square:
            from_xyz = np.array(square_to_metric(parsed.from_square, cfg), dtype=np.float32)
            source = SpatialEndpoint(xyz=from_xyz, is_board_frame=True)
        else:
            source = SpatialEndpoint(xyz=stow_xyz, is_board_frame=False)

        bin_coords = (
            cfg.landmarks.clear_left
            if parsed.clear_bin_side == ClearBinSide.LEFT
            else cfg.landmarks.clear_right
        )
        target = SpatialEndpoint(xyz=np.array(bin_coords, dtype=np.float32), is_board_frame=False)
        one_hot = np.array([0.0, 1.0, 0.0, 0.0], dtype=np.float32)

    elif parsed.task_type == TaskType.PICK:
        if parsed.from_square:
            from_xyz = np.array(square_to_metric(parsed.from_square, cfg), dtype=np.float32)
            source = SpatialEndpoint(xyz=from_xyz, is_board_frame=True)
        else:
            source = SpatialEndpoint(xyz=stow_xyz, is_board_frame=False)

        target = SpatialEndpoint(xyz=stow_xyz, is_board_frame=False)
        one_hot = np.array([0.0, 0.0, 1.0, 0.0], dtype=np.float32)

    elif parsed.task_type == TaskType.PLACE:
        source = SpatialEndpoint(xyz=stow_xyz, is_board_frame=False)
        if parsed.to_square:
            to_xyz = np.array(square_to_metric(parsed.to_square, cfg), dtype=np.float32)
            target = SpatialEndpoint(xyz=to_xyz, is_board_frame=True)
        else:
            target = SpatialEndpoint(xyz=stow_xyz, is_board_frame=False)

        one_hot = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)

    else:
        raise ValueError(f"Unsupported task type: {parsed.task_type}")

    return EpisodeSpatialPlan(
        task_type=parsed.task_type,
        canonical_task=task_text,
        source=source,
        target=target,
        task_one_hot=one_hot,
        stow_xyz=stow_xyz,
    )


def sort_episodes(
    episodes: Sequence[Path],
    task_extractor: Optional[Callable[[Path], str]] = None,
) -> List[Path]:
    """Sort episode directories by Task (MOVE -> CLEAR -> PICK -> PLACE), Piece Type, Color, and ID.

    Guarantees deterministic grouping of episodes for LeRobot dataset creation.
    """
    def _episode_sort_key(ep_dir: Path) -> Tuple[int, int, int, str]:
        task_str = task_extractor(ep_dir) if task_extractor is not None else ""
        if not task_str:
            return (99, 99, 99, ep_dir.name)

        try:
            parsed = parse_chess_task(task_str)
            t_rank = TASK_ORDER.get(parsed.task_type, 99)
            p_rank = PIECE_ORDER.get((parsed.piece_type or "unknown").lower(), 99)
            c_rank = COLOR_ORDER.get((parsed.piece_color or "unknown").lower(), 99)
            return (t_rank, p_rank, c_rank, ep_dir.name)
        except Exception:
            return (99, 99, 99, ep_dir.name)

    return sorted(episodes, key=_episode_sort_key)

