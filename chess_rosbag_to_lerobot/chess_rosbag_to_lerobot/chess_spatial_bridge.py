# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Chess spatial bridge: maps chess algebraic notation to 3D metric coordinates.

Adheres to Single Source of Truth (SSOT) matching:
- lekiwi_chess_master::domain
- lekiwi_motion::ChessboardMapper
- chessboard_pose_estimator
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import re
from typing import Callable, Optional, Sequence, Tuple
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


@dataclass(frozen=True)
class ParsedChessTask:
    """Structured result of grammar parsing an input chess task instruction."""
    task_type: TaskType
    from_square: Optional[str] = None
    to_square: Optional[str] = None
    clear_bin_side: Optional[ClearBinSide] = None


# -----------------------------------------------------------------------------
# Regex Matchers for Chess Grammar
# -----------------------------------------------------------------------------

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


def _parse_clear_task(cleaned: str) -> Optional[ParsedChessTask]:
    """Parse CLEAR instruction, enforcing explicit left/right bin side."""
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

    # Detect ambiguous CLEAR lacking side specification
    if _CLEAR_AMBIGUOUS_RE.search(cleaned) or (
        re.search(r"\bclear\b", cleaned, re.IGNORECASE)
        and re.search(r"\b[a-h][1-8]\b", cleaned, re.IGNORECASE)
    ):
        raise ValueError(
            f"Ambiguous CLEAR task missing left/right bin specification: '{cleaned}'"
        )

    return None


def _parse_atomic_task(cleaned: str) -> Optional[ParsedChessTask]:
    """Parse atomic PICK or PLACE instruction."""
    m_pick = _PICK_ATOMIC_RE.match(cleaned)
    if m_pick:
        return ParsedChessTask(TaskType.PICK, from_square=m_pick.group(1).lower())

    m_place = _PLACE_ATOMIC_RE.match(cleaned)
    if m_place:
        return ParsedChessTask(TaskType.PLACE, to_square=m_place.group(1).lower())

    return None


def _parse_move_task(cleaned: str) -> Optional[ParsedChessTask]:
    """Parse full MOVE instruction (pick/place pair, UCI, or descriptive)."""
    m_move = _MOVE_PICK_PLACE_RE.match(cleaned)
    if m_move:
        return ParsedChessTask(
            TaskType.MOVE,
            from_square=m_move.group(1).lower(),
            to_square=m_move.group(2).lower(),
        )

    # Check for direct UCI match
    uci_match = UCI_REGEX.search(cleaned)
    if uci_match:
        return ParsedChessTask(
            TaskType.MOVE,
            from_square=uci_match.group(1).lower(),
            to_square=uci_match.group(2).lower(),
        )

    # Check for pair of squares anywhere in descriptive text
    pair_match = SQUARE_PAIR_REGEX.search(cleaned)
    if pair_match:
        return ParsedChessTask(
            TaskType.MOVE,
            from_square=pair_match.group(1).lower(),
            to_square=pair_match.group(2).lower(),
        )

    return None


def parse_chess_task(task_text: str) -> ParsedChessTask:
    """Parse a chess task instruction into a structured ParsedChessTask.

    Supports:
        1. CLEAR with explicit side: 'Clear d4 to left onboard bin', 'clear_right e5'
        2. CLEAR ambiguous: 'Clear d4 to onboard bin' -> raises ValueError
        3. Atomic PICK: 'Pick d5'
        4. Atomic PLACE: 'Place e4'
        5. Full MOVE: 'Pick c7, place c5', 'e2e4', 'Move from e2 to e4'

    Raises:
        ValueError: if instruction is empty, ambiguous, or unparseable.
    """
    cleaned = (task_text or "").strip()
    if not cleaned:
        raise ValueError("Chess task instruction is empty")

    res = _parse_clear_task(cleaned)
    if res is not None:
        return res

    res = _parse_atomic_task(cleaned)
    if res is not None:
        return res

    res = _parse_move_task(cleaned)
    if res is not None:
        return res

    raise ValueError(
        f"Cannot parse chess task instruction into known primitives: '{task_text}'"
    )


def resolve_episode_spatial_plan(
    task_text: str,
    cfg: ChessboardConfig,
) -> EpisodeSpatialPlan:
    """Resolve an episode's spatial conditioning plan using direct Cartesian landmarks."""
    parsed = parse_chess_task(task_text)

    if parsed.task_type == TaskType.MOVE:
        from_xyz = np.array(square_to_metric(parsed.from_square, cfg), dtype=np.float32)
        to_xyz = np.array(square_to_metric(parsed.to_square, cfg), dtype=np.float32)
        source = SpatialEndpoint(xyz=from_xyz, is_board_frame=True)
        target = SpatialEndpoint(xyz=to_xyz, is_board_frame=True)
        one_hot = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        canonical = f"Pick {parsed.from_square}, place {parsed.to_square}"

    elif parsed.task_type == TaskType.CLEAR:
        from_xyz = np.array(square_to_metric(parsed.from_square, cfg), dtype=np.float32)
        source = SpatialEndpoint(xyz=from_xyz, is_board_frame=True)
        bin_coords = (
            cfg.landmarks.clear_left
            if parsed.clear_bin_side == ClearBinSide.LEFT
            else cfg.landmarks.clear_right
        )
        target_xyz = np.array(bin_coords, dtype=np.float32)
        target = SpatialEndpoint(xyz=target_xyz, is_board_frame=False)
        one_hot = np.array([0.0, 1.0, 0.0, 0.0], dtype=np.float32)
        canonical = f"Clear {parsed.from_square} to {parsed.clear_bin_side.value} onboard bin"

    elif parsed.task_type == TaskType.PICK:
        from_xyz = np.array(square_to_metric(parsed.from_square, cfg), dtype=np.float32)
        source = SpatialEndpoint(xyz=from_xyz, is_board_frame=True)
        stow_xyz = np.array(cfg.landmarks.stow, dtype=np.float32)
        target = SpatialEndpoint(xyz=stow_xyz, is_board_frame=False)
        one_hot = np.array([0.0, 0.0, 1.0, 0.0], dtype=np.float32)
        canonical = f"Pick {parsed.from_square}"

    elif parsed.task_type == TaskType.PLACE:
        stow_xyz = np.array(cfg.landmarks.stow, dtype=np.float32)
        source = SpatialEndpoint(xyz=stow_xyz, is_board_frame=False)
        to_xyz = np.array(square_to_metric(parsed.to_square, cfg), dtype=np.float32)
        target = SpatialEndpoint(xyz=to_xyz, is_board_frame=True)
        one_hot = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        canonical = f"Place {parsed.to_square}"

    else:
        raise ValueError(f"Unsupported task type: {parsed.task_type}")

    return EpisodeSpatialPlan(
        task_type=parsed.task_type,
        canonical_task=canonical,
        source=source,
        target=target,
        task_one_hot=one_hot,
    )


def assemble_environment_state(
    plan: EpisodeSpatialPlan,
    transform_fn: Callable[[np.ndarray], Optional[np.ndarray]],
) -> np.ndarray:
    """Assemble 10D environment state vector for current reference tick.

    Vector format:
    [source_x, source_y, source_z, target_x, target_y, target_z,
     is_move, is_clear, is_pick, is_place]
    """
    if plan.source.is_board_frame:
        transformed_src = transform_fn(plan.source.xyz)
        src_xyz = transformed_src if transformed_src is not None else plan.source.xyz
    else:
        src_xyz = plan.source.xyz

    if plan.target.is_board_frame:
        transformed_tgt = transform_fn(plan.target.xyz)
        tgt_xyz = transformed_tgt if transformed_tgt is not None else plan.target.xyz
    else:
        tgt_xyz = plan.target.xyz

    return np.concatenate([src_xyz, tgt_xyz, plan.task_one_hot]).astype(np.float32)
