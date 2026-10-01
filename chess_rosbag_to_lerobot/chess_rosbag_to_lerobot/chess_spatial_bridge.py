# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Chess spatial bridge: maps chess algebraic notation to 3D metric coordinates.

Adheres to Single Source of Truth (SSOT) matching:
- lekiwi_chess_master::domain
- lekiwi_motion::ChessboardMapper
- chessboard_pose_estimator
"""

from __future__ import annotations

import re
from typing import Optional, Tuple
import numpy as np

from chess_rosbag_to_lerobot.config import ChessboardConfig

# Regex to find two consecutive algebraic chess squares (e.g., 'e2e4' or 'from e2 to e4')
UCI_REGEX = re.compile(r"\b([a-h][1-8])([a-h][1-8])([qrbn])?\b", re.IGNORECASE)
SQUARE_PAIR_REGEX = re.compile(r"([a-h][1-8]).*?([a-h][1-8])", re.IGNORECASE)


def square_to_metric(square: str, cfg: ChessboardConfig) -> Tuple[float, float, float]:
    """Convert an algebraic square (e.g., 'e4', 'a1', 'h8') to (x, y, z) in chessboard_frame.

    Args:
        square: 2-character FIDE algebraic square name (e.g. 'e4').
        cfg: Chessboard configuration with square size and origin placement.

    Returns:
        (x, y, z) coordinates in meters relative to chessboard_frame.
    """
    cleaned = square.strip().lower()
    if len(cleaned) < 2 or not ("a" <= cleaned[0] <= "h" and "1" <= cleaned[1] <= "8"):
        raise ValueError(f"Invalid chess square notation: '{square}'")

    file_idx = ord(cleaned[0]) - ord("a")  # 0 to 7 (a to h)
    rank_idx = ord(cleaned[1]) - ord("1")  # 0 to 7 (1 to 8)

    if cfg.origin_at_center:
        # Center of board is between rank 4 & 5, and file d & e
        x = (file_idx - 3.5) * cfg.square_size_m
        y = (rank_idx - 3.5) * cfg.square_size_m
    else:
        # Origin at corner A1
        x = (file_idx + 0.5) * cfg.square_size_m
        y = (rank_idx + 0.5) * cfg.square_size_m

    z = cfg.grasp_clearance_z_m
    return x, y, z


def parse_uci_move(uci_str: str) -> Tuple[str, str]:
    """Parse a UCI move string (e.g., 'e2e4' or 'e7e8q') into (from_sq, to_sq)."""
    cleaned = uci_str.strip().lower()
    if len(cleaned) < 4:
        raise ValueError(f"UCI move string too short: '{uci_str}'")

    from_sq = cleaned[:2]
    to_sq = cleaned[2:4]
    if not ("a" <= from_sq[0] <= "h" and "1" <= from_sq[1] <= "8"):
        raise ValueError(f"Invalid from_square in UCI move: '{from_sq}'")
    if not ("a" <= to_sq[0] <= "h" and "1" <= to_sq[1] <= "8"):
        raise ValueError(f"Invalid to_square in UCI move: '{to_sq}'")

    return from_sq, to_sq


def extract_move_squares(task_text: str) -> Tuple[Optional[str], Optional[str]]:
    """Extract (from_sq, to_sq) from arbitrary task descriptions or UCI strings.

    Examples:
        - "e2e4" -> ("e2", "e4")
        - "Move white pawn from e2 to e4" -> ("e2", "e4")
        - "Pick e7, place e5" -> ("e7", "e5")
    """
    if not task_text:
        return None, None

    # Check for direct UCI match first
    uci_match = UCI_REGEX.search(task_text)
    if uci_match:
        return uci_match.group(1).lower(), uci_match.group(2).lower()

    # Check for pair of squares anywhere in text
    pair_match = SQUARE_PAIR_REGEX.search(task_text)
    if pair_match:
        return pair_match.group(1).lower(), pair_match.group(2).lower()

    return None, None


def resolve_pick_place_poses(
    task_text: str,
    cfg: ChessboardConfig,
    default_square_from: str = "e2",
    default_square_to: str = "e4",
) -> Tuple[np.ndarray, np.ndarray]:
    """Resolve 3D Cartesian coordinates [x, y, z] for pick and place targets.

    Returns:
        (pick_xyz, place_xyz) as float32 NumPy arrays of shape (3,).
    """
    from_sq, to_sq = extract_move_squares(task_text)
    if not from_sq or not to_sq:
        from_sq = default_square_from
        to_sq = default_square_to

    pick_x, pick_y, pick_z = square_to_metric(from_sq, cfg)
    place_x, place_y, place_z = square_to_metric(to_sq, cfg)

    pick_vec = np.array([pick_x, pick_y, pick_z], dtype=np.float32)
    place_vec = np.array([place_x, place_y, place_z], dtype=np.float32)

    return pick_vec, place_vec
