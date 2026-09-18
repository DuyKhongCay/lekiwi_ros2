# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""
Single Responsibility Module: Chessboard Algebraic to Metric Coordinate Mapper.

Converts standard FIDE chess notation ("a1".."h8") into 3D metric coordinates (x, y, z)
in the chessboard_frame, and parses UCI move strings (e.g., "e2e4", "e7e8q").
Completely independent of ROS runtime graph for deterministic unit testing.
"""

from __future__ import annotations

from dataclasses import dataclass
import re
from typing import Optional, Tuple

from geometry_msgs.msg import Point

DEFAULT_BOARD_WIDTH_M = 0.20
DEFAULT_BOARD_HEIGHT_M = 0.20
DEFAULT_PIECE_GRASP_Z_M = (
    0.025  # ~2.5cm above board surface for chess piece grasp center
)


@dataclass(frozen=True)
class SquareCoordinate:
    """Represents a resolved 3D metric position for a chess square."""

    square_name: str
    file_char: str
    rank_char: str
    file_idx: int  # 0 for 'a' .. 7 for 'h'
    rank_idx: int  # 0 for '1' .. 7 for '8'
    x: float
    y: float
    z: float

    def to_point_msg(self) -> Point:
        """Convert to geometry_msgs/Point."""
        pt = Point()
        pt.x = float(self.x)
        pt.y = float(self.y)
        pt.z = float(self.z)
        return pt


@dataclass(frozen=True)
class UciMoveDetails:
    """Structured decomposition of a UCI move command."""

    uci: str
    from_square: str
    to_square: str
    promotion: Optional[str]
    pick_point: Point
    place_point: Point
    is_capture: bool


class ChessboardCoordinateMapper:
    """
    Translates FIDE algebraic square names to metric coordinates.

    Assumes chessboard_frame has:
    - X axis running along files a -> h (East)
    - Y axis running along ranks 1 -> 8 (North)
    - Z axis pointing upwards perpendicular to board surface
    - Origin at board center by default (or corner A1 if configured)
    """

    def __init__(
        self,
        board_width: float = DEFAULT_BOARD_WIDTH_M,
        board_height: float = DEFAULT_BOARD_HEIGHT_M,
        grasp_z: float = DEFAULT_PIECE_GRASP_Z_M,
        origin_at_center: bool = True,
    ) -> None:
        if board_width <= 0.0 or board_height <= 0.0:
            raise ValueError(
                f"Board dimensions must be positive (got {board_width}x{board_height})"
            )

        self._board_width = float(board_width)
        self._board_height = float(board_height)
        self._grasp_z = float(grasp_z)
        self._origin_at_center = bool(origin_at_center)

        self._square_size_x = self._board_width / 8.0
        self._square_size_y = self._board_height / 8.0

    @property
    def board_width(self) -> float:
        return self._board_width

    @property
    def board_height(self) -> float:
        return self._board_height

    @property
    def square_size_x(self) -> float:
        return self._square_size_x

    @property
    def square_size_y(self) -> float:
        return self._square_size_y

    def square_to_metric(
        self, square: str, z_offset: Optional[float] = None
    ) -> SquareCoordinate:
        """
        Convert algebraic square (e.g., 'e4', 'a1', 'h8') to metric coordinates.

        Parameters
        ----------
        square : str
            Two-character algebraic notation (e.g. 'e2').
        z_offset : Optional[float]
            Optional override for Z elevation. Defaults to self._grasp_z.

        Returns
        -------
        SquareCoordinate
            Named tuple containing square details and (x, y, z) in meters.
        """
        cleaned = square.strip().lower()
        if len(cleaned) != 2:
            raise ValueError(
                f"Invalid chess square notation: '{square}'. Must be 2 characters."
            )

        file_char = cleaned[0]
        rank_char = cleaned[1]

        if file_char not in "abcdefgh" or rank_char not in "12345678":
            raise ValueError(
                f"Square '{square}' is outside standard FIDE board limits [a-h][1-8]."
            )

        file_idx = ord(file_char) - ord("a")  # 0 .. 7
        rank_idx = ord(rank_char) - ord("1")  # 0 .. 7

        if self._origin_at_center:
            # Center of square relative to board center
            x = (file_idx - 3.5) * self._square_size_x
            y = (rank_idx - 3.5) * self._square_size_y
        else:
            # Relative to A1 bottom-left corner
            x = (file_idx + 0.5) * self._square_size_x
            y = (rank_idx + 0.5) * self._square_size_y

        z = self._grasp_z if z_offset is None else float(z_offset)

        return SquareCoordinate(
            square_name=cleaned,
            file_char=file_char,
            rank_char=rank_char,
            file_idx=file_idx,
            rank_idx=rank_idx,
            x=x,
            y=y,
            z=z,
        )

    def parse_uci_move(self, uci_move: str, is_capture: bool = False) -> UciMoveDetails:
        """
        Parse standard UCI move string into structured details with 3D pick & place points.

        Parameters
        ----------
        uci_move : str
            UCI string (e.g., 'e2e4', 'e7e8q', 'g1f3').
        is_capture : bool
            Whether this move is marked as a capture of an opponent piece.

        Returns
        -------
        UciMoveDetails
        """
        cleaned = uci_move.strip().lower()
        match = re.match(r"^([a-h][1-8])([a-h][1-8])([qrbn])?$", cleaned)
        if not match:
            raise ValueError(
                f"Invalid UCI move string: '{uci_move}'. Expected format like 'e2e4' or 'e7e8q'."
            )

        from_sq = match.group(1)
        to_sq = match.group(2)
        promo = match.group(3)

        pick_coord = self.square_to_metric(from_sq)
        place_coord = self.square_to_metric(to_sq)

        return UciMoveDetails(
            uci=cleaned,
            from_square=from_sq,
            to_square=to_sq,
            promotion=promo,
            pick_point=pick_coord.to_point_msg(),
            place_point=place_coord.to_point_msg(),
            is_capture=is_capture,
        )
