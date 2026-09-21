# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for ChessboardCoordinateMapper (SRP & Math verification)."""

import math
import pytest

from lekiwi_orchestrator.chessboard_coordinate_mapper import (
    ChessboardCoordinateMapper,
    DEFAULT_BOARD_HEIGHT_M,
    DEFAULT_BOARD_WIDTH_M,
    DEFAULT_PIECE_GRASP_Z_M,
)


def test_mapper_initialization_defaults():
    mapper = ChessboardCoordinateMapper()
    assert math.isclose(mapper.board_width, DEFAULT_BOARD_WIDTH_M)
    assert math.isclose(mapper.board_height, DEFAULT_BOARD_HEIGHT_M)
    assert math.isclose(mapper.square_size_x, DEFAULT_BOARD_WIDTH_M / 8.0)
    assert math.isclose(mapper.square_size_y, DEFAULT_BOARD_HEIGHT_M / 8.0)


def test_mapper_invalid_dimensions():
    with pytest.raises(ValueError):
        ChessboardCoordinateMapper(board_width=-0.1, board_height=0.2)
    with pytest.raises(ValueError):
        ChessboardCoordinateMapper(board_width=0.2, board_height=0.0)


def test_square_symmetry_around_center():
    # 0.20m width => each square is 0.025m (25mm)
    mapper = ChessboardCoordinateMapper(
        board_width=0.20, board_height=0.20, origin_at_center=True
    )
    sq_size = 0.025

    # A1 (col 0, row 0): center is at (-3.5 * 0.025, -3.5 * 0.025) = (-0.0875, -0.0875)
    a1 = mapper.square_to_metric("a1")
    assert math.isclose(a1.x, -3.5 * sq_size, abs_tol=1e-6)
    assert math.isclose(a1.y, -3.5 * sq_size, abs_tol=1e-6)
    assert math.isclose(a1.z, DEFAULT_PIECE_GRASP_Z_M, abs_tol=1e-6)

    # H8 (col 7, row 7): center is at (+3.5 * 0.025, +3.5 * 0.025) = (+0.0875, +0.0875)
    h8 = mapper.square_to_metric("h8")
    assert math.isclose(h8.x, 3.5 * sq_size, abs_tol=1e-6)
    assert math.isclose(h8.y, 3.5 * sq_size, abs_tol=1e-6)

    # D4 (col 3, row 3): center is (-0.5 * 0.025, -0.5 * 0.025)
    d4 = mapper.square_to_metric("d4")
    assert math.isclose(d4.x, -0.5 * sq_size, abs_tol=1e-6)
    assert math.isclose(d4.y, -0.5 * sq_size, abs_tol=1e-6)

    # E5 (col 4, row 4): center is (+0.5 * 0.025, +0.5 * 0.025)
    e5 = mapper.square_to_metric("e5")
    assert math.isclose(e5.x, 0.5 * sq_size, abs_tol=1e-6)
    assert math.isclose(e5.y, 0.5 * sq_size, abs_tol=1e-6)

    # Point message conversion
    pt = e5.to_point_msg()
    assert math.isclose(pt.x, e5.x)
    assert math.isclose(pt.y, e5.y)
    assert math.isclose(pt.z, e5.z)


def test_square_invalid_notations():
    mapper = ChessboardCoordinateMapper()
    with pytest.raises(ValueError, match="Must be 2 characters"):
        mapper.square_to_metric("e")
    with pytest.raises(ValueError, match="outside standard FIDE"):
        mapper.square_to_metric("i4")
    with pytest.raises(ValueError, match="outside standard FIDE"):
        mapper.square_to_metric("e9")
    with pytest.raises(ValueError, match="outside standard FIDE"):
        mapper.square_to_metric("a0")


def test_parse_uci_move_standard():
    mapper = ChessboardCoordinateMapper(board_width=0.20, board_height=0.20)
    move = mapper.parse_uci_move("e2e4")
    assert move.uci == "e2e4"
    assert move.from_square == "e2"
    assert move.to_square == "e4"
    assert move.promotion is None
    assert not move.is_capture

    # Check that points correspond to e2 and e4
    e2 = mapper.square_to_metric("e2")
    e4 = mapper.square_to_metric("e4")
    assert math.isclose(move.pick_point.x, e2.x)
    assert math.isclose(move.pick_point.y, e2.y)
    assert math.isclose(move.place_point.x, e4.x)
    assert math.isclose(move.place_point.y, e4.y)


def test_parse_uci_move_promotion_and_capture():
    mapper = ChessboardCoordinateMapper()
    move = mapper.parse_uci_move("e7e8q", is_capture=True)
    assert move.uci == "e7e8q"
    assert move.from_square == "e7"
    assert move.to_square == "e8"
    assert move.promotion == "q"
    assert move.is_capture is True


def test_parse_uci_move_malformed():
    mapper = ChessboardCoordinateMapper()
    with pytest.raises(ValueError, match="Invalid UCI move string"):
        mapper.parse_uci_move("e2")
    with pytest.raises(ValueError, match="Invalid UCI move string"):
        mapper.parse_uci_move("e2-e4")
    with pytest.raises(ValueError, match="Invalid UCI move string"):
        mapper.parse_uci_move("invalid")
