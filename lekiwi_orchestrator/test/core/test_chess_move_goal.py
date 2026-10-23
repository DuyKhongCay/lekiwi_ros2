# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for ChessMoveGoal domain entity, parsing, en-passant precision, and castling."""

import pytest
from lekiwi_interfaces.msg import ChessMoveDetails
from lekiwi_orchestrator.core.mission_types import ChessMoveGoal


def test_goal_from_clean_uci_standard():
    goal = ChessMoveGoal.from_uci_or_details("e2e4")
    assert goal is not None
    assert goal.uci == "e2e4"
    assert goal.from_square == "e2"
    assert goal.to_square == "e4"
    assert goal.piece == "pawn"
    assert goal.color == "white"
    assert goal.is_capture is False
    assert goal.captured_square == ""
    assert goal.promotion is None
    assert goal.is_castling is False


def test_goal_from_clean_uci_promotion():
    goal = ChessMoveGoal.from_uci_or_details("e7e8q")
    assert goal is not None
    assert goal.uci == "e7e8q"
    assert goal.from_square == "e7"
    assert goal.to_square == "e8"
    assert goal.promotion == "q"


def test_goal_from_move_details_normal_capture():
    details = ChessMoveDetails(
        uci="e4d5",
        piece_type="pawn:knight",
        piece_color="w",
        is_capture=True,
        is_en_passant=False,
        is_castling=False,
    )
    goal = ChessMoveGoal.from_uci_or_details("e4d5", move_details=details)
    assert goal is not None
    assert goal.uci == "e4d5"
    assert goal.from_square == "e4"
    assert goal.to_square == "d5"
    assert goal.piece == "pawn"
    assert goal.captured_piece == "knight"
    assert goal.is_capture is True
    assert goal.captured_square == "d5"
    assert goal.opposing_color == "black"


def test_goal_from_move_details_en_passant_white_captures_black():
    """White pawn on e5 captures Black pawn on f5 en-passant via e5f6. Target clear is f5!"""
    details = ChessMoveDetails(
        uci="e5f6",
        piece_type="pawn:pawn",
        piece_color="w",
        is_capture=True,
        is_en_passant=True,
        is_castling=False,
    )
    goal = ChessMoveGoal.from_uci_or_details("e5f6", move_details=details)
    assert goal is not None
    assert goal.uci == "e5f6"
    assert goal.from_square == "e5"
    assert goal.to_square == "f6"
    assert goal.piece == "pawn"
    assert goal.captured_piece == "pawn"
    assert goal.is_capture is True
    # En-passant clear square MUST be f5 (to_file 'f', from_rank '5')
    assert goal.captured_square == "f5"
    assert goal.opposing_color == "black"


def test_goal_from_move_details_en_passant_black_captures_white():
    """Black pawn on d4 captures White pawn on c4 en-passant via d4c3. Target clear is c4!"""
    details = ChessMoveDetails(
        uci="d4c3",
        piece_type="pawn:pawn",
        piece_color="b",
        is_capture=True,
        is_en_passant=True,
        is_castling=False,
    )
    goal = ChessMoveGoal.from_uci_or_details("d4c3", move_details=details)
    assert goal is not None
    assert goal.uci == "d4c3"
    assert goal.from_square == "d4"
    assert goal.to_square == "c3"
    assert goal.piece == "pawn"
    assert goal.color == "black"
    assert goal.is_capture is True
    # En-passant clear square MUST be c4 (to_file 'c', from_rank '4')
    assert goal.captured_square == "c4"
    assert goal.opposing_color == "white"


def test_goal_from_move_details_castling_white_kingside():
    """White castles kingside: e1g1 -> Rook moves h1 to f1."""
    details = ChessMoveDetails(
        uci="e1g1",
        piece_type="king",
        piece_color="w",
        is_capture=False,
        is_en_passant=False,
        is_castling=True,
    )
    goal = ChessMoveGoal.from_uci_or_details("e1g1", move_details=details)
    assert goal is not None
    assert goal.is_castling is True
    assert goal.castling_rook_from == "h1"
    assert goal.castling_rook_to == "f1"


def test_goal_from_move_details_castling_black_queenside():
    """Black castles queenside: e8c8 -> Rook moves a8 to d8."""
    details = ChessMoveDetails(
        uci="e8c8",
        piece_type="king",
        piece_color="b",
        is_capture=False,
        is_en_passant=False,
        is_castling=True,
    )
    goal = ChessMoveGoal.from_uci_or_details("e8c8", move_details=details)
    assert goal is not None
    assert goal.is_castling is True
    assert goal.color == "black"
    assert goal.castling_rook_from == "a8"
    assert goal.castling_rook_to == "d8"


def test_goal_malformed_short_uci():
    assert ChessMoveGoal.from_uci_or_details("e2") is None
    assert ChessMoveGoal.from_uci_or_details("") is None


def test_goal_direct_instantiation_minimal():
    """Verify minimal instantiation with only UCI creates functional entity with properties."""
    goal = ChessMoveGoal(uci="e2e4")
    assert goal.uci == "e2e4"
    assert goal.from_square == "e2"
    assert goal.to_square == "e4"
    assert goal.piece == "pawn"
    assert goal.captured_piece == ""
    assert goal.captured_square == ""
    assert goal.is_capture is False
    assert goal.is_en_passant is False
    assert goal.is_castling is False


def test_goal_direct_instantiation_capture_and_en_passant():
    """Verify direct instantiation correctly resolves en-passant clear square dynamically."""
    goal = ChessMoveGoal(
        uci="e5f6",
        piece_type="pawn:pawn",
        color="white",
        is_capture=True,
        is_en_passant=True,
    )
    assert goal.from_square == "e5"
    assert goal.to_square == "f6"
    assert goal.piece == "pawn"
    assert goal.captured_piece == "pawn"
    assert goal.captured_square == "f5"


def test_goal_direct_instantiation_castling():
    """Verify direct instantiation correctly resolves castling rooks from UCI."""
    goal = ChessMoveGoal(
        uci="e1g1",
        piece_type="king",
        color="white",
        is_castling=True,
    )
    assert goal.is_castling is True
    assert goal.castling_rook_from == "h1"
    assert goal.castling_rook_to == "f1"

