# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for ChessMoveGoal domain entity, parsing, en-passant precision, and castling."""

import pytest
from lekiwi_interfaces.msg import ChessMoveDetails
from lekiwi_orchestrator.core.mission_types import ChessMoveGoal


@pytest.mark.parametrize(
    (
        "uci",
        "move_details",
        "expected_from",
        "expected_to",
        "expected_piece",
        "expected_color",
        "expected_capture",
        "expected_captured_sq",
        "expected_promo",
        "expected_castle",
        "expected_rook_from",
        "expected_rook_to",
    ),
    [
        (
            "e2e4",
            None,
            "e2",
            "e4",
            "pawn",
            "white",
            False,
            "",
            "",
            False,
            "",
            "",
        ),
        (
            "e7e8q",
            None,
            "e7",
            "e8",
            "pawn",
            "white",
            False,
            "",
            "q",
            False,
            "",
            "",
        ),
        (
            "e4d5",
            ChessMoveDetails(
                uci="e4d5",
                piece_type="pawn:knight",
                piece_color="white",
                is_capture=True,
                is_en_passant=False,
                is_castling=False,
            ),
            "e4",
            "d5",
            "pawn",
            "white",
            True,
            "d5",
            "",
            False,
            "",
            "",
        ),
        (
            "e5f6",
            ChessMoveDetails(
                uci="e5f6",
                piece_type="pawn:pawn",
                piece_color="white",
                is_capture=True,
                is_en_passant=True,
                is_castling=False,
            ),
            "e5",
            "f6",
            "pawn",
            "white",
            True,
            "f5",
            "",
            False,
            "",
            "",
        ),
        (
            "d4c3",
            ChessMoveDetails(
                uci="d4c3",
                piece_type="pawn:pawn",
                piece_color="black",
                is_capture=True,
                is_en_passant=True,
                is_castling=False,
            ),
            "d4",
            "c3",
            "pawn",
            "black",
            True,
            "c4",
            "",
            False,
            "",
            "",
        ),
        (
            "e1g1",
            ChessMoveDetails(
                uci="e1g1",
                piece_type="king",
                piece_color="white",
                is_capture=False,
                is_en_passant=False,
                is_castling=True,
            ),
            "e1",
            "g1",
            "king",
            "white",
            False,
            "",
            "",
            True,
            "h1",
            "f1",
        ),
        (
            "e8c8",
            ChessMoveDetails(
                uci="e8c8",
                piece_type="king",
                piece_color="black",
                is_capture=False,
                is_en_passant=False,
                is_castling=True,
            ),
            "e8",
            "c8",
            "king",
            "black",
            False,
            "",
            "",
            True,
            "a8",
            "d8",
        ),
    ],
)
def test_chess_move_goal_parsing_and_properties(
    uci,
    move_details,
    expected_from,
    expected_to,
    expected_piece,
    expected_color,
    expected_capture,
    expected_captured_sq,
    expected_promo,
    expected_castle,
    expected_rook_from,
    expected_rook_to,
):
    """Verify ChessMoveGoal parsing, semantics, properties, and direct instantiation."""
    goal = ChessMoveGoal.from_uci_or_details(uci, move_details=move_details)
    assert goal is not None
    assert goal.uci == uci
    assert goal.from_square == expected_from
    assert goal.to_square == expected_to
    assert goal.piece == expected_piece
    assert goal.piece_color == expected_color
    assert goal.is_capture is expected_capture
    assert goal.captured_square == expected_captured_sq
    assert goal.promotion_piece == expected_promo
    assert goal.is_castling is expected_castle
    if expected_castle:
        assert goal.castling_rook_from == expected_rook_from
        assert goal.castling_rook_to == expected_rook_to

    # Also verify direct instantiation equivalence
    direct = ChessMoveGoal(
        uci=uci,
        piece_type=move_details.piece_type if move_details else "pawn",
        piece_color=expected_color,
        promotion_piece=expected_promo,
        is_capture=expected_capture,
        is_en_passant=bool(move_details and move_details.is_en_passant),
        is_castling=expected_castle,
    )
    assert direct.from_square == expected_from
    assert direct.to_square == expected_to
    assert direct.piece_color == expected_color
    assert direct.is_capture is expected_capture
    assert direct.captured_square == expected_captured_sq
    assert direct.is_castling is expected_castle
    if expected_castle:
        assert direct.castling_rook_from == expected_rook_from
        assert direct.castling_rook_to == expected_rook_to


def test_goal_from_existing_chess_move_goal_identity_preserved():
    """Verify that passing an existing ChessMoveGoal returns it directly without data loss or color corruption."""
    original = ChessMoveGoal(
        uci="f2h2",
        piece_type="rook:pawn",
        piece_color="b",
        promotion_piece="",
        is_capture=True,
        is_en_passant=False,
        is_castling=False,
        force_pick_place=True,
        clear_bin_side="left",
        bypass_steps=("PICK", "PLACE"),
    )
    parsed = ChessMoveGoal.from_uci_or_details(original.uci, move_details=original)
    assert parsed is original
    assert parsed.piece_color == "b"
    assert parsed.color_name == "black"
    assert parsed.opposing_color_name == "white"
    assert parsed.force_pick_place is True
    assert parsed.clear_bin_side == "left"
    assert parsed.bypass_steps == ("PICK", "PLACE")


def test_parse_piece_from_fen():
    """Verify parsing piece type and standardized color ('white'/'black') from FEN."""
    from lekiwi_orchestrator.core.mission_types import parse_piece_from_fen

    initial_fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
    assert parse_piece_from_fen(initial_fen, "c7") == ("pawn", "black")
    assert parse_piece_from_fen(initial_fen, "e2") == ("pawn", "white")
    assert parse_piece_from_fen(initial_fen, "g8") == ("knight", "black")
    assert parse_piece_from_fen(initial_fen, "e4") is None

    midgame_fen = "r1bqkbnr/pppp1ppp/2n5/4p3/4P3/5N2/PPPP1PPP/RNBQKB1R b KQkq - 1 2"
    assert parse_piece_from_fen(midgame_fen, "c6") == ("knight", "black")
    assert parse_piece_from_fen(midgame_fen, "f3") == ("knight", "white")
    assert parse_piece_from_fen(midgame_fen, "invalid") is None
    assert parse_piece_from_fen("", "c6") is None


