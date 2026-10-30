# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

from pathlib import Path
import pytest

from chess_rosbag_to_lerobot.chess_spatial_bridge import (
    ClearBinSide,
    TaskType,
    parse_chess_task,
    sort_episodes,
)


def test_parse_chess_task_smolvla_instructions():
    # 1. MOVE without square brackets (orchestrator default)
    res = parse_chess_task("Move black pawn to target")
    assert res.task_type == TaskType.MOVE
    assert res.piece_color == "black"
    assert res.piece_type == "pawn"
    assert res.from_square is None
    assert res.to_square is None

    # MOVE with square brackets
    res_sq = parse_chess_task("Move white knight to target [g1 to f3]")
    assert res_sq.task_type == TaskType.MOVE
    assert res_sq.piece_color == "white"
    assert res_sq.piece_type == "knight"
    assert res_sq.from_square == "g1"
    assert res_sq.to_square == "f3"

    # 2. CLEAR without square brackets
    res_clear = parse_chess_task("Clear white knight to left bin")
    assert res_clear.task_type == TaskType.CLEAR
    assert res_clear.piece_color == "white"
    assert res_clear.piece_type == "knight"
    assert res_clear.clear_bin_side == ClearBinSide.LEFT
    assert res_clear.from_square is None

    # CLEAR with square brackets and right bin
    res_clear_sq = parse_chess_task("Clear black queen to right bin [d8]")
    assert res_clear_sq.task_type == TaskType.CLEAR
    assert res_clear_sq.piece_color == "black"
    assert res_clear_sq.piece_type == "queen"
    assert res_clear_sq.clear_bin_side == ClearBinSide.RIGHT
    assert res_clear_sq.from_square == "d8"

    # 3. PICK without and with square brackets
    res_pick = parse_chess_task("Pick black bishop")
    assert res_pick.task_type == TaskType.PICK
    assert res_pick.piece_color == "black"
    assert res_pick.piece_type == "bishop"
    assert res_pick.from_square is None

    res_pick_sq = parse_chess_task("Pick white king [e1]")
    assert res_pick_sq.task_type == TaskType.PICK
    assert res_pick_sq.piece_color == "white"
    assert res_pick_sq.piece_type == "king"
    assert res_pick_sq.from_square == "e1"

    # 4. PLACE without and with square brackets
    res_place = parse_chess_task("Place black bishop")
    assert res_place.task_type == TaskType.PLACE
    assert res_place.piece_color == "black"
    assert res_place.piece_type == "bishop"
    assert res_place.to_square is None

    res_place_sq = parse_chess_task("Place white rook [a1]")
    assert res_place_sq.task_type == TaskType.PLACE
    assert res_place_sq.piece_color == "white"
    assert res_place_sq.piece_type == "rook"
    assert res_place_sq.to_square == "a1"


def test_parse_chess_task_primitives():
    # Legacy MOVE
    res = parse_chess_task("Pick c7, place c5")
    assert res.task_type == TaskType.MOVE
    assert res.from_square == "c7"
    assert res.to_square == "c5"

    res = parse_chess_task("e2e4")
    assert res.task_type == TaskType.MOVE
    assert res.from_square == "e2"
    assert res.to_square == "e4"

    # Legacy CLEAR (explicit)
    res_left = parse_chess_task("Clear d4 to left onboard bin")
    assert res_left.task_type == TaskType.CLEAR
    assert res_left.from_square == "d4"
    assert res_left.clear_bin_side == ClearBinSide.LEFT

    res_right = parse_chess_task("Clear d4 to right onboard bin")
    assert res_right.task_type == TaskType.CLEAR
    assert res_right.from_square == "d4"
    assert res_right.clear_bin_side == ClearBinSide.RIGHT

    res_prefix = parse_chess_task("clear_left e5")
    assert res_prefix.task_type == TaskType.CLEAR
    assert res_prefix.from_square == "e5"
    assert res_prefix.clear_bin_side == ClearBinSide.LEFT

    # Legacy PICK (atomic)
    res_pick = parse_chess_task("Pick d5")
    assert res_pick.task_type == TaskType.PICK
    assert res_pick.from_square == "d5"

    # Legacy PLACE (atomic)
    res_place = parse_chess_task("Place e4")
    assert res_place.task_type == TaskType.PLACE
    assert res_place.to_square == "e4"


def test_parse_chess_task_standard_value_error():
    """Verify that ambiguous clear or invalid syntax raises standard Python ValueError."""
    with pytest.raises(ValueError) as excinfo:
        parse_chess_task("Clear d4 to onboard bin")
    assert "Ambiguous CLEAR task" in str(excinfo.value)

    with pytest.raises(ValueError):
        parse_chess_task("Clear d4")

    with pytest.raises(ValueError):
        parse_chess_task("")

    with pytest.raises(ValueError):
        parse_chess_task("Invalid task instruction without squares")


def test_sort_episodes():
    """Verify deterministic sorting of episodes: Task (MOVE, CLEAR, PICK, PLACE) -> Piece -> Color -> ID."""
    mock_episodes = [
        Path("episode_000004"),  # PLACE white pawn
        Path("episode_000002"),  # CLEAR white knight to left bin
        Path("episode_000001"),  # MOVE black pawn to target
        Path("episode_000003"),  # PICK white rook
        Path("episode_000000"),  # MOVE white pawn to target
        Path("episode_000005"),  # MOVE white knight to target
    ]

    task_map = {
        "episode_000000": "Move white pawn to target",
        "episode_000001": "Move black pawn to target",
        "episode_000002": "Clear white knight to left bin",
        "episode_000003": "Pick white rook",
        "episode_000004": "Place white pawn",
        "episode_000005": "Move white knight to target",
    }

    sorted_eps = sort_episodes(mock_episodes, task_extractor=lambda p: task_map.get(p.name, ""))
    sorted_names = [p.name for p in sorted_eps]

    # Expected order:
    # 1. MOVE white pawn (ep_000000)
    # 2. MOVE black pawn (ep_000001)
    # 3. MOVE white knight (ep_000005)
    # 4. CLEAR white knight (ep_000002)
    # 5. PICK white rook (ep_000003)
    # 6. PLACE white pawn (ep_000004)
    expected = [
        "episode_000000",
        "episode_000001",
        "episode_000005",
        "episode_000002",
        "episode_000003",
        "episode_000004",
    ]
    assert sorted_names == expected
