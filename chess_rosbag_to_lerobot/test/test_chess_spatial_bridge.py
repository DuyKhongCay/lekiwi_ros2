# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

import pytest
import numpy as np

from chess_rosbag_to_lerobot.config import ChessboardConfig
from chess_rosbag_to_lerobot.chess_spatial_bridge import (
    square_to_metric,
    parse_uci_move,
    extract_move_squares,
    resolve_pick_place_poses,
)


def test_square_to_metric_center_origin():
    cfg = ChessboardConfig(
        square_size_m=0.040,
        grasp_clearance_z_m=0.025,
        origin_at_center=True,
    )

    # Center is between d/e and 4/5:
    # d4: file_idx=3 -> (3 - 3.5)*0.040 = -0.020
    #     rank_idx=3 -> (3 - 3.5)*0.040 = -0.020
    x, y, z = square_to_metric("d4", cfg)
    assert pytest.approx(x, abs=1e-5) == -0.020
    assert pytest.approx(y, abs=1e-5) == -0.020
    assert pytest.approx(z, abs=1e-5) == 0.025

    # e5: file_idx=4 -> (4 - 3.5)*0.040 = +0.020
    #     rank_idx=4 -> (4 - 3.5)*0.040 = +0.020
    x, y, z = square_to_metric("e5", cfg)
    assert pytest.approx(x, abs=1e-5) == 0.020
    assert pytest.approx(y, abs=1e-5) == 0.020

    # a1: file_idx=0 -> -3.5 * 0.040 = -0.140
    #     rank_idx=0 -> -3.5 * 0.040 = -0.140
    x, y, z = square_to_metric("a1", cfg)
    assert pytest.approx(x, abs=1e-5) == -0.140
    assert pytest.approx(y, abs=1e-5) == -0.140

    # h8: file_idx=7 -> +3.5 * 0.040 = +0.140
    #     rank_idx=7 -> +3.5 * 0.040 = +0.140
    x, y, z = square_to_metric("h8", cfg)
    assert pytest.approx(x, abs=1e-5) == 0.140
    assert pytest.approx(y, abs=1e-5) == 0.140


def test_invalid_square_raises():
    cfg = ChessboardConfig()
    with pytest.raises(ValueError):
        square_to_metric("z9", cfg)
    with pytest.raises(ValueError):
        square_to_metric("e", cfg)


def test_parse_uci_move():
    from_sq, to_sq = parse_uci_move("e2e4")
    assert from_sq == "e2"
    assert to_sq == "e4"

    from_sq, to_sq = parse_uci_move("e7e8q")
    assert from_sq == "e7"
    assert to_sq == "e8"


def test_extract_move_squares():
    assert extract_move_squares("e2e4") == ("e2", "e4")
    assert extract_move_squares("Move white pawn from e2 to e4") == ("e2", "e4")
    assert extract_move_squares("Capture black knight on f6 with g5") == ("f6", "g5")


def test_resolve_pick_place_poses():
    cfg = ChessboardConfig(square_size_m=0.040, grasp_clearance_z_m=0.025, origin_at_center=True)
    pick_pose, place_pose = resolve_pick_place_poses("e2e4", cfg)

    assert isinstance(pick_pose, np.ndarray)
    assert isinstance(place_pose, np.ndarray)
    assert pick_pose.shape == (3,)
    assert place_pose.shape == (3,)

    # e2: file 'e' (4) -> +0.020; rank '2' (1) -> (1-3.5)*0.04 = -0.100
    assert pytest.approx(pick_pose[0], abs=1e-5) == 0.020
    assert pytest.approx(pick_pose[1], abs=1e-5) == -0.100
    assert pytest.approx(pick_pose[2], abs=1e-5) == 0.025

    # e4: file 'e' (4) -> +0.020; rank '4' (3) -> (3-3.5)*0.04 = -0.020
    assert pytest.approx(place_pose[0], abs=1e-5) == 0.020
    assert pytest.approx(place_pose[1], abs=1e-5) == -0.020
    assert pytest.approx(place_pose[2], abs=1e-5) == 0.025
