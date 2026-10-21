# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

import pytest
import numpy as np

from chess_rosbag_to_lerobot.config import ChessboardConfig, LandmarksConfig
from chess_rosbag_to_lerobot.chess_spatial_bridge import (
    ClearBinSide,
    EpisodeSpatialPlan,
    SpatialEndpoint,
    TaskType,
    assemble_environment_state,
    parse_chess_task,
    resolve_episode_spatial_plan,
    square_to_metric,
)


def test_square_to_metric():
    cfg = ChessboardConfig(
        square_size_m=0.040,
        grasp_clearance_z_m=0.025,
    )

    # d4: file_idx=3 -> (3 - 3.5)*0.040 = -0.020, rank_idx=3 -> -0.020
    x, y, z = square_to_metric("d4", cfg)
    assert pytest.approx(x, abs=1e-5) == -0.020
    assert pytest.approx(y, abs=1e-5) == -0.020
    assert pytest.approx(z, abs=1e-5) == 0.025

    # e5: file_idx=4 -> +0.020, rank_idx=4 -> +0.020
    x, y, z = square_to_metric("e5", cfg)
    assert pytest.approx(x, abs=1e-5) == 0.020
    assert pytest.approx(y, abs=1e-5) == 0.020
    assert pytest.approx(z, abs=1e-5) == 0.025

    with pytest.raises(ValueError):
        square_to_metric("z9", cfg)
    with pytest.raises(ValueError):
        square_to_metric("e", cfg)


def test_parse_chess_task_primitives():
    # MOVE
    res = parse_chess_task("Pick c7, place c5")
    assert res.task_type == TaskType.MOVE
    assert res.from_square == "c7"
    assert res.to_square == "c5"

    res = parse_chess_task("e2e4")
    assert res.task_type == TaskType.MOVE
    assert res.from_square == "e2"
    assert res.to_square == "e4"

    # CLEAR (explicit)
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

    # PICK (atomic)
    res_pick = parse_chess_task("Pick d5")
    assert res_pick.task_type == TaskType.PICK
    assert res_pick.from_square == "d5"

    # PLACE (atomic)
    res_place = parse_chess_task("Place e4")
    assert res_place.task_type == TaskType.PLACE
    assert res_place.to_square == "e4"


def test_parse_chess_task_standard_value_error():
    """Verify that ambiguous clear or invalid syntax raises standard Python ValueError."""
    # Ambiguous CLEAR without side must raise standard ValueError
    with pytest.raises(ValueError) as excinfo:
        parse_chess_task("Clear d4 to onboard bin")
    assert "Ambiguous CLEAR task" in str(excinfo.value)

    with pytest.raises(ValueError):
        parse_chess_task("Clear d4")

    # Invalid / empty string raises ValueError
    with pytest.raises(ValueError):
        parse_chess_task("")

    with pytest.raises(ValueError):
        parse_chess_task("Invalid task instruction without squares")


def test_resolve_episode_spatial_plan_and_assemble_10d():
    cfg = ChessboardConfig(
        square_size_m=0.040,
        grasp_clearance_z_m=0.025,
        landmarks=LandmarksConfig(
            stow=[0.070, -0.188, 0.175],
            clear_left=[-0.003, 0.143, 0.170],
            clear_right=[-0.003, -0.145, 0.163],
        ),
    )

    # 1. MOVE plan
    plan_move = resolve_episode_spatial_plan("Pick c7, place c5", cfg)
    assert plan_move.task_type == TaskType.MOVE
    assert plan_move.canonical_task == "Pick c7, place c5"
    assert plan_move.source.is_board_frame is True
    assert plan_move.target.is_board_frame is True
    assert np.array_equal(plan_move.task_one_hot, [1.0, 0.0, 0.0, 0.0])

    # 2. CLEAR plan
    plan_clear = resolve_episode_spatial_plan("Clear d4 to left onboard bin", cfg)
    assert plan_clear.task_type == TaskType.CLEAR
    assert plan_clear.canonical_task == "Clear d4 to left onboard bin"
    assert plan_clear.source.is_board_frame is True
    assert plan_clear.target.is_board_frame is False  # Landmark is in base_footprint!
    assert np.allclose(plan_clear.target.xyz, [-0.003, 0.143, 0.170])
    assert np.array_equal(plan_clear.task_one_hot, [0.0, 1.0, 0.0, 0.0])

    # 3. PICK plan
    plan_pick = resolve_episode_spatial_plan("Pick d5", cfg)
    assert plan_pick.task_type == TaskType.PICK
    assert plan_pick.source.is_board_frame is True
    assert plan_pick.target.is_board_frame is False
    assert np.allclose(plan_pick.target.xyz, [0.070, -0.188, 0.175])
    assert np.array_equal(plan_pick.task_one_hot, [0.0, 0.0, 1.0, 0.0])

    # 4. PLACE plan
    plan_place = resolve_episode_spatial_plan("Place e4", cfg)
    assert plan_place.task_type == TaskType.PLACE
    assert plan_place.source.is_board_frame is False
    assert plan_place.target.is_board_frame is True
    assert np.allclose(plan_place.source.xyz, [0.070, -0.188, 0.175])
    assert np.array_equal(plan_place.task_one_hot, [0.0, 0.0, 0.0, 1.0])

    # Assemble 10D environment state vector
    def mock_transform(pt: np.ndarray) -> np.ndarray:
        return pt + np.array([1.0, 2.0, 3.0], dtype=np.float32)

    vec_10d = assemble_environment_state(plan_clear, mock_transform)
    assert vec_10d.shape == (10,)
    assert vec_10d.dtype == np.float32

    # Source was in board frame -> mock_transform was applied
    assert np.allclose(vec_10d[:3], plan_clear.source.xyz + np.array([1.0, 2.0, 3.0]))
    # Target was in base_footprint (is_board_frame=False) -> mock_transform was NOT applied
    assert np.allclose(vec_10d[3:6], plan_clear.target.xyz)
    # One-hot is clear [0, 1, 0, 0]
    assert np.allclose(vec_10d[6:10], [0.0, 1.0, 0.0, 0.0])
