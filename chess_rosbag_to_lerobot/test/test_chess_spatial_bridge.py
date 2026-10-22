# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

import pytest
import numpy as np

from chess_rosbag_to_lerobot.config import (
    ChessboardConfig,
    Config,
    LandmarksConfig,
    PhaseConditioningConfig,
)
from chess_rosbag_to_lerobot.chess_spatial_bridge import (
    ClearBinSide,
    EpisodeSpatialPlan,
    KinematicMilestones,
    SpatialEndpoint,
    TaskType,
    assemble_environment_state,
    condition_episode_frames,
    detect_kinematic_milestones,
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


def test_resolve_episode_spatial_plan_and_assemble_12d():
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
    assert np.allclose(plan_move.stow_xyz, [0.070, -0.188, 0.175])

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

    # Assemble 12D environment state vector
    def mock_transform(pt: np.ndarray) -> np.ndarray:
        return pt + np.array([1.0, 2.0, 3.0], dtype=np.float32)

    vec_12d = assemble_environment_state(plan_clear, mock_transform)
    assert vec_12d.shape == (12,)
    assert vec_12d.dtype == np.float32

    # Source was in board frame -> mock_transform was applied
    assert np.allclose(vec_12d[:3], plan_clear.source.xyz + np.array([1.0, 2.0, 3.0]))
    # Target was in base_footprint (is_board_frame=False) -> mock_transform was NOT applied
    assert np.allclose(vec_12d[3:6], plan_clear.target.xyz)
    # One-hot is clear [0, 1, 0, 0]
    assert np.allclose(vec_12d[6:10], [0.0, 1.0, 0.0, 0.0])
    # Latches default to [0, 0]
    assert np.allclose(vec_12d[10:12], [0.0, 0.0])


def test_assemble_environment_state_all_phases_and_override():
    """Verify 12D assembly across all 4 tasks, phase combinations, and stow override."""
    cfg = ChessboardConfig(
        landmarks=LandmarksConfig(stow=[0.10, -0.20, 0.15]),
    )
    def dummy_tf(pt: np.ndarray) -> np.ndarray:
        return pt

    tasks = ["Pick c7, place c5", "Clear d4 to left onboard bin", "Pick d5", "Place e4"]
    for task_str in tasks:
        plan = resolve_episode_spatial_plan(task_str, cfg)

        # Phase 0: [0, 0]
        v0 = assemble_environment_state(plan, dummy_tf, has_grasped=0.0, has_released=0.0)
        assert v0.shape == (12,)
        assert np.allclose(v0[10:12], [0.0, 0.0])

        # Phase 1: [1, 0]
        v1 = assemble_environment_state(plan, dummy_tf, has_grasped=1.0, has_released=0.0)
        assert np.allclose(v1[10:12], [1.0, 0.0])

        # Phase 2: [1, 1] with override_to_stow=True
        v2 = assemble_environment_state(
            plan, dummy_tf, has_grasped=1.0, has_released=1.0, override_to_stow=True
        )
        assert np.allclose(v2[10:12], [1.0, 1.0])
        # Crucial invariant: when override_to_stow=True, both src and tgt are set to stow_xyz
        assert np.allclose(v2[:3], plan.stow_xyz)
        assert np.allclose(v2[3:6], plan.stow_xyz)


def _generate_synthetic_trajectory(
    task_type: TaskType,
    n_lead_idle: int = 15,
    n_transport: int = 20,
    n_tail_hold: int = 40,
) -> list[np.ndarray]:
    """Helper creating joint state trajectory [pan, lift, elbow, wrist_flex, wrist_roll, gripper]."""
    stow_q = np.array([1.55, -1.73, 1.55, 0.0, 0.0, 0.0], dtype=np.float32)
    traj: list[np.ndarray] = []

    # 1. Leading idle frames at stow
    for _ in range(n_lead_idle):
        traj.append(stow_q.copy())

    # 2. Movement onset & reach
    # Open gripper and lift arm
    reach_steps = 10
    for s in range(reach_steps):
        q = stow_q.copy()
        q[0] += 0.05 * (s + 1)  # motion onset
        q[1] = -1.73 + (1.2 * (s + 1) / reach_steps)  # lift reaches > -1.0
        q[5] = 0.25  # gripper opened > 0.22
        traj.append(q)

    # 3. Grasp event (for MOVE, CLEAR, PICK)
    if task_type != TaskType.PLACE:
        q_grasp = traj[-1].copy()
        q_grasp[5] = 0.05  # gripper closed < 0.12 while lift > -1.0
        traj.append(q_grasp)

    # 4. Transport piece
    for s in range(n_transport):
        q = traj[-1].copy()
        q[0] += 0.02
        traj.append(q)

    # 5. Release event (for MOVE, CLEAR, PLACE)
    if task_type != TaskType.PICK:
        q_rel = traj[-1].copy()
        q_rel[5] = 0.30  # gripper open > 0.22
        traj.append(q_rel)

    # 6. Retract to stow
    retract_steps = 10
    for s in range(retract_steps):
        q = traj[-1].copy()
        q[1] = -0.5 - (1.23 * (s + 1) / retract_steps)  # reaches -1.73
        q[5] = 0.0
        traj.append(q)

    # 7. Trailing hold at stow (lift < -1.35)
    for _ in range(n_tail_hold):
        traj.append(stow_q.copy())

    return traj


def test_kinematic_milestones_and_conditioning_move():
    """Verify milestone detection and frame conditioning for MOVE task."""
    cb_cfg = ChessboardConfig()
    plan = resolve_episode_spatial_plan("Pick c7, place c5", cb_cfg)
    cfg = Config(
        robot_type="lekiwi",
        fps=25,
        reference_topic="/cam",
        default_max_age_s=0.1,
        task="Pick c7, place c5",
        chessboard=cb_cfg,
        phase_conditioning=PhaseConditioningConfig(
            trim_leading_idle_frames=5,
            target_trailing_stow_frames=25,
            motion_onset_delta_rad=0.03,
            stow_settle_delta_rad=0.02,
            arm_extended_lift_min_rad=-1.0,
            gripper_open_min_rad=0.22,
            gripper_closed_max_rad=0.12,
            min_transport_duration_s=0.5,
        ),
    )

    states = _generate_synthetic_trajectory(TaskType.MOVE, n_lead_idle=15, n_tail_hold=40)
    raw_frames = [
        {"observation.state": q.copy(), "action": q.copy()}
        for q in states
    ]

    milestones = detect_kinematic_milestones(states, TaskType.MOVE, cfg.phase_conditioning, 25)
    assert milestones.onset_idx == 15
    assert milestones.grasp_idx is not None and milestones.grasp_idx > milestones.onset_idx
    assert milestones.release_idx is not None and milestones.release_idx > milestones.grasp_idx
    assert milestones.settle_idx > milestones.release_idx

    conditioned = condition_episode_frames(raw_frames, plan, cfg)

    # Trimmed leading idle: exactly 5 frames before onset (onset was 15 -> start at 10)
    # Trailing hold: clamped from 40 down to exactly 25 frames
    assert len(conditioned) > 0
    # First frame has [0, 0] latch
    assert np.allclose(conditioned[0]["observation.environment_state"][10:12], [0.0, 0.0])
    # Last frame has [1, 1] latch and stow override
    assert np.allclose(conditioned[-1]["observation.environment_state"][10:12], [1.0, 1.0])
    assert np.allclose(conditioned[-1]["observation.environment_state"][:3], plan.stow_xyz)
    assert np.allclose(conditioned[-1]["observation.environment_state"][3:6], plan.stow_xyz)


def test_kinematic_conditioning_pick_place_and_padding():
    """Verify PICK (no release), PLACE (already grasped), and tail padding when hold < 25 frames."""
    cb_cfg = ChessboardConfig()
    cfg = Config(
        robot_type="lekiwi",
        fps=25,
        reference_topic="/cam",
        default_max_age_s=0.1,
        task="test",
        chessboard=cb_cfg,
        phase_conditioning=PhaseConditioningConfig(
            trim_leading_idle_frames=5,
            target_trailing_stow_frames=25,
        ),
    )

    # 1. PICK test (has_released is always 0.0, overrides to stow after grasp)
    plan_pick = resolve_episode_spatial_plan("Pick d5", cb_cfg)
    states_pick = _generate_synthetic_trajectory(TaskType.PICK, n_lead_idle=10, n_tail_hold=30)
    raw_pick = [{"observation.state": q.copy(), "action": q.copy()} for q in states_pick]
    cond_pick = condition_episode_frames(raw_pick, plan_pick, cfg)

    # Final frame of PICK must have has_grasped=1.0 and has_released=0.0
    assert np.allclose(cond_pick[-1]["observation.environment_state"][10:12], [1.0, 0.0])
    assert np.allclose(cond_pick[-1]["observation.environment_state"][:3], plan_pick.stow_xyz)
    assert np.allclose(cond_pick[-1]["observation.environment_state"][3:6], plan_pick.stow_xyz)

    # 2. PLACE test (starts with has_grasped=1.0, switches to [1, 1] after release)
    plan_place = resolve_episode_spatial_plan("Place e4", cb_cfg)
    states_place = _generate_synthetic_trajectory(TaskType.PLACE, n_lead_idle=10, n_tail_hold=30)
    raw_place = [{"observation.state": q.copy(), "action": q.copy()} for q in states_place]
    cond_place = condition_episode_frames(raw_place, plan_place, cfg)

    assert np.allclose(cond_place[0]["observation.environment_state"][10:12], [1.0, 0.0])
    assert np.allclose(cond_place[-1]["observation.environment_state"][10:12], [1.0, 1.0])

    # 3. Trailing stow hold padding (n_tail < 25)
    states_short_tail = _generate_synthetic_trajectory(TaskType.MOVE, n_lead_idle=10, n_tail_hold=5)
    raw_short = [{"observation.state": q.copy(), "action": np.ones(6, dtype=np.float32)} for q in states_short_tail]
    cond_short = condition_episode_frames(raw_short, resolve_episode_spatial_plan("Pick c7, place c5", cb_cfg), cfg)

    # Check that padded frames at end have action == observation.state (zero velocity attractor)
    assert np.allclose(cond_short[-1]["action"], cond_short[-1]["observation.state"])


def test_milestone_detection_raises_value_error_on_missing_events():
    """Verify explicit ValueError when required events are missing from trajectory."""
    pc = PhaseConditioningConfig()
    stow_q = np.array([1.55, -1.73, 1.55, 0.0, 0.0, 0.0], dtype=np.float32)

    # Trajectory that stays at stow (never grasps)
    idle_states = [stow_q.copy() for _ in range(50)]
    with pytest.raises(ValueError) as exc:
        detect_kinematic_milestones(idle_states, TaskType.MOVE, pc, 25)
    assert "Missing grasp event" in str(exc.value)

    # Empty states
    with pytest.raises(ValueError):
        detect_kinematic_milestones([], TaskType.MOVE, pc, 25)
