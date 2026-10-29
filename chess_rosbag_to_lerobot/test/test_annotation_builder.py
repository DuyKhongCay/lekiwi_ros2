# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unit tests for AnnotationBuilder and LeRobot steerable language pipeline."""

import json
from pathlib import Path
import tempfile
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
import pytest

from chess_rosbag_to_lerobot.annotation_builder import AnnotationBuilder
from chess_rosbag_to_lerobot.trajectory_analyzer import SubtaskSpan


@pytest.fixture
def sample_spans() -> list[SubtaskSpan]:
    return [
        SubtaskSpan(
            name="Pick piece at source",
            goal_xyz=np.array([0.1, -0.05, 0.035], dtype=np.float32),
            start_ts_ns=0,
            end_ts_ns=2_000_000_000,
        ),
        SubtaskSpan(
            name="Place piece to target",
            goal_xyz=np.array([0.2, 0.08, 0.035], dtype=np.float32),
            start_ts_ns=2_000_000_000,
            end_ts_ns=4_500_000_000,
        ),
        SubtaskSpan(
            name="Retract arm to stow",
            goal_xyz=np.array([0.07, -0.18, 0.175], dtype=np.float32),
            start_ts_ns=4_500_000_000,
            end_ts_ns=6_000_000_000,
        ),
    ]


def test_build_task_augmentations():
    base_task = "Move white pawn to target"
    records_default = AnnotationBuilder.build_task_augmentations(base_task, t0_s=0.0)
    assert len(records_default) == 2  # default target_count is 2
    for r in records_default:
        assert r["role"] == "user"
        assert r["style"] == "task_aug"
        assert r["timestamp"] == 0.0

    records_5 = AnnotationBuilder.build_task_augmentations(base_task, t0_s=0.0, target_count=5)
    assert len(records_5) == 5


def test_build_all_language_records_fixed_10_shape(sample_spans):
    """Verify that both move/clear (3 spans) and pick/place (2 spans) yield exactly 10 records."""
    # 1. move task (3 spans): 2 task_aug + 3 subtasks + 3 plans + 2 memories = 10
    move_records = AnnotationBuilder.build_all_language_records(
        sample_spans, "Move black knight to target", task_type="move"
    )
    assert len(move_records) == 10
    move_styles = [r["style"] for r in move_records]
    assert move_styles.count("task_aug") == 2
    assert move_styles.count("subtask") == 3
    assert move_styles.count("plan") == 3
    assert move_styles.count("memory") == 2

    # 2. clear task (3 spans): 2 task_aug + 3 subtasks + 3 plans + 2 memories = 10
    clear_records = AnnotationBuilder.build_all_language_records(
        sample_spans, "Clear white pawn to left bin", task_type="clear"
    )
    assert len(clear_records) == 10
    clear_styles = [r["style"] for r in clear_records]
    assert clear_styles.count("task_aug") == 2
    assert clear_styles.count("subtask") == 3
    assert clear_styles.count("plan") == 3
    assert clear_styles.count("memory") == 2

    # 3. pick task (2 spans): 5 task_aug + 2 subtasks + 2 plans + 1 memory = 10
    pick_spans = sample_spans[:2]
    pick_records = AnnotationBuilder.build_all_language_records(
        pick_spans, "Pick white pawn", task_type="pick"
    )
    assert len(pick_records) == 10
    pick_styles = [r["style"] for r in pick_records]
    assert pick_styles.count("task_aug") == 5
    assert pick_styles.count("subtask") == 2
    assert pick_styles.count("plan") == 2
    assert pick_styles.count("memory") == 1

    # 4. place task (2 spans): 5 task_aug + 2 subtasks + 2 plans + 1 memory = 10
    place_records = AnnotationBuilder.build_all_language_records(
        pick_spans, "Place white pawn", task_type="place"
    )
    assert len(place_records) == 10
    place_styles = [r["style"] for r in place_records]
    assert place_styles.count("task_aug") == 5
    assert place_styles.count("subtask") == 2
    assert place_styles.count("plan") == 2
    assert place_styles.count("memory") == 1


def test_build_subtask_records(sample_spans):
    records = AnnotationBuilder.build_subtask_records(sample_spans)
    assert len(records) == 3

    assert records[0]["role"] == "assistant"
    assert records[0]["style"] == "subtask"
    assert records[0]["content"] == "Pick piece at source"
    assert records[0]["timestamp"] == 0.0

    assert records[1]["content"] == "Place piece to target"
    assert records[1]["timestamp"] == 2.0

    assert records[2]["content"] == "Retract arm to stow"
    assert records[2]["timestamp"] == 4.5


def test_build_shrinking_plans(sample_spans):
    records = AnnotationBuilder.build_shrinking_plans(sample_spans)
    assert len(records) == 3

    for r in records:
        assert r["role"] == "assistant"
        assert r["style"] == "plan"

    # Plan at t=0 has all 3 steps
    assert "1. Pick piece at source; 2. Place piece to target; 3. Retract arm to stow" in records[0]["content"]
    assert records[0]["timestamp"] == 0.0

    # Plan at t=2.0s has remaining 2 steps
    assert "1. Place piece to target; 2. Retract arm to stow" in records[1]["content"]
    assert records[1]["timestamp"] == 2.0

    # Plan at t=4.5s has remaining 1 step
    assert "1. Retract arm to stow" in records[2]["content"]
    assert records[2]["timestamp"] == 4.5


def test_build_semantic_memories(sample_spans):
    records = AnnotationBuilder.build_semantic_memories(sample_spans, "Move white pawn to target")
    # Memories are generated at phase boundaries after completion (indices 1 and 2)
    assert len(records) == 2

    assert records[0]["role"] == "assistant"
    assert records[0]["style"] == "memory"
    assert records[0]["timestamp"] == 2.0
    assert "grasped" in records[0]["content"]

    assert records[1]["style"] == "memory"
    assert records[1]["timestamp"] == 4.5
    assert "released" in records[1]["content"]


def test_attach_language_annotations(sample_spans):
    """Test parquet schema injection and info.json registration."""
    with tempfile.TemporaryDirectory() as tmpdir:
        root = Path(tmpdir)
        data_dir = root / "data" / "chunk-000"
        data_dir.mkdir(parents=True, exist_ok=True)
        meta_dir = root / "meta"
        meta_dir.mkdir(parents=True, exist_ok=True)

        # Write dummy info.json
        info_path = meta_dir / "info.json"
        info_path.write_text(json.dumps({"features": {}}), encoding="utf-8")

        # Write dummy parquet file with 2 episodes
        parquet_path = data_dir / "episode_000000.parquet"
        dummy_table = pa.Table.from_pydict({
            "episode_index": [0, 0, 1, 1],
            "frame_index": [0, 1, 0, 1],
        })
        pq.write_table(dummy_table, parquet_path)

        # Prepare annotations for episodes 0 and 1
        all_records_ep0 = AnnotationBuilder.build_all_language_records(sample_spans, "Move white pawn")
        all_records_ep1 = AnnotationBuilder.build_all_language_records(sample_spans, "Move black knight")

        AnnotationBuilder.attach_language_annotations(
            root,
            {0: all_records_ep0, 1: all_records_ep1},
        )

        # Verify parquet columns
        updated_table = pq.read_table(parquet_path)
        assert "language_persistent" in updated_table.column_names
        assert "language_events" in updated_table.column_names
        assert len(updated_table) == 4

        # Read back row 0 language_persistent
        row0_persistent = updated_table.column("language_persistent")[0].as_py()
        assert len(row0_persistent) > 0
        styles = {rec["style"] for rec in row0_persistent}
        assert "task_aug" in styles
        assert "subtask" in styles
        assert "plan" in styles
        assert "memory" in styles

        # Verify info.json
        updated_info = json.loads(info_path.read_text(encoding="utf-8"))
        assert "language_persistent" in updated_info["features"]
        assert updated_info["features"]["language_persistent"]["dtype"] == "language"
        assert "language_events" in updated_info["features"]


def test_build_records_relative_timestamps_from_epoch():
    """Verify that epoch timestamps in nanoseconds are correctly converted to relative seconds starting at 0.0."""
    epoch_base_ns = 1_792_765_700_000_000_000  # ~1.79 billion seconds
    spans = [
        SubtaskSpan("Pick piece", np.zeros(3), epoch_base_ns, epoch_base_ns + 10_000_000_000),
        SubtaskSpan("Place piece", np.zeros(3), epoch_base_ns + 10_000_000_000, epoch_base_ns + 20_000_000_000),
    ]

    records = AnnotationBuilder.build_all_language_records(spans, "Move piece")
    # All timestamps must be in [0.0, 20.0], NOT in billions of seconds
    timestamps = [r["timestamp"] for r in records]
    assert min(timestamps) == 0.0
    assert max(timestamps) <= 20.0
    for r in records:
        assert r["timestamp"] < 1000.0, f"Timestamp {r['timestamp']} is not relative seconds!"

