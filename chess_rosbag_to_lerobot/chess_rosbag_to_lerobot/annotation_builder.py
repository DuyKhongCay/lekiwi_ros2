# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Steerable Language Annotation Builder for LeRobot v3.0 / v0.6.1.

Generates persistent language records complying with LeRobot's language schema:
- task_aug: 5-axis linguistic augmentations at t=0
- subtask: Contiguous task primitive spans across episode
- plan: Shrinking step-by-step numbered plan at each phase boundary
- memory: First-person past-tense compressed episodic memory (Torne et al., MEM 2026)
"""

from __future__ import annotations

import logging
from typing import Any, Dict, List, Sequence
from chess_rosbag_to_lerobot.trajectory_analyzer import SubtaskSpan

logger = logging.getLogger(__name__)


class AnnotationBuilder:
    """Constructs LeRobot language_persistent records from trajectory plans."""

    @staticmethod
    def build_task_augmentations(
        base_task: str,
        t0_s: float = 0.0,
        target_count: int = 2,
    ) -> List[Dict[str, Any]]:
        """Generate deduplicated task augmentations stamped at t0 up to target_count."""
        cleaned = (base_task or "").strip()
        if not cleaned:
            return []

        # Generate candidates based on synonyms and phrasing templates
        candidates = [
            cleaned,
            f"Please {cleaned[0].lower() + cleaned[1:] if len(cleaned) > 1 else cleaned.lower()}",
            f"Carefully {cleaned[0].lower() + cleaned[1:] if len(cleaned) > 1 else cleaned.lower()}",
            cleaned.replace("Move", "Transfer").replace("move", "transfer"),
            cleaned.replace("Pick", "Grasp").replace("pick", "grasp"),
            cleaned.replace("Place", "Put down").replace("place", "put down"),
            cleaned.replace("Clear", "Remove").replace("clear", "remove"),
            f"Kindly {cleaned[0].lower() + cleaned[1:] if len(cleaned) > 1 else cleaned.lower()}",
            f"Now {cleaned[0].lower() + cleaned[1:] if len(cleaned) > 1 else cleaned.lower()}",
        ]
        unique_variants = list(dict.fromkeys(candidates))[:target_count]

        rows: List[Dict[str, Any]] = []
        for variant in unique_variants:
            rows.append(
                {
                    "role": "user",
                    "content": variant,
                    "style": "task_aug",
                    "timestamp": float(t0_s),
                    "camera": None,
                    "tool_calls": None,
                }
            )
        return rows

    @staticmethod
    def build_subtask_records(spans: Sequence[SubtaskSpan], t0_ns: int = 0) -> List[Dict[str, Any]]:
        """Generate persistent 'subtask' records stamped at each span's start timestamp in relative seconds."""
        rows: List[Dict[str, Any]] = []
        for span in spans:
            start_s = (span.start_ts_ns - t0_ns) * 1e-9
            rows.append(
                {
                    "role": "assistant",
                    "content": span.name,
                    "style": "subtask",
                    "timestamp": max(0.0, float(start_s)),
                    "camera": None,
                    "tool_calls": None,
                }
            )
        return rows

    @staticmethod
    def build_shrinking_plans(spans: Sequence[SubtaskSpan], t0_ns: int = 0) -> List[Dict[str, Any]]:
        """Generate 'plan' records that shrink as work progresses across boundaries."""
        rows: List[Dict[str, Any]] = []
        n = len(spans)
        for i in range(n):
            start_s = (spans[i].start_ts_ns - t0_ns) * 1e-9
            remaining = [s.name for s in spans[i:]]
            plan_text = "; ".join(f"{idx + 1}. {name}" for idx, name in enumerate(remaining))
            rows.append(
                {
                    "role": "assistant",
                    "content": plan_text,
                    "style": "plan",
                    "timestamp": max(0.0, float(start_s)),
                    "camera": None,
                    "tool_calls": None,
                }
            )
        return rows

    @staticmethod
    def build_semantic_memories(
        spans: Sequence[SubtaskSpan],
        canonical_task: str,
        t0_ns: int = 0,
    ) -> List[Dict[str, Any]]:
        """Generate first-person past-tense 'memory' records at phase transitions (MEM, 2026)."""
        rows: List[Dict[str, Any]] = []
        # Memory starts at the boundary of completed subtasks (from span 1 onward)
        for i in range(1, len(spans)):
            start_s = (spans[i].start_ts_ns - t0_ns) * 1e-9
            just_completed = spans[i - 1].name

            if "pick" in just_completed.lower():
                mem_text = "I have successfully grasped the piece from the board."
            elif "place" in just_completed.lower():
                mem_text = "I have delivered and released the piece at the target square."
            elif "clear" in just_completed.lower():
                mem_text = "I have deposited the piece into the designated clearance bin."
            else:
                mem_text = f"I completed: {just_completed.lower()}."

            rows.append(
                {
                    "role": "assistant",
                    "content": mem_text,
                    "style": "memory",
                    "timestamp": max(0.0, float(start_s)),
                    "camera": None,
                    "tool_calls": None,
                }
            )
        return rows

    @classmethod
    def build_all_language_records(
        cls,
        spans: Sequence[SubtaskSpan],
        canonical_task: str,
        t0_ns: Optional[int] = None,
        task_type: Optional[str] = None,
    ) -> List[Dict[str, Any]]:
        """Assemble complete language_persistent payload for an episode in relative seconds.

        Guarantees strict shape consistency of exactly 10 records per episode:
        - For 'move' / 'clear': 2 task_aug + 3 subtasks + 3 plans + 2 memories = 10 records.
        - For 'pick' / 'place': 5 task_aug + 2 subtasks + 2 plans + 1 memory   = 10 records.
        """
        base_t0 = spans[0].start_ts_ns if (t0_ns is None and spans) else (t0_ns or 0)

        # Determine target task_aug count based on task type and number of spans
        t_type = (task_type or "").lower()
        if not t_type and spans:
            # Infer from number of spans: 3 spans -> move/clear, 2 spans -> pick/place
            t_type = "move" if len(spans) >= 3 else "pick"

        if t_type in ("move", "clear") or len(spans) >= 3:
            aug_count = 2
        else:
            aug_count = 5

        all_rows: List[Dict[str, Any]] = []
        all_rows.extend(cls.build_task_augmentations(canonical_task, t0_s=0.0, target_count=aug_count))
        all_rows.extend(cls.build_subtask_records(spans, t0_ns=base_t0))
        all_rows.extend(cls.build_shrinking_plans(spans, t0_ns=base_t0))
        all_rows.extend(cls.build_semantic_memories(spans, canonical_task, t0_ns=base_t0))

        # Sort deterministically by timestamp then style
        all_rows.sort(key=lambda r: (r["timestamp"], r.get("style") or "", r.get("role") or ""))
        return all_rows

    @classmethod
    def attach_language_annotations(
        cls,
        dataset_root: Path,
        episode_to_records: Dict[int, List[Dict[str, Any]]],
    ) -> None:
        """Attach language_persistent and language_events columns to dataset parquet shards.

        Updates meta/info.json schema to register language features according to
        LeRobot v3.0 / v0.6.1 specification.
        """
        import json
        import pyarrow as pa
        import pyarrow.parquet as pq

        data_dir = dataset_root / "data"
        parquet_shards = sorted(list(data_dir.rglob("*.parquet")))
        if not parquet_shards:
            logger.debug("No parquet shards found under %s to attach annotations", data_dir)
            return

        for pfile in parquet_shards:
            table = pq.read_table(pfile)
            if "episode_index" not in table.column_names:
                continue

            ep_indices = table.column("episode_index").to_pylist()
            per_row_persistent: List[List[Dict[str, Any]]] = []
            per_row_events: List[List[Dict[str, Any]]] = []

            for ep_idx in ep_indices:
                records = episode_to_records.get(ep_idx, [])
                per_row_persistent.append(records)
                per_row_events.append([])

            persistent_arr = pa.array(per_row_persistent)
            events_arr = pa.array(per_row_events)

            cols = []
            names = []
            for name in table.column_names:
                if name in ("language_persistent", "language_events"):
                    continue
                cols.append(table.column(name))
                names.append(name)

            cols.extend([persistent_arr, events_arr])
            names.extend(["language_persistent", "language_events"])
            new_table = pa.Table.from_arrays(cols, names=names)
            pq.write_table(new_table, pfile)

        # Update meta/info.json features
        info_path = dataset_root / "meta" / "info.json"
        if info_path.is_file():
            try:
                info = json.loads(info_path.read_text(encoding="utf-8"))
                info.setdefault("features", {})
                info["features"]["language_persistent"] = {
                    "dtype": "language",
                    "shape": [1],
                    "names": None,
                }
                info["features"]["language_events"] = {
                    "dtype": "language",
                    "shape": [1],
                    "names": None,
                }
                info_path.write_text(json.dumps(info, indent=2), encoding="utf-8")
                logger.info("Successfully updated %s with language features", info_path)
            except Exception as exc:
                logger.warning("Failed to update %s with language features: %s", info_path, exc)
