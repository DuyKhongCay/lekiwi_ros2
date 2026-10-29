# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Trajectory analysis and teleoperation false event filtering.

Conforms to Single Responsibility & Open/Closed Principles:
- Analyzes /joint_states and TF streams to extract key manipulation events.
- Applies 4-layer false event filtering (Z-height, Dwell time, Bounds, Temporal ordering).
- Decomposes episodes into subtask phases with active 3D Cartesian goals.
"""

from __future__ import annotations

from dataclasses import dataclass
import logging
from typing import List, Optional, Sequence, Tuple
import numpy as np

from chess_rosbag_to_lerobot.config import LandmarksConfig, TeleopFilterConfig
from chess_rosbag_to_lerobot.tf_replayer import TfReplayer

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class SubtaskSpan:
    """A contiguous temporal subtask segment within an episode."""
    name: str
    goal_xyz: np.ndarray  # shape (3,), float32 in base_footprint frame
    start_ts_ns: int
    end_ts_ns: int


@dataclass(frozen=True)
class EpisodeTrajectoryPlan:
    """Full trajectory execution plan extracted from rosbag data."""
    task_type: str
    canonical_task: str
    source_xyz: np.ndarray
    target_xyz: np.ndarray
    stow_xyz: np.ndarray
    t_pick_ns: Optional[int]
    t_place_ns: Optional[int]
    subtask_spans: List[SubtaskSpan]

    def get_active_subtask(self, ts_ns: int) -> SubtaskSpan:
        """Return the active SubtaskSpan at timestamp ts_ns."""
        for span in self.subtask_spans:
            if span.start_ts_ns <= ts_ns <= span.end_ts_ns:
                return span
        if self.subtask_spans:
            if ts_ns < self.subtask_spans[0].start_ts_ns:
                return self.subtask_spans[0]
            return self.subtask_spans[-1]
        raise RuntimeError("No subtask spans available in plan")


@dataclass(frozen=True)
class GripperTransition:
    """Detected state transition of the arm gripper."""
    event_type: str  # "close" or "open"
    ts_ns: int
    duration_s: float
    tcp_xyz: np.ndarray  # shape (3,), float32


class TrajectoryAnalyzer:
    """Analyzes arm trajectory and extracts valid pick/place manipulation events."""

    def __init__(
        self,
        tf_replayer: TfReplayer,
        filter_cfg: TeleopFilterConfig,
        landmarks: LandmarksConfig,
    ) -> None:
        self.tf_replayer = tf_replayer
        self.filter_cfg = filter_cfg
        self.landmarks = landmarks
        self.stow_xyz = np.array(landmarks.stow, dtype=np.float32)

    def analyze(
        self,
        joint_samples: Sequence[Tuple[int, np.ndarray]],
        task_str: str,
        task_type: str,
        clear_bin_side: Optional[str] = None,
    ) -> Optional[EpisodeTrajectoryPlan]:
        """Analyze episode joint samples and TF buffer to resolve episode trajectory plan.

        Args:
            joint_samples: Chronologically ordered list of (timestamp_ns, joint_positions_6d).
                joint_positions_6d index 5 is arm_gripper.
            task_str: Canonical task string (e.g. "Move black knight to target").
            task_type: Primitive task type ("move", "clear", "pick", "place").
            clear_bin_side: "left" or "right" for clear tasks.

        Returns:
            EpisodeTrajectoryPlan if valid manipulation events found; None if invalid/corrupt.
        """
        if not joint_samples or len(joint_samples) < 2:
            logger.warning("Episode has insufficient joint state samples (< 2)")
            return None

        # 1. Detect candidate gripper transitions with dwell time verification
        transitions = self._detect_transitions(joint_samples)

        # 2. Filter candidate transitions by Z-height gating (Z <= z_threshold_m)
        valid_transitions = self._filter_by_z_height(transitions)

        # 3. Resolve events and subtask timeline based on task_type
        if task_type == "move":
            return self._resolve_move(valid_transitions, joint_samples, task_str)
        elif task_type == "place":
            return self._resolve_place(valid_transitions, joint_samples, task_str)
        elif task_type == "pick":
            return self._resolve_pick(valid_transitions, joint_samples, task_str)
        elif task_type == "clear":
            return self._resolve_clear(
                valid_transitions, joint_samples, task_str, clear_bin_side, all_transitions=transitions
            )
        else:
            logger.warning("Unsupported task type: '%s'", task_type)
            return None

    def _detect_transitions(
        self,
        joint_samples: Sequence[Tuple[int, np.ndarray]],
    ) -> List[GripperTransition]:
        """Detect open/close transitions that meet the state dwell time threshold."""
        transitions: List[GripperTransition] = []
        n = len(joint_samples)
        close_thresh = self.filter_cfg.gripper_close_threshold
        dwell_ns = int(self.filter_cfg.dwell_time_s * 1e9)

        # Boolean state array: True = closed (<= threshold), False = open (> threshold)
        is_closed_states = [sample[1][5] <= close_thresh for sample in joint_samples]

        i = 1
        while i < n:
            prev_state = is_closed_states[i - 1]
            curr_state = is_closed_states[i]

            if prev_state != curr_state:
                # State transition detected at sample i
                event_type = "close" if curr_state else "open"
                start_ts = joint_samples[i][0]

                # Verify state dwell time (must hold the new state for >= dwell_ns)
                held_until_ts = start_ts
                k = i
                while k < n and is_closed_states[k] == curr_state:
                    held_until_ts = joint_samples[k][0]
                    k += 1

                held_duration_s = (held_until_ts - start_ts) * 1e-9

                # If this is the last state in the episode, allow slightly shorter dwell
                # if it holds until the very end
                is_terminal = (k == n)
                if held_duration_s >= self.filter_cfg.dwell_time_s or (is_terminal and held_duration_s >= 0.3):
                    # Query TCP pose at event start timestamp
                    try:
                        tcp = self.tf_replayer.lookup_tcp_pose(
                            self.filter_cfg.tool_frame,
                            self.filter_cfg.base_frame,
                            start_ts,
                        )
                        transitions.append(
                            GripperTransition(
                                event_type=event_type,
                                ts_ns=start_ts,
                                duration_s=held_duration_s,
                                tcp_xyz=tcp,
                            )
                        )
                    except Exception as exc:
                        logger.debug("Failed TF lookup for transition at %d ns: %s", start_ts, exc)

                i = k
            else:
                i += 1

        return transitions

    def _filter_by_z_height(
        self,
        transitions: List[GripperTransition],
    ) -> List[GripperTransition]:
        """Filter out transitions occurring above z_threshold_m (e.g. at stow or in air)."""
        valid: List[GripperTransition] = []
        for trans in transitions:
            z = float(trans.tcp_xyz[2])
            if z <= self.filter_cfg.z_threshold_m:
                valid.append(trans)
            else:
                logger.debug(
                    "Rejected '%s' transition at ts=%d ns due to Z-height (Z=%.4f m > threshold %.4f m)",
                    trans.event_type,
                    trans.ts_ns,
                    z,
                    self.filter_cfg.z_threshold_m,
                )
        return valid

    def _resolve_move(
        self,
        transitions: List[GripperTransition],
        joint_samples: Sequence[Tuple[int, np.ndarray]],
        task_str: str,
    ) -> Optional[EpisodeTrajectoryPlan]:
        """Resolve MOVE task boundaries: pick at source, place at target, retract to stow."""
        close_events = [t for t in transitions if t.event_type == "close"]
        open_events = [t for t in transitions if t.event_type == "open"]

        if not close_events or not open_events:
            logger.warning(
                "MOVE task requires at least 1 valid close and 1 valid open event. "
                "Found closes=%d, opens=%d",
                len(close_events),
                len(open_events),
            )
            return None

        transit_min_ns = int(self.filter_cfg.transit_min_s * 1e9)

        # Rule: t_pick is the LAST valid close before the first subsequent valid open
        best_pair: Optional[Tuple[GripperTransition, GripperTransition]] = None
        for close_ev in close_events:
            # Candidate open events must occur after close_ev by at least transit_min_s
            candidate_opens = [
                op for op in open_events if (op.ts_ns - close_ev.ts_ns) >= transit_min_ns
            ]
            if candidate_opens:
                # We pick the first valid open event after transit
                best_pair = (close_ev, candidate_opens[0])

        if best_pair is None:
            logger.warning("MOVE task failed temporal ordering (no valid pick-place pair with transit time)")
            return None

        pick_ev, place_ev = best_pair
        t_pick = pick_ev.ts_ns
        t_place = place_ev.ts_ns
        t_start = joint_samples[0][0]
        t_end = joint_samples[-1][0]

        source_xyz = pick_ev.tcp_xyz
        target_xyz = place_ev.tcp_xyz

        # Phase 1: Pick at source (0 -> t_pick)
        # Phase 2: Place at target (t_pick -> t_place)
        # Phase 3: Retract to stow (t_place -> end)
        spans = [
            SubtaskSpan(
                name="Pick piece at source",
                goal_xyz=source_xyz,
                start_ts_ns=t_start,
                end_ts_ns=t_pick,
            ),
            SubtaskSpan(
                name="Place piece to target",
                goal_xyz=target_xyz,
                start_ts_ns=t_pick,
                end_ts_ns=t_place,
            ),
            SubtaskSpan(
                name="Retract arm to stow",
                goal_xyz=self.stow_xyz,
                start_ts_ns=t_place,
                end_ts_ns=t_end,
            ),
        ]

        return EpisodeTrajectoryPlan(
            task_type="move",
            canonical_task=task_str,
            source_xyz=source_xyz,
            target_xyz=target_xyz,
            stow_xyz=self.stow_xyz,
            t_pick_ns=t_pick,
            t_place_ns=t_place,
            subtask_spans=spans,
        )

    def _resolve_place(
        self,
        transitions: List[GripperTransition],
        joint_samples: Sequence[Tuple[int, np.ndarray]],
        task_str: str,
    ) -> Optional[EpisodeTrajectoryPlan]:
        """Resolve PLACE task boundaries: place at target, retract to stow."""
        open_events = [t for t in transitions if t.event_type == "open"]
        if not open_events:
            logger.warning("PLACE task requires at least 1 valid open event at destination")
            return None

        place_ev = open_events[0]
        t_place = place_ev.ts_ns
        t_start = joint_samples[0][0]
        t_end = joint_samples[-1][0]

        target_xyz = place_ev.tcp_xyz
        source_xyz = self.stow_xyz

        spans = [
            SubtaskSpan(
                name="Place piece to target",
                goal_xyz=target_xyz,
                start_ts_ns=t_start,
                end_ts_ns=t_place,
            ),
            SubtaskSpan(
                name="Retract arm to stow",
                goal_xyz=self.stow_xyz,
                start_ts_ns=t_place,
                end_ts_ns=t_end,
            ),
        ]

        return EpisodeTrajectoryPlan(
            task_type="place",
            canonical_task=task_str,
            source_xyz=source_xyz,
            target_xyz=target_xyz,
            stow_xyz=self.stow_xyz,
            t_pick_ns=None,
            t_place_ns=t_place,
            subtask_spans=spans,
        )

    def _resolve_pick(
        self,
        transitions: List[GripperTransition],
        joint_samples: Sequence[Tuple[int, np.ndarray]],
        task_str: str,
    ) -> Optional[EpisodeTrajectoryPlan]:
        """Resolve PICK task boundaries: pick at source, retract to stow."""
        close_events = [t for t in transitions if t.event_type == "close"]
        if not close_events:
            logger.warning("PICK task requires at least 1 valid close event on piece")
            return None

        pick_ev = close_events[-1]  # Last valid grasp before retracting
        t_pick = pick_ev.ts_ns
        t_start = joint_samples[0][0]
        t_end = joint_samples[-1][0]

        source_xyz = pick_ev.tcp_xyz
        target_xyz = self.stow_xyz

        spans = [
            SubtaskSpan(
                name="Pick piece at source",
                goal_xyz=source_xyz,
                start_ts_ns=t_start,
                end_ts_ns=t_pick,
            ),
            SubtaskSpan(
                name="Retract arm to stow",
                goal_xyz=self.stow_xyz,
                start_ts_ns=t_pick,
                end_ts_ns=t_end,
            ),
        ]

        return EpisodeTrajectoryPlan(
            task_type="pick",
            canonical_task=task_str,
            source_xyz=source_xyz,
            target_xyz=target_xyz,
            stow_xyz=self.stow_xyz,
            t_pick_ns=t_pick,
            t_place_ns=None,
            subtask_spans=spans,
        )

    def _resolve_clear(
        self,
        transitions: List[GripperTransition],
        joint_samples: Sequence[Tuple[int, np.ndarray]],
        task_str: str,
        clear_bin_side: Optional[str],
        all_transitions: Optional[List[GripperTransition]] = None,
    ) -> Optional[EpisodeTrajectoryPlan]:
        """Resolve CLEAR task boundaries: pick piece on board, transfer to clear bin, retract."""
        close_events = [t for t in transitions if t.event_type == "close"]
        if not close_events:
            logger.warning("CLEAR task requires at least 1 valid close event on piece")
            return None

        pick_ev = close_events[0]
        t_pick = pick_ev.ts_ns
        t_start = joint_samples[0][0]
        t_end = joint_samples[-1][0]

        source_xyz = pick_ev.tcp_xyz
        bin_coords = (
            self.landmarks.clear_left
            if (clear_bin_side or "").lower() == "left"
            else self.landmarks.clear_right
        )
        target_xyz = np.array(bin_coords, dtype=np.float32)

        # Look for open event after pick for dropping into bin
        search_opens = all_transitions if all_transitions is not None else transitions
        open_near_bin = [
            t for t in search_opens
            if t.event_type == "open" and t.ts_ns > t_pick
            and np.linalg.norm(t.tcp_xyz[:2] - target_xyz[:2]) < 0.15
        ]
        if open_near_bin:
            t_drop = open_near_bin[0].ts_ns
        else:
            open_after_pick = [t for t in search_opens if t.event_type == "open" and t.ts_ns > t_pick]
            t_drop = open_after_pick[0].ts_ns if open_after_pick else int((t_pick + t_end) / 2)

        spans = [
            SubtaskSpan(
                name="Pick piece for clear",
                goal_xyz=source_xyz,
                start_ts_ns=t_start,
                end_ts_ns=t_pick,
            ),
            SubtaskSpan(
                name="Clear piece to bin",
                goal_xyz=target_xyz,
                start_ts_ns=t_pick,
                end_ts_ns=t_drop,
            ),
            SubtaskSpan(
                name="Retract arm to stow",
                goal_xyz=self.stow_xyz,
                start_ts_ns=t_drop,
                end_ts_ns=t_end,
            ),
        ]

        return EpisodeTrajectoryPlan(
            task_type="clear",
            canonical_task=task_str,
            source_xyz=source_xyz,
            target_xyz=target_xyz,
            stow_xyz=self.stow_xyz,
            t_pick_ns=t_pick,
            t_place_ns=t_drop,
            subtask_spans=spans,
        )
