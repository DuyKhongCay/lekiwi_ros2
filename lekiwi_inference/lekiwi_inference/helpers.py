# Copyright 2026 LeKiwi Robotics Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Lightweight LeRobot 0.6.1 async inference data structures and helpers.

Pure Python & NumPy implementation avoiding heavy dependencies (PyTorch, transformers)
on edge platforms like Raspberry Pi 5.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Optional

import numpy as np

# Aggregate function registry - mirrors lerobot.async_inference.configs
AGGREGATE_FUNCTIONS: dict[str, Callable[[np.ndarray, np.ndarray], np.ndarray]] = {
    "weighted_average": lambda old, new: (0.3 * old + 0.7 * new).astype(np.float32),
    "latest_only": lambda old, new: new.astype(np.float32),
    "average": lambda old, new: (0.5 * old + 0.5 * new).astype(np.float32),
    "conservative": lambda old, new: (0.7 * old + 0.3 * new).astype(np.float32),
}


@dataclass
class TimedData:
    """Base data container tracking chronological timestamp and discrete timestep."""

    timestamp: float
    timestep: int

    def get_timestamp(self) -> float:
        return self.timestamp

    def get_timestep(self) -> int:
        return self.timestep


@dataclass
class TimedAction(TimedData):
    """Action payload wrapped with timestep and arrival timestamp."""

    action: np.ndarray

    def get_action(self) -> np.ndarray:
        return self.action


@dataclass
class TimedObservation(TimedData):
    """Observation payload containing images, joint states, and environmental vectors."""

    observation: dict[str, Any]
    must_go: bool = False
    inference_delay: int = 14
    execution_horizon: int = 18
    prev_chunk_left_over: Optional[np.ndarray] = None

    def get_observation(self) -> dict[str, Any]:
        return self.observation


@dataclass
class LatencyTracker:
    """Tracks rolling network and inference latency to estimate discrete delay steps."""

    window_size: int = 8
    history: list[float] = field(default_factory=list)

    def add(self, latency_s: float) -> None:
        """Adds a measured latency sample in seconds."""
        self.history.append(float(latency_s))
        if len(self.history) > self.window_size:
            self.history.pop(0)

    def max(self) -> float:
        """Returns the maximum recorded latency in seconds."""
        return max(self.history) if self.history else 0.0

    def mean(self) -> float:
        """Returns the average recorded latency in seconds."""
        return float(np.mean(self.history)) if self.history else 0.0

    def compute_delay_steps(self, dt: float, default_steps: int = 14) -> int:
        """Computes discrete delay steps using conservative max latency."""
        if not self.history:
            return default_steps
        effective_dt = max(1e-4, float(dt))
        # Round to 6 decimal places to prevent floating-point epsilon (e.g. 0.28/0.04=7.000000000000001) from inflating ceil
        ratio = round(self.max() / effective_dt, 6)
        return max(1, int(np.ceil(ratio)))

    def reset(self) -> None:
        """Flushes recorded history."""
        self.history.clear()


@dataclass
class RemotePolicyConfig:
    """Parameters required to initialize and negotiate remote policy server sessions."""

    policy_type: str = "act"
    pretrained_name_or_path: str = ""
    lerobot_features: dict[str, Any] = field(default_factory=dict)
    actions_per_chunk: int = 40
    device: str = "cuda"
    rename_map: dict[str, str] = field(default_factory=dict)


@dataclass
class FPSTracker:
    """Tracks effective observation throughput against configured target rate."""

    target_fps: float = 25.0
    first_timestamp: Optional[float] = None
    total_obs_count: int = 0

    def calculate_fps_metrics(self, current_timestamp: float) -> dict[str, float]:
        """Calculates rolling and overall average frame rate."""
        self.total_obs_count += 1
        if self.first_timestamp is None:
            self.first_timestamp = current_timestamp

        total_duration = current_timestamp - self.first_timestamp
        avg_fps = (self.total_obs_count - 1) / total_duration if total_duration > 1e-6 else 0.0
        return {"avg_fps": avg_fps, "target_fps": self.target_fps}

    def reset(self) -> None:
        """Resets the timing accumulator."""
        self.first_timestamp = None
        self.total_obs_count = 0


def observations_similar(
    state_a: np.ndarray,
    state_b: np.ndarray,
    atol: float,
) -> bool:
    """Check if two joint-space observation states are within Euclidean distance threshold.

    Calculates ||state_a - state_b||_2 < atol using pure NumPy.
    Used for observation filtering on edge clients to eliminate action looping/stalling.

    Args:
        state_a: Current robot state vector (e.g. 6D arm joint positions).
        state_b: Reference or previously sent robot state vector.
        atol: Euclidean distance threshold in radians.

    Returns:
        True if Euclidean norm is strictly less than atol, False otherwise.
    """
    diff = np.asarray(state_a, dtype=np.float64).ravel() - np.asarray(state_b, dtype=np.float64).ravel()
    return bool(np.linalg.norm(diff) < atol)

