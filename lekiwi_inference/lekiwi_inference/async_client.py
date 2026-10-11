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

"""Thread-safe action queue and asynchronous background worker driving inference transactions."""

from collections import deque
from dataclasses import dataclass, field
import logging
import queue
import threading
import time
from typing import Callable, Optional

import numpy as np

from lekiwi_inference.protocol import RemotePolicyConfig, TimedAction, TimedObservation
from lekiwi_inference.transport.base import PolicyTransport

logger = logging.getLogger(__name__)

AGGREGATION_FUNCTIONS: dict[str, Callable[[np.ndarray, np.ndarray], np.ndarray]] = {
    "weighted_average": lambda old, new: (0.3 * old + 0.7 * new).astype(np.float32),
    "conservative": lambda old, new: (0.7 * old + 0.3 * new).astype(np.float32),
    "latest_only": lambda old, new: new.astype(np.float32),
    "average": lambda old, new: (0.5 * old + 0.5 * new).astype(np.float32),
}


class FPSTracker:
    """Computes moving average inference throughput and timing jitter over rolling horizons."""

    def __init__(self, window_size: int = 30) -> None:
        """Initializes timing history window."""
        self.window_size = window_size
        self._timestamps: deque[float] = deque(maxlen=window_size)

    def record(self, timestamp: Optional[float] = None) -> float:
        """Records frame timestamp and calculates current rolling frequency in Hz."""
        ts = timestamp if timestamp is not None else time.time()
        self._timestamps.append(ts)
        if len(self._timestamps) < 2:
            return 0.0
        elapsed = self._timestamps[-1] - self._timestamps[0]
        if elapsed <= 0.0:
            return 0.0
        return float((len(self._timestamps) - 1) / elapsed)


class ActionQueue:
    """Maintains a thread-safe sliding buffer of future actions with temporal blending."""

    def __init__(self, aggregation_fn_name: str = "weighted_average") -> None:
        """Initializes action queue lock and resolves target temporal blending function."""
        self._queue: deque[np.ndarray] = deque()
        self._lock = threading.Lock()
        self.aggregation_fn = AGGREGATION_FUNCTIONS.get(
            aggregation_fn_name, AGGREGATION_FUNCTIONS["weighted_average"]
        )

    def add_chunk(self, actions: list[TimedAction]) -> None:
        """Merges predicted action horizon chunk into the queue with smooth boundary blending."""
        if not actions:
            return

        with self._lock:
            existing_count = len(self._queue)
            for idx, timed_action in enumerate(actions):
                act = timed_action.get_action()
                if not isinstance(act, np.ndarray):
                    act = np.asarray(act, dtype=np.float32)

                if idx < existing_count:
                    old_act = self._queue[idx]
                    self._queue[idx] = self.aggregation_fn(old_act, act)
                else:
                    self._queue.append(act.copy())

    def pop_action(self) -> Optional[np.ndarray]:
        """Pops the immediate next action step for real-time motor controller publishing."""
        with self._lock:
            if not self._queue:
                return None
            return self._queue.popleft()

    def clear(self) -> None:
        """Flushes buffered actions upon safety e-stop or policy mode transitions."""
        with self._lock:
            self._queue.clear()

    def size(self) -> int:
        """Reports the number of remaining actions in the execution buffer."""
        with self._lock:
            return len(self._queue)


@dataclass
class ClientConfig:
    """Configures connection parameters and inference thresholds for the client."""

    server_address: str = "127.0.0.1:8090"
    policy_type: str = "act"
    repo_id: str = "legalaspro/act_so101_pnp_microsanity_20_50hz_v0"
    fps: float = 50.0
    actions_per_chunk: int = 100
    chunk_size_threshold: float = 0.5
    aggregation_fn: str = "weighted_average"
    features: dict = field(default_factory=dict)


class AsyncInferenceClient:
    """Coordinates observation offloading, thread synchronization, and action queue ingestion."""

    def __init__(
        self,
        transport: PolicyTransport,
        config: ClientConfig,
    ) -> None:
        """Initializes network transport, action buffer, and worker synchronization primitives."""
        self.transport = transport
        self.config = config
        self.action_queue = ActionQueue(config.aggregation_fn)
        self.fps_tracker = FPSTracker(window_size=30)

        self._obs_queue: queue.Queue[TimedObservation] = queue.Queue(maxsize=1)
        self._stop_event = threading.Event()
        self._worker_thread: Optional[threading.Thread] = None
        self._timestep = 0
        self._last_action_timestamp = 0.0

    def start(self) -> bool:
        """Connects transport, negotiates policy loading, and spawns the inference thread."""
        if not self.transport.connect():
            logger.error("Failed to connect transport.")
            return False

        if not self.transport.send_handshake():
            logger.error("Failed handshake with remote policy server.")
            return False

        policy_cfg = RemotePolicyConfig(
            repo_id=self.config.repo_id,
            policy_type=self.config.policy_type,
            lerobot_features=self.config.features,
            fps=self.config.fps,
            actions_per_chunk=self.config.actions_per_chunk,
            chunk_size_threshold=self.config.chunk_size_threshold,
        )
        if not self.transport.send_policy_config(policy_cfg):
            logger.error("Failed to configure remote policy on server.")
            return False

        self._stop_event.clear()
        self._worker_thread = threading.Thread(
            target=self._inference_loop, name="InferenceWorker", daemon=True
        )
        self._worker_thread.start()
        logger.info("AsyncInferenceClient worker thread successfully started.")
        return True

    def stop(self) -> None:
        """Signals worker termination, clears pending queues, and disconnects transport."""
        self._stop_event.set()
        if self._worker_thread and self._worker_thread.is_alive():
            self._worker_thread.join(timeout=1.0)
        self.action_queue.clear()
        self.transport.disconnect()
        logger.info("AsyncInferenceClient cleanly stopped.")

    def submit_observation(self, observation: dict, timestamp: Optional[float] = None) -> bool:
        """Schedules latest observation snapshot for remote evaluation, discarding stale frames."""
        ts = timestamp if timestamp is not None else time.time()
        self._timestep += 1
        timed_obs = TimedObservation(
            timestamp=ts,
            timestep=self._timestep,
            observation=observation,
        )

        try:
            # Drop older pending frame if worker is currently transmitting
            if self._obs_queue.full():
                try:
                    self._obs_queue.get_nowait()
                except queue.Empty:
                    pass
            self._obs_queue.put_nowait(timed_obs)
            return True
        except queue.Full:
            return False

    def get_next_action(self) -> Optional[np.ndarray]:
        """Retrieves next continuous joint angle action for motor execution."""
        return self.action_queue.pop_action()

    def get_last_action_timestamp(self) -> float:
        """Returns arrival timestamp of the most recent action chunk."""
        return self._last_action_timestamp

    def _inference_loop(self) -> None:
        """Continuous background worker loop processing pending frames over ZeroMQ."""
        while not self._stop_event.is_shutdown() if hasattr(self._stop_event, "is_shutdown") else not self._stop_event.is_set():
            try:
                obs = self._obs_queue.get(timeout=0.05)
            except queue.Empty:
                continue

            try:
                actions = self.transport.infer(obs)
                if actions:
                    self._last_action_timestamp = time.time()
                    self.action_queue.add_chunk(actions)
                    self.fps_tracker.record(self._last_action_timestamp)
                else:
                    logger.warning("Inference request returned empty action chunk.")
            except Exception as exc:
                logger.error("Error during inference transaction: %s", exc)

