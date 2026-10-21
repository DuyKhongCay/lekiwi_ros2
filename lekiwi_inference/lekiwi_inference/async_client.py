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

"""Thread-safe asynchronous background client driving LeRobot 0.6.1 policy transactions."""

from __future__ import annotations

from dataclasses import dataclass, field
import logging
from queue import Empty, Queue
import threading
import time
from typing import Any, Optional

import numpy as np

from lekiwi_inference.helpers import (
    AGGREGATE_FUNCTIONS,
    FPSTracker,
    RemotePolicyConfig,
    TimedAction,
    TimedObservation,
)
from lekiwi_inference.transport.base import PolicyTransport

log = logging.getLogger(__name__)


@dataclass
class ClientConfig:
    """Configures connection parameters and inference thresholds for the client."""

    server_address: str = "127.0.0.1:8090"
    policy_type: str = "act"
    repo_id: str = "DuyKhongCay/act_lekiwi_chess"
    fps: float = 25.0
    actions_per_chunk: int = 40
    chunk_size_threshold: float = 0.5
    aggregation_fn: str = "weighted_average"
    device: str = "cuda"
    features: dict[str, Any] = field(default_factory=dict)
    rename_map: dict[str, str] = field(default_factory=dict)


class AsyncInferenceClient:
    """Coordinates observation offloading, thread synchronization, and action queue ingestion."""

    def __init__(
        self,
        transport: PolicyTransport,
        config: ClientConfig,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        """Initializes network transport, action buffer, and worker synchronization primitives."""
        self.transport = transport
        self.config = config
        self._log = logger or log

        self.latest_action_lock = threading.Lock()
        self.latest_action = -1
        self.action_chunk_size = max(1, config.actions_per_chunk)

        self.action_queue: Queue[TimedAction] = Queue()
        self.action_queue_lock = threading.Lock()

        # Observation queue for asynchronous inference worker
        self._pending_obs_queue: Queue[TimedObservation] = Queue(maxsize=1)
        self._shutdown_event = threading.Event()
        self._worker_thread: Optional[threading.Thread] = None

        self.fps_tracker = FPSTracker(target_fps=config.fps)
        self.must_go = threading.Event()
        self.must_go.set()

        fn_name = config.aggregation_fn
        if fn_name not in AGGREGATE_FUNCTIONS:
            raise ValueError(
                f"Unknown aggregate function '{fn_name}'. Available: {list(AGGREGATE_FUNCTIONS.keys())}"
            )
        self._aggregate_fn = AGGREGATE_FUNCTIONS[fn_name]

        # Telemetry counters
        self._obs_sent_count = 0
        self._obs_dropped_count = 0
        self._actions_executed_count = 0
        self._chunks_received_count = 0
        self._inference_in_flight = False
        self._last_action_timestamp = 0.0

    def start(self) -> bool:
        """Connects transport, negotiates policy loading, and spawns the inference thread."""
        if not self.transport.connect():
            self._log.error("Failed to connect transport to policy server.")
            return False

        if hasattr(self.transport, "send_handshake"):
            if not self.transport.send_handshake():
                self._log.error("Failed handshake with remote policy server.")
                return False
        elif hasattr(self.transport, "handshake"):
            if not self.transport.handshake():
                self._log.error("Failed handshake with remote policy server.")
                return False

        policy_cfg = RemotePolicyConfig(
            policy_type=self.config.policy_type,
            pretrained_name_or_path=self.config.repo_id,
            lerobot_features=self.config.features,
            actions_per_chunk=self.config.actions_per_chunk,
            device=self.config.device,
            rename_map=self.config.rename_map,
        )

        if not self.transport.send_policy_config(policy_cfg):
            self._log.error("Failed to configure remote policy on server.")
            return False

        self._shutdown_event.clear()
        self._worker_thread = threading.Thread(
            target=self._inference_loop, name="InferenceWorker", daemon=True
        )
        self._worker_thread.start()
        self._log.info("AsyncInferenceClient worker thread successfully started.")
        return True

    def stop(self) -> None:
        """Signals worker termination, clears pending queues, and disconnects transport."""
        self._shutdown_event.set()
        if self._worker_thread and self._worker_thread.is_alive():
            self._worker_thread.join(timeout=1.0)

        with self.action_queue_lock:
            self.action_queue = Queue()

        self.transport.disconnect()
        self._log.info("AsyncInferenceClient cleanly stopped.")

    def reset_episode(self) -> None:
        """Flushes action and observation queues and resets timestep tracking for a new episode."""
        with self.action_queue_lock:
            self.action_queue = Queue()
        with self.latest_action_lock:
            self.latest_action = -1
        while not self._pending_obs_queue.empty():
            try:
                self._pending_obs_queue.get_nowait()
            except Empty:
                break
        self._inference_in_flight = False
        self.must_go.set()
        self.fps_tracker.reset()
        self._log.info("Episode reset: queues cleared, timestep tracking re-initialized.")

    def ready_to_send(self) -> bool:
        """Return True when the action queue is low enough to request more actions from server."""
        if self._inference_in_flight:
            return False
        if not self._pending_obs_queue.empty():
            return False

        with self.action_queue_lock:
            qsize = self.action_queue.qsize()

        if qsize == 0:
            return True

        ratio = qsize / max(1, self.action_chunk_size)
        return ratio <= self.config.chunk_size_threshold

    def submit_observation(self, raw_obs: dict[str, Any], timestamp: Optional[float] = None) -> bool:
        """Schedules latest observation snapshot for remote evaluation, discarding stale frames."""
        ts = timestamp if timestamp is not None else time.time()
        with self.latest_action_lock:
            latest = max(self.latest_action, 0)

        with self.action_queue_lock:
            qsize = self.action_queue.qsize()

        must_go_val = self.must_go.is_set() and (qsize == 0)
        if must_go_val:
            self.must_go.clear()

        obs = TimedObservation(
            timestamp=ts,
            timestep=latest,
            observation=raw_obs,
            must_go=must_go_val,
        )

        if self._pending_obs_queue.full():
            try:
                self._pending_obs_queue.get_nowait()
                self._obs_dropped_count += 1
            except Empty:
                pass

        try:
            self._pending_obs_queue.put_nowait(obs)
            return True
        except Exception:
            return False

    def pop_action(self) -> Optional[np.ndarray]:
        """Pops next action step and converts to flat 1D numpy array."""
        with self.action_queue_lock:
            if self.action_queue.empty():
                return None
            timed_action = self.action_queue.get_nowait()

        action_val = timed_action.get_action()
        if hasattr(action_val, "detach"):
            action_np = action_val.detach().cpu().numpy().astype(np.float64).reshape(-1)
        elif isinstance(action_val, np.ndarray):
            action_np = action_val.astype(np.float64).reshape(-1)
        else:
            action_np = np.asarray(action_val, dtype=np.float64).reshape(-1)

        with self.latest_action_lock:
            self.latest_action = timed_action.get_timestep()

        self._actions_executed_count += 1
        return action_np

    def actions_available(self) -> bool:
        """Reports whether at least one action is buffered and ready for execution."""
        with self.action_queue_lock:
            return not self.action_queue.empty()

    def get_queue_size(self) -> int:
        """Reports current number of queued actions."""
        with self.action_queue_lock:
            return self.action_queue.qsize()

    def get_last_action_timestamp(self) -> float:
        """Returns arrival timestamp of the most recent action chunk."""
        return self._last_action_timestamp

    def _inference_loop(self) -> None:
        """Continuous background worker loop processing pending frames over ZeroMQ."""
        self._log.info("Inference worker thread started.")
        while not self._shutdown_event.is_set():
            try:
                obs = self._pending_obs_queue.get(timeout=0.05)
            except Empty:
                continue

            self._inference_in_flight = True
            try:
                self._obs_sent_count += 1
                actions = self.transport.infer(obs)
                if actions:
                    self._last_action_timestamp = time.time()
                    self._aggregate_actions(actions)
                    self.fps_tracker.calculate_fps_metrics(self._last_action_timestamp)
                else:
                    self._log.warning("Inference request returned empty action chunk.")
            except Exception as exc:
                self._log.error("Error during inference transaction: %s", exc)
                time.sleep(0.04)
            finally:
                self._inference_in_flight = False

    def _aggregate_actions(self, timed_actions: list[TimedAction]) -> None:
        """Merge incoming actions into the queue — single-pass, LeRobot-style."""
        self._chunks_received_count += 1
        chunk_len = len(timed_actions)
        self.action_chunk_size = max(self.action_chunk_size, chunk_len)

        with self.latest_action_lock:
            latest = self.latest_action

        with self.action_queue_lock:
            old_by_ts: dict[int, np.ndarray] = {}
            while not self.action_queue.empty():
                try:
                    item = self.action_queue.get_nowait()
                    old_by_ts[item.get_timestep()] = item.get_action()
                except Empty:
                    break

            new_queue: Queue[TimedAction] = Queue()
            for action in timed_actions:
                ts = action.get_timestep()
                if ts <= latest:
                    continue

                act_data = action.get_action()
                if not isinstance(act_data, np.ndarray):
                    act_data = np.asarray(act_data, dtype=np.float32)

                if ts in old_by_ts:
                    merged = self._aggregate_fn(old_by_ts[ts], act_data)
                    new_queue.put(
                        TimedAction(
                            timestamp=action.get_timestamp(),
                            timestep=ts,
                            action=merged,
                        )
                    )
                else:
                    new_queue.put(
                        TimedAction(
                            timestamp=action.get_timestamp(),
                            timestep=ts,
                            action=act_data,
                        )
                    )

            self.action_queue = new_queue
