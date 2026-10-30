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

"""Unit tests for Real-Time Chunking (RTC) action queue management and leftover extraction."""

import time
import numpy as np
import pytest

from lekiwi_inference.async_client import AsyncInferenceClient, ClientConfig
from lekiwi_inference.helpers import TimedAction, TimedObservation
from lekiwi_inference.transport.base import PolicyTransport


class MockTransport(PolicyTransport):
    """Minimal mock transport for RTC unit testing."""

    def __init__(self):
        self.last_sent_obs = None
        self._connected = True

    def connect(self) -> bool:
        return True

    def disconnect(self) -> None:
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected

    def send_handshake(self) -> bool:
        return True

    def send_policy_config(self, config) -> bool:
        return True

    def infer(self, observation: TimedObservation):
        self.last_sent_obs = observation
        return []

    def ping(self) -> float:
        return 20.0


def test_rtc_leftover_extraction_empty_queue():
    """When action_queue is empty, prev_chunk_left_over must be None."""
    cfg = ClientConfig(enable_rtc=True, default_inference_delay=14, rtc_execution_horizon=18)
    transport = MockTransport()
    client = AsyncInferenceClient(transport, cfg)

    obs_payload = {"observation.state": np.zeros(6, dtype=np.float32)}
    assert client.submit_observation(obs_payload) is True

    pending_obs = client._pending_obs_queue.get_nowait()
    assert pending_obs.inference_delay == 14
    assert pending_obs.prev_chunk_left_over is None


def test_rtc_leftover_extraction_with_actions():
    """When action_queue has unconsumed actions, leftover prefix is capped at rtc_execution_horizon."""
    cfg = ClientConfig(enable_rtc=True, default_inference_delay=14, rtc_execution_horizon=18)
    transport = MockTransport()
    client = AsyncInferenceClient(transport, cfg)

    # Populate 25 actions into action queue
    for i in range(25):
        client.action_queue.put(
            TimedAction(
                timestamp=time.time(),
                timestep=i,
                action=np.full(6, float(i), dtype=np.float32),
            )
        )

    obs_payload = {"observation.state": np.zeros(6, dtype=np.float32)}
    assert client.submit_observation(obs_payload) is True

    pending_obs = client._pending_obs_queue.get_nowait()
    assert pending_obs.inference_delay == 14
    assert pending_obs.prev_chunk_left_over is not None
    # Capped at rtc_execution_horizon = 18
    assert pending_obs.prev_chunk_left_over.shape == (18, 6)
    assert np.allclose(pending_obs.prev_chunk_left_over[0], np.zeros(6))
    assert np.allclose(pending_obs.prev_chunk_left_over[17], np.full(6, 17.0))


def test_rtc_merge_actions_replaces_queue_and_discards_delay():
    """RTC queue merge must discard clamped_delay actions and replace the entire queue."""
    cfg = ClientConfig(enable_rtc=True, default_inference_delay=14, rtc_execution_horizon=18)
    transport = MockTransport()
    client = AsyncInferenceClient(transport, cfg)

    # Old stale actions in queue
    for i in range(5):
        client.action_queue.put(
            TimedAction(timestamp=time.time(), timestep=i, action=np.zeros(6, dtype=np.float32))
        )
    assert client.get_queue_size() == 5

    # New incoming chunk of 30 actions
    new_actions = [
        TimedAction(
            timestamp=100.0 + i * 0.04,
            timestep=i,
            action=np.full(6, float(i), dtype=np.float32),
        )
        for i in range(30)
    ]

    # Real delay is 14 steps
    client._merge_rtc_actions(new_actions, real_delay=14)

    # 30 - 14 = 16 actions remaining in queue
    assert client.get_queue_size() == 16
    popped = client.pop_action()
    assert popped is not None
    # First action should be step index 14
    assert np.allclose(popped, np.full(6, 14.0))


def test_rtc_merge_actions_clamped_delay_overflow():
    """When real_delay >= len(timed_actions), all actions are discarded without error."""
    cfg = ClientConfig(enable_rtc=True)
    transport = MockTransport()
    client = AsyncInferenceClient(transport, cfg)

    new_actions = [
        TimedAction(timestamp=100.0, timestep=0, action=np.ones(6, dtype=np.float32))
    ]

    client._merge_rtc_actions(new_actions, real_delay=5)
    assert client.get_queue_size() == 0
    assert client.pop_action() is None

