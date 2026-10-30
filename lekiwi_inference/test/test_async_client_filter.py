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

"""Unit tests for Edge Observation Filtering and Bypass Rule in AsyncInferenceClient."""

import time
import numpy as np
import pytest

from lekiwi_inference.async_client import AsyncInferenceClient, ClientConfig
from lekiwi_inference.helpers import TimedAction
from lekiwi_inference.transport.base import PolicyTransport


class MockTransport(PolicyTransport):
    """Minimal mock transport satisfying PolicyTransport protocol."""

    def connect(self) -> bool:
        return True

    def disconnect(self) -> None:
        pass

    def send_handshake(self) -> bool:
        return True

    def send_policy_config(self, config) -> bool:
        return True

    def infer(self, obs):
        return []

    def is_connected(self) -> bool:
        return True

    def ping(self) -> float:
        return 10.0


def test_ready_to_send_strict_signature():
    """Validates that ready_to_send strictly requires current_state with zero backward-compat fallback."""
    cfg = ClientConfig()
    client = AsyncInferenceClient(MockTransport(), cfg)

    # Calling without current_state must raise TypeError
    with pytest.raises(TypeError):
        client.ready_to_send()  # type: ignore[call-arg]


def test_bypass_rule_on_empty_action_queue():
    """When action queue is depleted (|At| == 0), inference must trigger immediately (starvation bypass)."""
    cfg = ClientConfig(actions_per_chunk=40, chunk_size_threshold=0.15, observation_filter_epsilon=0.05)
    client = AsyncInferenceClient(MockTransport(), cfg)
    client.must_go.clear()

    # Pre-populate last sent state
    joint_state = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6], dtype=np.float32)
    client.submit_observation({"observation.state": joint_state})
    assert client.get_last_sent_state() is not None
    # Simulate worker consuming the pending observation
    client._pending_obs_queue.get_nowait()

    # Queue is empty (0 actions) -> Bypass rule triggers even if state is identical
    assert client.get_queue_size() == 0
    assert client.ready_to_send(joint_state) is True


def test_must_go_bypass():
    """When must_go is active (episode init or transition), ready_to_send always evaluates to True."""
    cfg = ClientConfig(actions_per_chunk=40, chunk_size_threshold=0.15, observation_filter_epsilon=0.05)
    client = AsyncInferenceClient(MockTransport(), cfg)

    # By default must_go is set on init
    assert client.must_go.is_set()

    # Even with ample actions in queue, must_go takes priority
    for i in range(20):
        client.action_queue.put(TimedAction(timestep=i, timestamp=time.time(), action=np.zeros(6)))

    dummy_state = np.zeros(6, dtype=np.float32)
    assert client.ready_to_send(dummy_state) is True


def test_queue_threshold_replenishment_guarantee():
    """Validates that ratio <= chunk_size_threshold guarantees sending without similarity stalling."""
    cfg = ClientConfig(actions_per_chunk=40, chunk_size_threshold=0.15, observation_filter_epsilon=0.05)
    client = AsyncInferenceClient(MockTransport(), cfg)
    client.must_go.clear()

    # Fill queue with 10 actions: 10 / 40 = 0.25 > 0.15 -> should NOT send
    for i in range(10):
        client.action_queue.put(TimedAction(timestep=i, timestamp=time.time(), action=np.zeros(6)))

    state1 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
    assert client.ready_to_send(state1) is False

    # Pop 5 actions: 5 / 40 = 0.125 <= 0.15 -> within threshold
    for _ in range(5):
        client.pop_action()
    assert client.get_queue_size() == 5

    # Gating check: ready_to_send returns True to replenish queue
    assert client.ready_to_send(state1) is True

    # Simulate submit_observation
    client.submit_observation({"observation.state": state1})
    np.testing.assert_allclose(client.get_last_sent_state(), state1)
    # Simulate worker consuming the pending observation
    client._pending_obs_queue.get_nowait()

    # Even if stationary, as long as queue is at or below threshold, replenishment is guaranteed
    assert client.ready_to_send(state1) is True


def test_flush_action_queue():
    """Verify flush_action_queue retains exactly keep_steps actions."""
    cfg = ClientConfig()
    client = AsyncInferenceClient(MockTransport(), cfg)

    for i in range(10):
        client.action_queue.put(TimedAction(timestep=i, timestamp=time.time(), action=np.zeros(6)))

    assert client.get_queue_size() == 10
    client.flush_action_queue(keep_steps=2)
    assert client.get_queue_size() == 2


def test_reset_episode_and_transition_clears_last_sent_state():
    """Lifecycle resets must flush cached state so next observation is never falsely filtered."""
    cfg = ClientConfig()
    client = AsyncInferenceClient(MockTransport(), cfg)

    state = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0], dtype=np.float32)
    client.submit_observation({"observation.state": state})
    assert client.get_last_sent_state() is not None

    # reset_episode clears state
    client.reset_episode()
    assert client.get_last_sent_state() is None

    # submit again and test notify_phase_transition
    client.submit_observation({"observation.state": state})
    assert client.get_last_sent_state() is not None

    client.notify_phase_transition()
    assert client.get_last_sent_state() is None
    assert client.must_go.is_set()
