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

"""Unit tests verifying action queue FIFO buffering and temporal blending behavior."""

import numpy as np

from lekiwi_inference.async_client import ActionQueue
from lekiwi_inference.protocol import TimedAction


def test_action_queue_fifo():
    """Verifies that actions are retrieved in strict chronological order."""
    queue = ActionQueue(aggregation_fn_name="latest_only")
    actions = [
        TimedAction(timestamp=1.0, timestep=0, action=np.array([1.0, 2.0], dtype=np.float32)),
        TimedAction(timestamp=1.0, timestep=1, action=np.array([3.0, 4.0], dtype=np.float32)),
    ]

    queue.add_chunk(actions)
    assert queue.size() == 2

    a1 = queue.pop_action()
    assert a1 is not None and np.allclose(a1, [1.0, 2.0])

    a2 = queue.pop_action()
    assert a2 is not None and np.allclose(a2, [3.0, 4.0])

    assert queue.pop_action() is None
    assert queue.size() == 0


def test_action_queue_temporal_smoothing():
    """Verifies exponential moving average blending between existing and incoming action chunks."""
    queue = ActionQueue(aggregation_fn_name="weighted_average")

    # Initial chunk
    chunk1 = [
        TimedAction(timestamp=1.0, timestep=0, action=np.array([10.0], dtype=np.float32)),
        TimedAction(timestamp=1.0, timestep=1, action=np.array([20.0], dtype=np.float32)),
    ]
    queue.add_chunk(chunk1)

    # Overlapping chunk with new predictions
    chunk2 = [
        TimedAction(timestamp=2.0, timestep=0, action=np.array([0.0], dtype=np.float32)),
        TimedAction(timestamp=2.0, timestep=1, action=np.array([100.0], dtype=np.float32)),
    ]
    queue.add_chunk(chunk2)

    # 0.3 * old + 0.7 * new
    # Step 0: 0.3 * 10.0 + 0.7 * 0.0 = 3.0
    # Step 1: 0.3 * 20.0 + 0.7 * 100.0 = 6.0 + 70.0 = 76.0
    a0 = queue.pop_action()
    a1 = queue.pop_action()

    assert a0 is not None and np.allclose(a0, [3.0])
    assert a1 is not None and np.allclose(a1, [76.0])


def test_action_queue_clear():
    """Verifies that clear flushes all buffered actions immediately."""
    queue = ActionQueue()
    queue.add_chunk([TimedAction(timestamp=1.0, timestep=0, action=np.array([1.0], dtype=np.float32))])
    assert queue.size() == 1
    queue.clear()
    assert queue.size() == 0
    assert queue.pop_action() is None

