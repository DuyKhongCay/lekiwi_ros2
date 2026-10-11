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

"""Unit tests validating safety watchdog timeout evaluation and command arbitration."""

import numpy as np

from lekiwi_inference.safety_watchdog import SafetyState, SafetyWatchdog


def test_watchdog_healthy_execution():
    """Verifies that freshly received actions pass through unaffected when healthy."""
    watchdog = SafetyWatchdog(max_age_s=0.2)
    watchdog.notify_action_received(timestamp=100.0)

    cand_act = np.array([0.1, 0.2, 0.3], dtype=np.float32)
    curr_pos = np.array([0.0, 0.0, 0.0], dtype=np.float32)

    cmd, state = watchdog.arbitrate_command(
        candidate_action=cand_act,
        current_joint_positions=curr_pos,
        current_time=100.1,
    )

    assert state == SafetyState.HEALTHY
    assert np.allclose(cmd, cand_act)


def test_watchdog_timeout_holds_position():
    """Verifies that communication timeouts trigger fail-safe hold on current positions."""
    watchdog = SafetyWatchdog(max_age_s=0.2)
    watchdog.notify_action_received(timestamp=100.0)

    cand_act = np.array([0.5, 0.5, 0.5], dtype=np.float32)
    curr_pos = np.array([0.1, 0.2, 0.3], dtype=np.float32)

    # Simulate query after 0.3s (exceeds max_age_s=0.2)
    cmd, state = watchdog.arbitrate_command(
        candidate_action=cand_act,
        current_joint_positions=curr_pos,
        current_time=100.3,
    )

    assert state == SafetyState.STALE_HOLD
    assert np.allclose(cmd, curr_pos)
    assert watchdog.get_timeout_count() == 1


def test_watchdog_estop():
    """Verifies that engaging emergency stop immediately commands holding position."""
    watchdog = SafetyWatchdog(max_age_s=0.2)
    watchdog.notify_action_received(timestamp=100.0)
    watchdog.trigger_estop(True)

    cand_act = np.array([1.0, 1.0, 1.0], dtype=np.float32)
    curr_pos = np.array([0.2, 0.2, 0.2], dtype=np.float32)

    cmd, state = watchdog.arbitrate_command(
        candidate_action=cand_act,
        current_joint_positions=curr_pos,
        current_time=100.05,
    )

    assert state == SafetyState.EMERGENCY_STOP
    assert np.allclose(cmd, curr_pos)

