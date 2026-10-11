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

"""Safety watchdog tracking freshness and arbitrating safe motor positions upon network loss."""

from enum import Enum
import time
from typing import Optional

import numpy as np


class SafetyState(Enum):
    """Represents operational safety health states of the inference pipeline."""

    HEALTHY = "HEALTHY"
    STALE_HOLD = "STALE_HOLD"
    EMERGENCY_STOP = "EMERGENCY_STOP"


class SafetyWatchdog:
    """Supervises communication timeouts and enforces fail-safe position hold semantics."""

    def __init__(self, max_age_s: float = 0.25) -> None:
        """Initializes timeout threshold and initializes baseline state tracking."""
        self.max_age_s = max_age_s
        self._state = SafetyState.HEALTHY
        self._last_action_time = 0.0
        self._estop_active = False
        self._timeout_count = 0

    def notify_action_received(self, timestamp: Optional[float] = None) -> None:
        """Registers arrival of fresh action chunks to refresh the watchdog timer."""
        self._last_action_time = timestamp if timestamp is not None else time.time()
        if not self._estop_active:
            self._state = SafetyState.HEALTHY

    def trigger_estop(self, active: bool = True) -> None:
        """Engages or releases emergency stop to instantly inhibit robot motion."""
        self._estop_active = active
        self._state = SafetyState.EMERGENCY_STOP if active else SafetyState.HEALTHY

    def is_healthy(self, current_time: Optional[float] = None) -> bool:
        """Determines whether actions are within the permissible latency deadline."""
        if self._estop_active:
            return False
        now = current_time if current_time is not None else time.time()
        age = now - self._last_action_time
        return age <= self.max_age_s

    def evaluate_state(self, current_time: Optional[float] = None) -> SafetyState:
        """Evaluates and transitions the internal safety state machine."""
        if self._estop_active:
            self._state = SafetyState.EMERGENCY_STOP
            return self._state

        now = current_time if current_time is not None else time.time()
        if now - self._last_action_time > self.max_age_s:
            if self._state == SafetyState.HEALTHY:
                self._timeout_count += 1
            self._state = SafetyState.STALE_HOLD
        else:
            self._state = SafetyState.HEALTHY

        return self._state

    def arbitrate_command(
        self,
        candidate_action: Optional[np.ndarray],
        current_joint_positions: np.ndarray,
        current_time: Optional[float] = None,
    ) -> tuple[np.ndarray, SafetyState]:
        """Arbitrates between executing candidate actions or holding joint positions safely."""
        state = self.evaluate_state(current_time)
        if state == SafetyState.HEALTHY and candidate_action is not None:
            return candidate_action, state

        return current_joint_positions.copy(), state

    def get_action_age(self, current_time: Optional[float] = None) -> float:
        """Returns elapsed time in seconds since the last valid action was received."""
        now = current_time if current_time is not None else time.time()
        return max(0.0, now - self._last_action_time)

    def get_timeout_count(self) -> int:
        """Returns cumulative occurrences of network latency dropouts."""
        return self._timeout_count

