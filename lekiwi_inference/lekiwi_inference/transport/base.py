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

"""Abstract interface defining required network primitives for policy communication."""

from abc import ABC, abstractmethod
from typing import Optional

from lekiwi_inference.helpers import RemotePolicyConfig, TimedAction, TimedObservation


class PolicyTransport(ABC):
    """Establishes an implementation-agnostic contract for offloading neural policy inference."""

    @abstractmethod
    def connect(self) -> bool:
        """Initializes socket resources and binds or connects to remote policy servers."""
        pass

    @abstractmethod
    def disconnect(self) -> None:
        """Tears down network connections and cleans up active socket contexts gracefully."""
        pass

    @abstractmethod
    def is_connected(self) -> bool:
        """Queries transport readiness before attempting synchronous or asynchronous requests."""
        pass

    @abstractmethod
    def send_handshake(self) -> bool:
        """Verifies protocol compatibility and resets remote server session state."""
        pass

    @abstractmethod
    def send_policy_config(self, config: RemotePolicyConfig) -> bool:
        """Instructs the remote server to load target model weights and setup inference horizon."""
        pass

    @abstractmethod
    def infer(self, observation: TimedObservation) -> Optional[list[TimedAction]]:
        """Transmits observation frame (with RTC prefix and delay) to policy server and retrieves action chunk."""
        pass

    @abstractmethod
    def ping(self) -> float:
        """Measures round-trip latency to detect network degradation or Wi-Fi channel congestion."""
        pass

