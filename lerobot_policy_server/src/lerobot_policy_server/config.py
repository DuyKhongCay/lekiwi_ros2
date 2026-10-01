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

"""Configuration schemas for ZeroMQ Policy Server and Inference Engine."""

from __future__ import annotations

import os
from dataclasses import dataclass


@dataclass
class ZmqServerConfig:
    """Configuration parameters for the ZeroMQ policy server."""

    host: str = "0.0.0.0"
    port: int = 8090
    fps: int = 30
    inference_latency: float = 0.033
    obs_queue_timeout: float = 2.0
    device: str = "cuda"

    @property
    def environment_dt(self) -> float:
        """Target time delta per step in seconds."""
        return 1.0 / max(1, self.fps)

    @classmethod
    def from_env(cls) -> ZmqServerConfig:
        """Create configuration with environment variable overrides."""
        return cls(
            host=os.getenv("POLICY_SERVER_HOST", "0.0.0.0"),
            port=int(os.getenv("POLICY_SERVER_PORT", "8090")),
            fps=int(os.getenv("POLICY_SERVER_FPS", "30")),
            inference_latency=float(os.getenv("POLICY_SERVER_INFERENCE_LATENCY", "0.033")),
            obs_queue_timeout=float(os.getenv("POLICY_SERVER_OBS_TIMEOUT", "2.0")),
            device=os.getenv("POLICY_SERVER_DEVICE", "cuda"),
        )


@dataclass
class InferenceEngineConfig:
    """Configuration parameters for the policy inference pipeline."""

    fps: int = 30
    inference_latency: float = 0.033
    obs_queue_timeout: float = 2.0
    device: str = "cuda"

    @property
    def environment_dt(self) -> float:
        """Target time delta per step in seconds."""
        return 1.0 / max(1, self.fps)
