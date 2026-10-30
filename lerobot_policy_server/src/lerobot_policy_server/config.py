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

from dataclasses import dataclass, field
from pathlib import Path


def get_default_config_path() -> Path:
    """Resolve the default YAML configuration file path.

    Checks package-internal bundled asset first, then falls back to repository path.
    """
    bundled_path = Path(__file__).resolve().parent / "configs" / "default.yaml"
    if bundled_path.is_file():
        return bundled_path
    repo_path = Path(__file__).resolve().parents[2] / "config" / "default.yaml"
    return repo_path


@dataclass
class ZmqServerConfig:
    """Configuration parameters for the ZeroMQ policy server."""

    host: str = "0.0.0.0"
    port: int = 8090
    fps: int = 25

    @property
    def environment_dt(self) -> float:
        """Target time delta per step in seconds."""
        return 1.0 / max(1, self.fps)


@dataclass
class RTCServerConfig:
    """Real-Time Chunking (RTC) guidance configuration."""

    enabled: bool = False
    execution_horizon: int = 18
    max_guidance_weight: float = 10.0
    prefix_attention_schedule: str = "linear"  # "linear", "exp", "ones", "zeros"


@dataclass
class InferenceEngineConfig:
    """Configuration parameters for the policy inference pipeline."""

    fps: int = 25
    policy_type: str = "smolvla"
    pretrained_name_or_path: str = (
        "/home/duykhongcay/lerobot_ws/lekiwi_chess_smolvla/checkpoints/010000/pretrained_model"
    )
    actions_per_chunk: int = 30
    device: str = "cuda"
    preload: bool = True
    warmup: bool = True
    rtc: RTCServerConfig = field(default_factory=RTCServerConfig)

    @property
    def environment_dt(self) -> float:
        """Target time delta per step in seconds."""
        return 1.0 / max(1, self.fps)


@dataclass
class PolicyServerConfig:
    """Top-level application configuration schema for Draccus and YAML source of truth."""

    server: ZmqServerConfig = field(default_factory=ZmqServerConfig)
    engine: InferenceEngineConfig = field(default_factory=InferenceEngineConfig)
    log_level: str = "INFO"
