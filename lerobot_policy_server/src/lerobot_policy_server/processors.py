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

"""Custom LeRobot processor steps for LeKiwi chess tasks."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from lerobot.processor.converters import EnvTransition
from lerobot.processor.pipeline import ProcessorStep, ProcessorStepRegistry


@ProcessorStepRegistry.register("chess_goal_noise")
@dataclass
class ChessGoalNoiseStep(ProcessorStep):
    """Runtime inference step for chess_goal_noise data augmentation.

    During training on chess tasks, random noise was injected into target goal
    and end-effector coordinates to improve spatial policy robustness.
    During runtime evaluation/inference, this step acts strictly as a transparent
    identity pass-through (no noise added).
    """

    goal_noise_std: float = 0.008
    ee_noise_std: float = 0.003
    max_noise_clip: float = 0.022
    noise_prob: float = 0.8
    enabled: bool = False

    def __call__(self, transition: EnvTransition) -> EnvTransition:
        """Pass through transition without modifying observation or action tensors."""
        return transition

    def transform_features(self, features: dict[Any, Any]) -> dict[Any, Any]:
        """Return pipeline features unmodified."""
        return features

    def get_config(self) -> dict[str, Any]:
        """Serialize configuration parameters."""
        return {
            "goal_noise_std": self.goal_noise_std,
            "ee_noise_std": self.ee_noise_std,
            "max_noise_clip": self.max_noise_clip,
            "noise_prob": self.noise_prob,
            "enabled": self.enabled,
        }
