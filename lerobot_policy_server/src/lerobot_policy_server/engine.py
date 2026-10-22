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

"""Inference engine for executing LeRobot v0.6.1 policies.

Manages policy lifecycle, tensor preprocessing, JPEG image decoding,
action chunk prediction, and performance profiling.
"""

from __future__ import annotations

import gc
import logging
import threading
import time
from typing import Any

import cv2
import numpy as np
import torch

from lerobot.async_inference.constants import SUPPORTED_POLICIES
from lerobot.async_inference.helpers import (
    FPSTracker,
    Observation,
    RemotePolicyConfig,
    TimedAction,
    TimedObservation,
    is_image_key,
)
from lerobot.policies.factory import get_policy_class, make_pre_post_processors
from lerobot.processor import PolicyProcessorPipeline
from lerobot.utils.constants import OBS_ENV_STATE, OBS_STATE
from lerobot_policy_server.config import InferenceEngineConfig

logger = logging.getLogger("lerobot_policy_server.engine")


def _decode_jpeg_to_rgb(jpeg_bytes: bytes) -> np.ndarray:
    """Decode JPEG compressed bytes to RGB uint8 numpy array (H, W, 3)."""
    buf = np.frombuffer(jpeg_bytes, dtype=np.uint8)
    bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError("cv2.imdecode failed: corrupted or invalid image bytes.")
    return np.ascontiguousarray(bgr[:, :, ::-1], dtype=np.uint8)  # BGR -> RGB


def _raw_observation_to_observation(raw_obs: dict[str, Any]) -> Observation:
    """Convert raw robot observation dict to policy-ready PyTorch tensors.

    - Decodes JPEG bytes to RGB uint8 (H, W, 3).
    - Image keys (observation.images.*): converts to float32 [0.0, 1.0] (1, C, H, W)
      preserving native aspect ratio without pre-resizing.
    - State keys (observation.state 6D, observation.environment_state 10D):
      converts to float32 (1, D).
    - Task key: preserved for batch processor / tokenizer.
    """
    obs_tensors: Observation = {}

    for key, value in raw_obs.items():
        if isinstance(value, (bytes, bytearray)):
            value = _decode_jpeg_to_rgb(bytes(value))

        if is_image_key(key):
            if isinstance(value, np.ndarray):
                # (H, W, C) uint8 -> (1, C, H, W) float32 [0, 1]
                t = torch.from_numpy(value).permute(2, 0, 1).to(dtype=torch.float32).div_(255.0)
                obs_tensors[key] = t.contiguous().unsqueeze(0)
            elif isinstance(value, torch.Tensor):
                t = value
                if t.ndim == 3:
                    if t.shape[2] == 3:  # (H, W, C)
                        t = t.permute(2, 0, 1)
                    t = t.unsqueeze(0)
                if t.dtype == torch.uint8:
                    t = t.to(dtype=torch.float32).div_(255.0)
                obs_tensors[key] = t.contiguous()
        elif key in (OBS_STATE, OBS_ENV_STATE):
            if isinstance(value, np.ndarray):
                arr = value if value.flags.writeable else value.copy()
                t = torch.from_numpy(arr).to(dtype=torch.float32)
            elif isinstance(value, torch.Tensor):
                t = value.to(dtype=torch.float32)
            else:
                t = torch.tensor(value, dtype=torch.float32)
            if t.ndim == 1:
                t = t.unsqueeze(0)
            obs_tensors[key] = t
        elif key == "task":
            obs_tensors[key] = value
        else:
            if isinstance(value, np.ndarray):
                arr = value if value.flags.writeable else value.copy()
                t = torch.from_numpy(arr).to(dtype=torch.float32)
                if t.ndim == 1:
                    t = t.unsqueeze(0)
                obs_tensors[key] = t
            elif isinstance(value, (int, float)):
                obs_tensors[key] = torch.tensor([value], dtype=torch.float32).unsqueeze(0)
            else:
                obs_tensors[key] = value

    return obs_tensors


class InferenceEngine:
    """Pure LeRobot v0.6.1 inference pipeline executing neural policies."""

    def __init__(self, config: InferenceEngineConfig | None = None) -> None:
        self.config = config or InferenceEngineConfig()
        self.running: bool = True
        self.policy: Any = None
        self.policy_config: RemotePolicyConfig | None = None
        self.preprocessor: PolicyProcessorPipeline | None = None
        self.postprocessor: PolicyProcessorPipeline | None = None
        self.device: str = "cpu"
        self.actions_per_chunk: int = 40
        self.fps_tracker = FPSTracker(target_fps=float(self.config.fps))
        self._lock = threading.RLock()

    @staticmethod
    def _resolve_device(requested_device: str) -> str:
        """Resolve requested device with fallback to CPU if CUDA is unavailable."""
        if requested_device == "cuda" and not torch.cuda.is_available():
            logger.warning("CUDA requested but not available. Falling back to CPU.")
            return "cpu"
        return requested_device

    def load_policy(self, config: RemotePolicyConfig) -> None:
        """Load and warm up a LeRobot policy based on RemotePolicyConfig."""
        with self._lock:
            if config.policy_type not in SUPPORTED_POLICIES:
                raise ValueError(
                    f"Unsupported policy type: '{config.policy_type}'. "
                    f"Supported policies: {SUPPORTED_POLICIES}"
                )

            self.clear_session()
            self.policy_config = config
            self.device = self._resolve_device(config.device)
            self.actions_per_chunk = config.actions_per_chunk

            logger.info(
                "Loading policy '%s' (type: %s) on %s...",
                config.pretrained_name_or_path,
                config.policy_type,
                self.device,
            )

            policy_class = get_policy_class(config.policy_type)
            start_load = time.perf_counter()
            self.policy = policy_class.from_pretrained(config.pretrained_name_or_path)
            self.policy.to(self.device)
            self.policy.eval()

            device_override = {"device": self.device}
            preprocessor_overrides: dict[str, Any] = {"device_processor": device_override}
            if config.rename_map:
                preprocessor_overrides["rename_observations_processor"] = {
                    "rename_map": config.rename_map
                }

            self.preprocessor, self.postprocessor = make_pre_post_processors(
                self.policy.config,
                pretrained_path=config.pretrained_name_or_path,
                preprocessor_overrides=preprocessor_overrides,
                postprocessor_overrides={"device_processor": {"device": "cpu"}},
            )

            elapsed = time.perf_counter() - start_load
            logger.info("Policy successfully loaded in %.2fs on %s", elapsed, self.device)

    def clear_session(self) -> None:
        """Release loaded policy and reclaim GPU memory."""
        with self._lock:
            if self.policy is not None:
                del self.policy
                self.policy = None
            self.policy_config = None
            self.preprocessor = None
            self.postprocessor = None

            gc.collect()
            if torch.cuda.is_available():
                torch.cuda.empty_cache()
            logger.info("Policy session cleared.")

    def stop(self) -> None:
        """Stop engine and clean up resources."""
        self.running = False
        self.clear_session()

    def _get_action_chunk(self, observation: dict[str, torch.Tensor]) -> torch.Tensor:
        """Get an action chunk from the policy sliced to actions_per_chunk."""
        chunk = self.policy.predict_action_chunk(observation)
        if chunk.ndim != 3:
            chunk = chunk.unsqueeze(0)  # (1, chunk_size, action_dim)
        return chunk[:, : self.actions_per_chunk, :]

    def predict_action_chunk(self, obs_t: TimedObservation) -> list[TimedAction]:
        """Generate a chunk of timed action predictions from a TimedObservation.

        Pipeline:
        1. Decode JPEG & convert raw observation to LeRobot Observation tensors.
        2. Apply preprocessor (normalization, batching, device placement).
        3. Run policy.predict_action_chunk() and slice chunk horizon.
        4. Apply postprocessor step-by-step for each timestep in the chunk.
        5. Package into list[TimedAction] with environment_dt timestamps.
        """
        with self._lock:
            if getattr(obs_t, "must_go", False):
                self.fps_tracker.reset()

            if self.policy is None or self.preprocessor is None or self.postprocessor is None:
                raise RuntimeError("Policy not loaded. Send policy_config request first.")

            # 1. Prepare observation
            observation: Observation = _raw_observation_to_observation(obs_t.get_observation())

            # 2. Apply preprocessor
            with torch.inference_mode():
                observation = self.preprocessor(observation)

                # 3. Get action chunk
                action_tensor = self._get_action_chunk(observation)

                # 4. Apply postprocessor per action step in chunk
                _, chunk_size, _ = action_tensor.shape
                processed_actions = []
                for i in range(chunk_size):
                    single_action = action_tensor[:, i, :]
                    processed_action = self.postprocessor(single_action)
                    processed_actions.append(processed_action)

                # Stack back to (chunk_size, action_dim) on CPU
                action_tensor = torch.stack(processed_actions, dim=1).squeeze(0).detach().cpu()

            # 5. Convert to TimedAction list
            t_0 = obs_t.get_timestamp()
            i_0 = obs_t.get_timestep()
            dt = self.config.environment_dt

            return [
                TimedAction(
                    timestamp=t_0 + i * dt,
                    timestep=i_0 + i,
                    action=action,
                )
                for i, action in enumerate(action_tensor)
            ]
