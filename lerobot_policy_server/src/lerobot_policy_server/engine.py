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
from lerobot.policies.rtc.configuration_rtc import RTCAttentionSchedule, RTCConfig
from lerobot.processor import PolicyProcessorPipeline
from lerobot.utils.constants import OBS_STATE
from lerobot_policy_server.config import InferenceEngineConfig
import lerobot_policy_server.processors  # noqa: F401 - Register custom processor steps (e.g. chess_goal_noise)

logger = logging.getLogger("lerobot_policy_server.engine")


def _decode_jpeg_to_rgb(jpeg_bytes: bytes) -> np.ndarray:
    """Decode JPEG compressed bytes to RGB uint8 numpy array (H, W, 3)."""
    buf = np.frombuffer(jpeg_bytes, dtype=np.uint8)
    bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError("cv2.imdecode failed: corrupted or invalid image bytes.")
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def _raw_observation_to_observation(raw_obs: dict[str, Any]) -> Observation:
    """Convert raw robot observation dict to policy-ready PyTorch tensors for SmolVLA.

    - Fail-Fast check: 'task' instruction must be a non-empty string prompt.
    - Decodes JPEG bytes to RGB uint8 (H, W, 3).
    - Image keys (observation.images.*): converts to float32 [0.0, 1.0] (1, C, H, W)
      preserving native aspect ratio without pre-resizing.
    - State key (observation.state): verifies exact 12D unified layout
      [6 joints + 3D ee_xyz + 3D delta_xyz] per chess_rosbag_to_lerobot, converts to (1, 12).
    """
    raw_task = raw_obs.get("task")
    if not raw_task or not isinstance(raw_task, str) or not raw_task.strip():
        raise ValueError(
            "Missing or empty 'task' instruction in observation payload. "
            "SmolVLA requires an explicit, non-empty language task string."
        )

    obs_tensors: Observation = {"task": raw_task.strip()}

    for key, value in raw_obs.items():
        if key == "task":
            continue

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
        elif key == OBS_STATE:
            if isinstance(value, np.ndarray):
                arr = value if value.flags.writeable else value.copy()
                t = torch.from_numpy(arr).to(dtype=torch.float32)
            elif isinstance(value, torch.Tensor):
                t = value.to(dtype=torch.float32)
            else:
                t = torch.tensor(value, dtype=torch.float32)
            if t.ndim == 1:
                t = t.unsqueeze(0)
            if t.shape[-1] != 12:
                raise ValueError(
                    f"SmolVLA requires 12D observation.state [6 joints + 3D ee_xyz + 3D delta_xyz], "
                    f"got shape {tuple(t.shape)}"
                )
            obs_tensors[key] = t
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
        self.actions_per_chunk: int = self.config.actions_per_chunk
        self.fps_tracker = FPSTracker(target_fps=float(self.config.fps))
        self._lock = threading.RLock()

    @staticmethod
    def _resolve_device(requested_device: str) -> str:
        """Resolve requested device with fallback to CPU if CUDA is unavailable."""
        if requested_device == "cuda" and not torch.cuda.is_available():
            logger.warning("CUDA requested but not available. Falling back to CPU.")
            return "cpu"
        if requested_device == "cuda":
            torch.backends.cudnn.benchmark = True
        return requested_device

    def _synthesize_dummy_observation(self) -> TimedObservation:
        """Synthesize a dummy TimedObservation matching the policy input specifications."""
        obs_payload: dict[str, Any] = {"task": "warmup"}
        input_features = getattr(getattr(self.policy, "config", None), "input_features", {})

        for feat_name, feat in input_features.items():
            if is_image_key(feat_name):
                h = feat.shape[1] if hasattr(feat, "shape") and len(feat.shape) >= 2 else 384
                w = feat.shape[2] if hasattr(feat, "shape") and len(feat.shape) >= 3 else 384
                dummy_rgb = np.zeros((h, w, 3), dtype=np.uint8)
                _, enc = cv2.imencode(".jpg", dummy_rgb)
                obs_payload[feat_name] = enc.tobytes()
            elif feat_name == OBS_STATE:
                dim = feat.shape[0] if hasattr(feat, "shape") else 12
                obs_payload[feat_name] = np.zeros(dim, dtype=np.float32)

        if OBS_STATE not in obs_payload:
            obs_payload[OBS_STATE] = np.zeros(12, dtype=np.float32)

        return TimedObservation(
            timestamp=time.perf_counter(),
            timestep=0,
            observation=obs_payload,
            must_go=True,
        )

    def warmup(self) -> None:
        """Execute a dummy forward pass to compile CUDA kernels and JIT caches."""
        if self.policy is None or self.preprocessor is None or self.postprocessor is None:
            logger.warning("Inference warmup skipped: policy or processors not loaded.")
            return

        if self.device != "cuda" and not getattr(self.config, "force_warmup", False):
            logger.debug("Inference warmup skipped: device is '%s' (CUDA warmup only).", self.device)
            return

        logger.info("Executing inference warmup forward pass on %s...", self.device)
        t_start = time.perf_counter()
        dummy_obs = self._synthesize_dummy_observation()

        _ = self.predict_action_chunk(dummy_obs)

        # RTC gradient guidance warmup if RTC is active
        if self.policy is not None and getattr(self.policy, "_rtc_enabled", lambda: False)():
            rtc_dummy_obs = self._synthesize_dummy_observation()
            rtc_dummy_obs.prev_chunk_left_over = np.zeros((10, 6), dtype=np.float32)
            rtc_dummy_obs.inference_delay = 2
            _ = self.predict_action_chunk(rtc_dummy_obs)

        if self.device == "cuda" and torch.cuda.is_available():
            torch.cuda.synchronize()

        elapsed_ms = (time.perf_counter() - t_start) * 1000.0
        logger.info("Inference warmup completed in %.1fms. Ready for client inference.", elapsed_ms)

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

            # Configure Real-Time Chunking (RTC) if enabled in config
            rtc_cfg = getattr(self.config, "rtc", None)
            if (
                rtc_cfg
                and getattr(rtc_cfg, "enabled", False)
                and hasattr(self.policy, "supports_rtc")
                and self.policy.supports_rtc()
            ):
                schedule_str = str(getattr(rtc_cfg, "prefix_attention_schedule", "linear")).upper()
                schedule_enum = getattr(RTCAttentionSchedule, schedule_str, RTCAttentionSchedule.LINEAR)
                self.policy.config.rtc_config = RTCConfig(
                    enabled=True,
                    execution_horizon=getattr(rtc_cfg, "execution_horizon", 10),
                    max_guidance_weight=getattr(rtc_cfg, "max_guidance_weight", 10.0),
                    prefix_attention_schedule=schedule_enum,
                )
                self.policy.init_rtc_processor()
                logger.info(
                    "Real-Time Chunking (RTC) enabled on %s: schedule=%s, horizon=%d, max_weight=%.1f",
                    config.policy_type,
                    schedule_str,
                    rtc_cfg.execution_horizon,
                    rtc_cfg.max_guidance_weight,
                )

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

            if getattr(self.config, "warmup", True):
                self.warmup()

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

    def _normalize_action_prefix(self, raw_prefix: torch.Tensor) -> torch.Tensor:
        """Normalize physical actions (radians) into model latent normalized space."""
        if self.preprocessor is None:
            return raw_prefix
        for step in self.preprocessor.steps:
            if "Normalizer" in type(step).__name__ and hasattr(step, "features") and "action" in step.features:
                transition = {"action": raw_prefix}
                normed = step(transition)
                return normed["action"]
        return raw_prefix

    def _prepare_rtc_kwargs(self, obs_t: TimedObservation) -> dict[str, Any]:
        """Extract and prepare Real-Time Chunking (RTC) arguments for policy inference."""
        if not (self.policy is not None and getattr(self.policy, "_rtc_enabled", lambda: False)()):
            return {}

        leftover_raw = getattr(obs_t, "prev_chunk_left_over", None)
        if leftover_raw is None and hasattr(obs_t, "get_observation"):
            raw_obs = obs_t.get_observation()
            leftover_raw = raw_obs.get("prev_chunk_left_over") or raw_obs.get("rtc.prev_chunk_left_over")

        if leftover_raw is None:
            return {}

        if isinstance(leftover_raw, np.ndarray):
            leftover_tensor = torch.from_numpy(leftover_raw).to(dtype=torch.float32)
        elif isinstance(leftover_raw, torch.Tensor):
            leftover_tensor = leftover_raw.to(dtype=torch.float32)
        else:
            leftover_tensor = torch.tensor(leftover_raw, dtype=torch.float32)

        # Normalize physical action prefix using dataset stats on server
        normed_leftover = self._normalize_action_prefix(leftover_tensor).to(self.device)
        if normed_leftover.ndim == 2:
            normed_leftover = normed_leftover.unsqueeze(0)  # (1, T, action_dim)

        inference_delay = getattr(obs_t, "inference_delay", None)
        if inference_delay is None and hasattr(obs_t, "get_observation"):
            raw_obs = obs_t.get_observation()
            inference_delay = raw_obs.get("inference_delay") or raw_obs.get("rtc.inference_delay")

        execution_horizon = getattr(obs_t, "execution_horizon", None)
        if execution_horizon is None and hasattr(obs_t, "get_observation"):
            raw_obs = obs_t.get_observation()
            execution_horizon = raw_obs.get("execution_horizon") or raw_obs.get("rtc.execution_horizon")

        rtc_kwargs: dict[str, Any] = {"prev_chunk_left_over": normed_leftover}
        if inference_delay is not None:
            rtc_kwargs["inference_delay"] = int(inference_delay)
        if execution_horizon is not None:
            rtc_kwargs["execution_horizon"] = int(execution_horizon)

        return rtc_kwargs

    def _get_action_chunk(
        self,
        observation: dict[str, torch.Tensor],
        **kwargs: Any,
    ) -> torch.Tensor:
        """Get an action chunk from the policy sliced to actions_per_chunk."""
        chunk = self.policy.predict_action_chunk(observation, **kwargs)
        if chunk.ndim != 3:
            chunk = chunk.unsqueeze(0)  # (1, chunk_size, action_dim)
        return chunk[:, : self.actions_per_chunk, :]

    def predict_action_chunk(self, obs_t: TimedObservation) -> list[TimedAction]:
        """Generate a chunk of timed action predictions from a TimedObservation.

        Pipeline:
        1. Decode JPEG & convert raw observation to LeRobot Observation tensors.
        2. Apply preprocessor (normalization, batching, device placement).
        3. Run policy.predict_action_chunk() with optional RTC guidance parameters.
        4. Apply postprocessor in a single vectorized pass.
        5. Package into list[TimedAction] with environment_dt timestamps.
        """
        with self._lock:
            if getattr(obs_t, "must_go", False):
                self.fps_tracker.reset()

            if self.policy is None or self.preprocessor is None or self.postprocessor is None:
                raise RuntimeError("Policy not loaded. Send policy_config request first.")

            # 1. Prepare observation & RTC guidance parameters
            observation: Observation = _raw_observation_to_observation(obs_t.get_observation())
            rtc_kwargs = self._prepare_rtc_kwargs(obs_t)

            # 2. Apply preprocessor
            with torch.inference_mode():
                observation = self.preprocessor(observation)

            # 3. Get action chunk
            with torch.no_grad():
                action_tensor = self._get_action_chunk(observation, **rtc_kwargs)

                # 4. Apply postprocessor in a single vectorized pass
                action_tensor = self.postprocessor(action_tensor).squeeze(0).detach().cpu()

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
