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

"""Inference engine for executing LeRobot policies.

Manages policy lifecycle, tensor preprocessing, JPEG image decoding,
action chunk prediction, and performance profiling.
"""

from __future__ import annotations

import collections
import gc
import logging
import threading
import time
from typing import Any

import numpy as np

from lerobot_policy_server.config import InferenceEngineConfig
from lerobot_policy_server.protocol import RemotePolicyConfig, TimedAction, TimedObservation

logger = logging.getLogger("lerobot_policy_server.engine")

# Check optional libraries
try:
    import cv2
    _HAS_CV2 = True
except ImportError:
    _HAS_CV2 = False

try:
    import torch
    _HAS_TORCH = True
except ImportError:
    _HAS_TORCH = False


# ---------------------------------------------------------------------------
# Image decoding and tensor helpers
# ---------------------------------------------------------------------------


def decode_jpeg_to_rgb(jpeg_bytes: bytes) -> np.ndarray:
    """Decode JPEG compressed bytes to RGB uint8 numpy array (H, W, 3)."""
    if not _HAS_CV2:
        raise RuntimeError("OpenCV (cv2) is required for JPEG decoding.")
    buf = np.frombuffer(jpeg_bytes, dtype=np.uint8)
    bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError("cv2.imdecode failed: corrupted or invalid image bytes.")
    return np.ascontiguousarray(bgr[:, :, ::-1], dtype=np.uint8)  # BGR -> RGB


def decode_compressed_images(raw_obs: dict[str, Any]) -> dict[str, Any]:
    """In-place replace any image bytes with decoded RGB numpy arrays."""
    for key, value in raw_obs.items():
        if isinstance(value, (bytes, bytearray)):
            try:
                raw_obs[key] = decode_jpeg_to_rgb(value)
            except Exception as exc:
                logger.warning("Failed to decode image field '%s': %s", key, exc)
    return raw_obs


# ---------------------------------------------------------------------------
# FPS and Latency Tracker
# ---------------------------------------------------------------------------


class FPSTracker:
    """Calculates rolling FPS and latency metrics."""

    def __init__(self, window_size: int = 50) -> None:
        self.window_size = window_size
        self.timestamps: collections.deque[float] = collections.deque(maxlen=window_size)

    def calculate_fps_metrics(self, current_ts: float) -> dict[str, float]:
        """Record timestamp and compute average FPS."""
        self.timestamps.append(current_ts)
        if len(self.timestamps) < 2:
            return {"avg_fps": 0.0, "instant_fps": 0.0}

        dt_window = self.timestamps[-1] - self.timestamps[0]
        count = len(self.timestamps) - 1
        avg_fps = (count / dt_window) if dt_window > 0 else 0.0

        dt_instant = self.timestamps[-1] - self.timestamps[-2]
        instant_fps = (1.0 / dt_instant) if dt_instant > 0 else 0.0

        return {"avg_fps": float(avg_fps), "instant_fps": float(instant_fps)}


# ---------------------------------------------------------------------------
# Inference Engine
# ---------------------------------------------------------------------------


class InferenceEngine:
    """Transport-agnostic inference pipeline for LeRobot policies."""

    def __init__(self, config: InferenceEngineConfig | None = None) -> None:
        self.config = config or InferenceEngineConfig()
        self.running: bool = True
        self.policy: Any = None
        self.policy_config: RemotePolicyConfig | None = None
        self.preprocessor: Any = None
        self.postprocessor: Any = None
        self.device: str = self._resolve_device(self.config.device)
        self.fps_tracker = FPSTracker(window_size=50)
        self._lock = threading.RLock()

    @staticmethod
    def _resolve_device(requested_device: str) -> str:
        """Resolve target device with fallback to CPU if CUDA is unavailable."""
        if not _HAS_TORCH:
            return "cpu"
        if requested_device == "cuda" and not torch.cuda.is_available():
            logger.warning("CUDA requested but not available. Falling back to CPU.")
            return "cpu"
        return requested_device

    def load_policy(self, config: RemotePolicyConfig) -> None:
        """Load and warm up a LeRobot policy based on RemotePolicyConfig."""
        with self._lock:
            self.clear_session()
            self.policy_config = config
            logger.info("Loading policy repo: '%s' (type: %s)...", config.repo_id, config.policy_type)

            if not _HAS_TORCH:
                logger.warning("PyTorch not installed. Engine operating in mock/fallback mode.")
                return

            try:
                # Attempt to import LeRobot components
                from lerobot.policies.factory import get_policy_class, make_pre_post_processors

                policy_cls = get_policy_class(config.policy_type)
                self.policy = policy_cls.from_pretrained(config.repo_id)
                self.policy.to(self.device)
                self.policy.eval()

                # Build pre/post processors
                try:
                    self.preprocessor, self.postprocessor = make_pre_post_processors(
                        self.policy.config,
                        pretrained_path=config.repo_id,
                    )
                except Exception as proc_exc:
                    logger.warning("Could not instantiate LeRobot processors: %s", proc_exc)

                logger.info("Policy '%s' successfully loaded on device: %s", config.repo_id, self.device)
            except Exception as exc:
                logger.error("Failed to load policy '%s': %s", config.repo_id, exc)
                raise RuntimeError(f"Policy loading error: {exc}") from exc

    def clear_session(self) -> None:
        """Release policy and clear GPU memory cache."""
        with self._lock:
            if self.policy is not None:
                del self.policy
                self.policy = None
            self.policy_config = None
            self.preprocessor = None
            self.postprocessor = None

            gc.collect()
            if _HAS_TORCH and torch.cuda.is_available():
                torch.cuda.empty_cache()
            logger.info("Session state cleared.")

    def stop(self) -> None:
        """Stop engine and clean up resources."""
        self.running = False
        self.clear_session()

    def predict_action_chunk(self, obs: TimedObservation) -> list[TimedAction]:
        """Generate a chunk of timed action predictions from an observation."""
        with self._lock:
            raw_obs = dict(obs.get_observation())
            raw_obs = decode_compressed_images(raw_obs)

            timestep = obs.get_timestep()
            timestamp = obs.get_timestamp()
            dt = self.config.environment_dt

            # If torch and real policy are loaded, run neural inference
            if _HAS_TORCH and self.policy is not None:
                return self._infer_policy(raw_obs, timestep, timestamp, dt)

            # Fallback / mock prediction (for testing on Pi 5 or local dry-runs)
            return self._infer_mock(raw_obs, timestep, timestamp, dt)

    def _infer_policy(
        self,
        raw_obs: dict[str, Any],
        start_timestep: int,
        start_timestamp: float,
        dt: float,
    ) -> list[TimedAction]:
        """Execute PyTorch policy inference with tensor preprocessing."""
        # Convert raw observation to torch tensors
        torch_obs: dict[str, Any] = {}
        for key, val in raw_obs.items():
            if isinstance(val, np.ndarray):
                if val.ndim == 3 and val.shape[2] == 3:  # Image (H, W, C) -> (1, C, H, W)
                    tensor = torch.from_numpy(val).permute(2, 0, 1).unsqueeze(0).float() / 255.0
                else:  # Robot joint state vector
                    tensor = torch.from_numpy(val).unsqueeze(0).float()
                torch_obs[key] = tensor.to(self.device, non_blocking=True)
            elif isinstance(val, (int, float)):
                torch_obs[key] = torch.tensor([val], device=self.device)

        with torch.inference_mode():
            if self.preprocessor is not None:
                torch_obs = self.preprocessor(torch_obs)

            # Predict action chunk
            if hasattr(self.policy, "select_action"):
                actions_tensor = self.policy.select_action(torch_obs)
            else:
                actions_tensor = self.policy(torch_obs)

            if self.postprocessor is not None:
                actions_tensor = self.postprocessor(actions_tensor)

        # Convert back to list of TimedAction
        actions_np = actions_tensor.squeeze(0).cpu().numpy()
        if actions_np.ndim == 1:
            actions_np = actions_np[np.newaxis, :]  # Ensure (chunk_size, action_dim)

        actions: list[TimedAction] = []
        for i, act in enumerate(actions_np):
            actions.append(
                TimedAction(
                    timestep=start_timestep + i,
                    timestamp=start_timestamp + (i * dt),
                    action=act,
                )
            )
        return actions

    def _infer_mock(
        self,
        raw_obs: dict[str, Any],
        start_timestep: int,
        start_timestamp: float,
        dt: float,
    ) -> list[TimedAction]:
        """Mock chunk prediction for unit testing and non-GPU environments."""
        # Detect state dimension if present, default to 6 for arm / omni base
        state_dim = 6
        for key, val in raw_obs.items():
            if isinstance(val, np.ndarray) and val.ndim == 1:
                state_dim = len(val)
                break

        chunk_size = (
            self.policy_config.actions_per_chunk
            if self.policy_config
            else 10
        )
        actions: list[TimedAction] = []
        for i in range(chunk_size):
            actions.append(
                TimedAction(
                    timestep=start_timestep + i,
                    timestamp=start_timestamp + (i * dt),
                    action=np.zeros(state_dim, dtype=np.float32),
                )
            )
        return actions
