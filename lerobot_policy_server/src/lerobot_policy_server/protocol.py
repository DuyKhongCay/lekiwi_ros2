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

"""Binary wire protocol for LeRobot ZeroMQ communication.

The wire protocol uses a binary format:
    [4 bytes uint32-LE header length] + [msgpack metadata header] + [concatenated raw array bytes]

This avoids python pickle for security, low latency, and cross-language compatibility.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import Any

import numpy as np

# Safe fallback between standard msgpack and umsgpack
try:
    import msgpack
except ImportError:
    try:
        import umsgpack as msgpack  # type: ignore[no-redef]
    except ImportError as err:
        raise ImportError(
            "Neither 'msgpack' nor 'umsgpack' is installed. "
            "Please install msgpack: pip install msgpack"
        ) from err


# ---------------------------------------------------------------------------
# Data classes (LeRobot compatible fallbacks)
# ---------------------------------------------------------------------------

try:
    from lerobot.async_inference.helpers import (
        RemotePolicyConfig,
        TimedAction,
        TimedObservation,
    )
except ImportError:

    @dataclass
    class TimedObservation:
        """Observation with associated timing information."""

        timestamp: float
        timestep: int
        observation: dict[str, Any]
        must_go: bool = False

        def get_timestamp(self) -> float:
            return self.timestamp

        def get_timestep(self) -> int:
            return self.timestep

        def get_observation(self) -> dict[str, Any]:
            return self.observation

    @dataclass
    class TimedAction:
        """Action with associated timing information."""

        timestamp: float
        timestep: int
        action: Any  # torch.Tensor or numpy.ndarray

        def get_timestamp(self) -> float:
            return self.timestamp

        def get_timestep(self) -> int:
            return self.timestep

        def get_action(self) -> Any:
            return self.action

    @dataclass
    class RemotePolicyConfig:
        """Configuration sent by client to load a remote policy."""

        repo_id: str = ""
        policy_type: str = ""
        lerobot_features: dict[str, Any] = field(default_factory=dict)
        fps: float = 30.0
        actions_per_chunk: int = 50
        chunk_size_threshold: float = 0.6
        extra: dict[str, Any] = field(default_factory=dict)

        def __init__(self, **kwargs: Any) -> None:
            self.repo_id = kwargs.get("repo_id", "")
            self.policy_type = kwargs.get("policy_type", "")
            self.lerobot_features = kwargs.get("lerobot_features", {})
            self.fps = float(kwargs.get("fps", 30.0))
            self.actions_per_chunk = int(kwargs.get("actions_per_chunk", 50))
            self.chunk_size_threshold = float(kwargs.get("chunk_size_threshold", 0.6))
            self.extra = {
                k: v
                for k, v in kwargs.items()
                if k
                not in (
                    "repo_id",
                    "policy_type",
                    "lerobot_features",
                    "fps",
                    "actions_per_chunk",
                    "chunk_size_threshold",
                )
            }


# ---------------------------------------------------------------------------
# Wire serialization & deserialization helpers
# ---------------------------------------------------------------------------


def deserialize_observation(data: bytes) -> TimedObservation:
    """Deserialize wire bytes into a TimedObservation.

    Format: [4-byte header length] + [msgpack header] + [raw body bytes]
    """
    if len(data) < 4:
        raise ValueError("Invalid observation payload: data length < 4 bytes")

    header_len = struct.unpack("<I", data[:4])[0]
    if len(data) < 4 + header_len:
        raise ValueError(
            f"Observation payload truncated: expected header length {header_len}, "
            f"got total {len(data)} bytes"
        )

    header = msgpack.unpackb(data[4 : 4 + header_len], raw=False)
    body = data[4 + header_len :]

    raw_obs: dict[str, Any] = dict(header.get("scalars", {}))

    offset = 0
    for key, meta in header.get("arrays", {}).items():
        shape = tuple(meta["shape"])
        dtype = np.dtype(meta["dtype"])
        nbytes = int(np.prod(shape)) * dtype.itemsize
        if offset + nbytes > len(body):
            raise ValueError(
                f"Body truncated while reading array '{key}' with shape {shape} and dtype {dtype}"
            )
        arr = np.frombuffer(body[offset : offset + nbytes], dtype=dtype).reshape(shape)
        offset += nbytes
        raw_obs[key] = arr

    # If any images are sent as raw byte strings (e.g. JPEG compressed)
    for key, val in header.get("bytes_fields", {}).items():
        raw_obs[key] = val

    return TimedObservation(
        timestamp=header["timestamp"],
        timestep=header["timestep"],
        observation=raw_obs,
        must_go=header.get("must_go", False),
    )


def serialize_observation(obs: TimedObservation) -> bytes:
    """Serialize a TimedObservation into wire format (for clients / test fixtures)."""
    raw_obs = obs.get_observation()
    scalars: dict[str, Any] = {}
    arrays_meta: dict[str, Any] = {}
    bytes_fields: dict[str, bytes] = {}
    body_parts: list[bytes] = []

    for key, val in raw_obs.items():
        if isinstance(val, np.ndarray):
            arrays_meta[key] = {
                "shape": list(val.shape),
                "dtype": str(val.dtype),
            }
            body_parts.append(val.tobytes())
        elif isinstance(val, (bytes, bytearray)):
            bytes_fields[key] = bytes(val)
        elif isinstance(val, (int, float, str, bool, list)):
            scalars[key] = val

    header = {
        "timestamp": obs.get_timestamp(),
        "timestep": obs.get_timestep(),
        "must_go": getattr(obs, "must_go", False),
        "scalars": scalars,
        "arrays": arrays_meta,
        "bytes_fields": bytes_fields,
    }
    header_bytes = msgpack.packb(header, use_bin_type=True)
    header_len = struct.pack("<I", len(header_bytes))

    return header_len + header_bytes + b"".join(body_parts)


def serialize_actions(actions: list[TimedAction]) -> bytes:
    """Serialize a list of TimedAction to wire format.

    Format: [4-byte header length] + [msgpack header] + [raw tensor bytes]
    """
    action_metas = []
    tensor_parts: list[bytes] = []

    for ta in actions:
        act = ta.get_action()
        # Handle PyTorch Tensor or numpy ndarray
        if hasattr(act, "detach"):
            arr = act.detach().cpu().numpy()
        elif isinstance(act, np.ndarray):
            arr = act
        else:
            arr = np.array(act, dtype=np.float32)

        action_metas.append(
            {
                "timestamp": ta.get_timestamp(),
                "timestep": ta.get_timestep(),
                "shape": list(arr.shape),
                "dtype": str(arr.dtype),
            }
        )
        tensor_parts.append(arr.tobytes())

    header = {"type": "actions", "actions": action_metas}
    header_bytes = msgpack.packb(header, use_bin_type=True)
    header_len = struct.pack("<I", len(header_bytes))

    return header_len + header_bytes + b"".join(tensor_parts)


def deserialize_actions(data: bytes) -> list[TimedAction]:
    """Deserialize wire bytes back into a list of TimedAction (for clients/tests)."""
    if len(data) < 4:
        raise ValueError("Invalid actions payload: data length < 4 bytes")

    header_len = struct.unpack("<I", data[:4])[0]
    header = msgpack.unpackb(data[4 : 4 + header_len], raw=False)
    body = data[4 + header_len :]

    actions: list[TimedAction] = []
    offset = 0

    for meta in header.get("actions", []):
        shape = tuple(meta["shape"])
        dtype = np.dtype(meta["dtype"])
        nbytes = int(np.prod(shape)) * dtype.itemsize
        arr = np.frombuffer(body[offset : offset + nbytes], dtype=dtype).reshape(shape)
        offset += nbytes
        actions.append(
            TimedAction(
                timestamp=meta["timestamp"],
                timestep=meta["timestep"],
                action=arr,
            )
        )

    return actions


def deserialize_policy_config(payload: bytes) -> RemotePolicyConfig:
    """Deserialize msgpack bytes into a RemotePolicyConfig."""
    d = msgpack.unpackb(payload, raw=False)
    return RemotePolicyConfig(**d)


def serialize_policy_config(config: RemotePolicyConfig | dict[str, Any]) -> bytes:
    """Serialize a RemotePolicyConfig or config dict to msgpack bytes."""
    if hasattr(config, "__dict__"):
        d = dict(config.__dict__)
    else:
        d = dict(config)
    return msgpack.packb(d, use_bin_type=True)
