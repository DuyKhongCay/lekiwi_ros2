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

"""Wire protocol definition and serialization utilities for ZeroMQ inference."""

from __future__ import annotations

from dataclasses import dataclass, field
import struct
from typing import Any

import msgpack
import numpy as np


@dataclass
class TimedObservation:
    """Carries time-indexed sensor snapshots to ensure temporal synchronization across network hops."""

    timestamp: float
    timestep: int
    observation: dict[str, Any]
    must_go: bool = False

    def get_timestamp(self) -> float:
        """Returns epoch timestamp for network jitter and latency tracking."""
        return self.timestamp

    def get_timestep(self) -> int:
        """Returns sequence index to detect lost or out-of-order packets."""
        return self.timestep

    def get_observation(self) -> dict[str, Any]:
        """Provides access to raw images and kinematic states payload."""
        return self.observation


@dataclass
class TimedAction:
    """Encapsulates future motor commands predicted by the physical-AI policy."""

    timestamp: float
    timestep: int
    action: np.ndarray

    def get_timestamp(self) -> float:
        """Returns server generation timestamp to evaluate action chunk freshness."""
        return self.timestamp

    def get_timestep(self) -> int:
        """Returns horizon step index within the predicted action chunk."""
        return self.timestep

    def get_action(self) -> np.ndarray:
        """Returns raw joint position command array for motor actuation."""
        return self.action


@dataclass
class RemotePolicyConfig:
    """Configures model parameters on the remote GPU server during initial handshake."""

    repo_id: str = ""
    policy_type: str = "act"
    lerobot_features: dict[str, Any] = field(default_factory=dict)
    fps: float = 50.0
    actions_per_chunk: int = 100
    chunk_size_threshold: float = 0.5
    extra: dict[str, Any] = field(default_factory=dict)


def serialize_observation(obs: TimedObservation) -> bytes:
    """Encodes sensor observations into length-prefixed msgpack plus raw byte buffers to avoid pickle overhead."""
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


def deserialize_observation(data: bytes) -> TimedObservation:
    """Reconstructs timed observation structures from received ZeroMQ binary wire frames."""
    if len(data) < 4:
        raise ValueError("Invalid observation payload: length < 4 bytes")

    header_len = struct.unpack("<I", data[:4])[0]
    if len(data) < 4 + header_len:
        raise ValueError(f"Observation header truncated: expected {header_len} bytes")

    header = msgpack.unpackb(data[4 : 4 + header_len], raw=False)
    body = data[4 + header_len :]

    raw_obs: dict[str, Any] = dict(header.get("scalars", {}))
    offset = 0
    for key, meta in header.get("arrays", {}).items():
        shape = tuple(meta["shape"])
        dtype = np.dtype(meta["dtype"])
        nbytes = int(np.prod(shape)) * dtype.itemsize
        if offset + nbytes > len(body):
            raise ValueError(f"Body truncated while reading array '{key}'")
        raw_obs[key] = np.frombuffer(body[offset : offset + nbytes], dtype=dtype).reshape(shape)
        offset += nbytes

    for key, val in header.get("bytes_fields", {}).items():
        raw_obs[key] = val

    return TimedObservation(
        timestamp=header["timestamp"],
        timestep=header["timestep"],
        observation=raw_obs,
        must_go=header.get("must_go", False),
    )


def serialize_actions(actions: list[TimedAction]) -> bytes:
    """Packs batch action predictions into compact binary buffers for low-latency transmission."""
    action_metas = []
    tensor_parts: list[bytes] = []

    for ta in actions:
        act = ta.get_action()
        if hasattr(act, "detach"):
            act_np = act.detach().cpu().numpy()
        elif isinstance(act, np.ndarray):
            act_np = act
        else:
            act_np = np.asarray(act, dtype=np.float32)

        meta = {
            "timestamp": ta.get_timestamp(),
            "timestep": ta.get_timestep(),
            "shape": list(act_np.shape),
            "dtype": str(act_np.dtype),
        }
        action_metas.append(meta)
        tensor_parts.append(act_np.tobytes())

    header = {"type": "actions", "actions": action_metas}
    header_bytes = msgpack.packb(header, use_bin_type=True)
    header_len = struct.pack("<I", len(header_bytes))
    return header_len + header_bytes + b"".join(tensor_parts)


def deserialize_actions(data: bytes) -> list[TimedAction]:
    """Unpacks policy action predictions from binary payload into typed TimedAction objects."""
    if len(data) < 4:
        raise ValueError("Invalid actions payload: length < 4 bytes")

    header_len = struct.unpack("<I", data[:4])[0]
    if len(data) < 4 + header_len:
        raise ValueError(f"Actions header truncated: expected {header_len} bytes")

    header = msgpack.unpackb(data[4 : 4 + header_len], raw=False)
    body = data[4 + header_len :]

    actions: list[TimedAction] = []
    offset = 0
    for meta in header.get("actions", []):
        shape = tuple(meta["shape"])
        dtype = np.dtype(meta["dtype"])
        nbytes = int(np.prod(shape)) * dtype.itemsize
        if offset + nbytes > len(body):
            raise ValueError("Body truncated while parsing action arrays")
        arr = np.frombuffer(body[offset : offset + nbytes], dtype=dtype).reshape(shape).copy()
        offset += nbytes
        actions.append(TimedAction(timestamp=meta["timestamp"], timestep=meta["timestep"], action=arr))

    return actions


def serialize_policy_config(config: RemotePolicyConfig | dict[str, Any]) -> bytes:
    """Serializes remote policy configuration to msgpack bytes for transmission."""
    if hasattr(config, "__dict__"):
        d = dict(config.__dict__)
    else:
        d = dict(config)
    return msgpack.packb(d, use_bin_type=True)


def deserialize_policy_config(payload: bytes) -> RemotePolicyConfig:
    """Deserializes msgpack bytes into a typed RemotePolicyConfig."""
    d = msgpack.unpackb(payload, raw=False)
    return RemotePolicyConfig(**d)

