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

Wire format adheres to LeRobot 0.6.1 protocol:
- Header: 4-byte little-endian length prefix + msgpack metadata
- Payload: contiguous raw binary arrays / compressed image bytes
"""

from __future__ import annotations

from dataclasses import asdict
import struct
from typing import Any

import msgpack
import numpy as np

from lerobot.async_inference.helpers import (
    RemotePolicyConfig,
    TimedAction,
    TimedObservation,
)

__all__ = [
    "RemotePolicyConfig",
    "TimedAction",
    "TimedObservation",
    "serialize_observation",
    "deserialize_observation",
    "serialize_actions",
    "deserialize_actions",
    "serialize_policy_config",
    "deserialize_policy_config",
]


def serialize_observation(obs: TimedObservation) -> bytes:
    """Serializes TimedObservation to wire binary buffer (header_len + msgpack + raw bytes)."""
    raw_obs = dict(obs.get_observation())
    array_entries: list[tuple[str, bytes, list[int], str]] = []
    scalar_entries: dict[str, Any] = {}

    # Canonical RTC parameters from TimedObservation attributes
    rtc_leftover = getattr(obs, "prev_chunk_left_over", None)
    rtc_delay = getattr(obs, "inference_delay", None)
    rtc_horizon = getattr(obs, "execution_horizon", None)

    if rtc_leftover is not None:
        if hasattr(rtc_leftover, "detach"):
            rtc_leftover = rtc_leftover.detach().cpu().numpy()
        elif not isinstance(rtc_leftover, np.ndarray):
            rtc_leftover = np.asarray(rtc_leftover, dtype=np.float32)
        raw_obs["prev_chunk_left_over"] = np.ascontiguousarray(rtc_leftover, dtype=np.float32)

    for key, value in raw_obs.items():
        if isinstance(value, np.ndarray):
            array_entries.append((key, value.tobytes(), list(value.shape), str(value.dtype)))
        elif isinstance(value, (bytes, bytearray)):
            # Compressed JPEG image bytes
            val_bytes = bytes(value)
            array_entries.append((key, val_bytes, [len(val_bytes)], "bytes"))
        else:
            scalar_entries[key] = value

    header = {
        "type": "observation",
        "timestep": obs.get_timestep(),
        "timestamp": obs.get_timestamp(),
        "must_go": getattr(obs, "must_go", False),
        "arrays": {entry[0]: {"shape": entry[2], "dtype": entry[3]} for entry in array_entries},
        "scalars": scalar_entries,
        "rtc": {
            "inference_delay": int(rtc_delay) if rtc_delay is not None else None,
            "execution_horizon": int(rtc_horizon) if rtc_horizon is not None else None,
        },
    }

    header_bytes = msgpack.packb(header, use_bin_type=True)
    header_len = struct.pack("<I", len(header_bytes))
    body_bytes = b"".join(entry[1] for entry in array_entries)
    return header_len + header_bytes + body_bytes


def deserialize_observation(data: bytes) -> TimedObservation:
    """Deserializes wire buffer into TimedObservation with RTC metadata extraction."""
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
        dtype_str = meta["dtype"]
        shape = tuple(meta["shape"])
        if dtype_str == "bytes":
            nbytes = shape[0]
            if offset + nbytes > len(body):
                raise ValueError(f"Body truncated reading bytes field '{key}'")
            val = body[offset : offset + nbytes]
            offset += nbytes
            raw_obs[key] = val
        else:
            dt = np.dtype(dtype_str)
            nbytes = int(np.prod(shape)) * dt.itemsize
            if offset + nbytes > len(body):
                raise ValueError(f"Body truncated reading array '{key}'")
            arr = np.frombuffer(body[offset : offset + nbytes], dtype=dt).reshape(shape).copy()
            offset += nbytes
            raw_obs[key] = arr

    # Extract canonical RTC metadata (Zero backward-compatibility fallback)
    rtc_meta = header.get("rtc", {})
    prev_leftover = raw_obs.pop("prev_chunk_left_over", None)
    inference_delay = rtc_meta.get("inference_delay")
    execution_horizon = rtc_meta.get("execution_horizon")

    obs = TimedObservation(
        timestamp=header.get("timestamp", 0.0),
        timestep=header.get("timestep", 0),
        observation=raw_obs,
        must_go=header.get("must_go", False),
    )
    obs.prev_chunk_left_over = prev_leftover
    obs.inference_delay = inference_delay
    obs.execution_horizon = execution_horizon
    return obs


def serialize_actions(actions: list[TimedAction]) -> bytes:
    """Serializes list[TimedAction] to wire binary buffer."""
    actions_meta = []
    body_parts = []
    for action in actions:
        arr = action.get_action()
        if hasattr(arr, "detach"):
            arr = arr.detach().cpu().numpy()
        elif not isinstance(arr, np.ndarray):
            arr = np.asarray(arr, dtype=np.float32)

        actions_meta.append(
            {
                "timestamp": action.get_timestamp(),
                "timestep": action.get_timestep(),
                "shape": list(arr.shape),
                "dtype": str(arr.dtype),
            }
        )
        body_parts.append(arr.tobytes())

    header = {"actions": actions_meta}
    header_bytes = msgpack.packb(header, use_bin_type=True)
    header_len = struct.pack("<I", len(header_bytes))
    return header_len + header_bytes + b"".join(body_parts)


def deserialize_actions(data: bytes) -> list[TimedAction]:
    """Deserializes wire buffer into list[TimedAction]."""
    if len(data) < 4:
        raise ValueError("Invalid actions payload: data length < 4 bytes")

    header_len = struct.unpack("<I", data[:4])[0]
    if len(data) < 4 + header_len:
        raise ValueError(
            f"Actions payload truncated: expected header length {header_len}, "
            f"got total {len(data)} bytes"
        )

    header = msgpack.unpackb(data[4 : 4 + header_len], raw=False)
    body = data[4 + header_len :]

    actions: list[TimedAction] = []
    offset = 0
    for meta in header.get("actions", []):
        shape = tuple(meta["shape"])
        dt = np.dtype(meta["dtype"])
        nbytes = int(np.prod(shape)) * dt.itemsize
        if offset + nbytes > len(body):
            raise ValueError("Body truncated reading action item")
        arr = np.frombuffer(body[offset : offset + nbytes], dtype=dt).reshape(shape)
        offset += nbytes
        actions.append(
            TimedAction(
                timestamp=meta["timestamp"],
                timestep=meta["timestep"],
                action=arr.copy(),
            )
        )
    return actions


def serialize_policy_config(config: RemotePolicyConfig | dict[str, Any]) -> bytes:
    """Serializes RemotePolicyConfig to msgpack bytes."""
    if isinstance(config, RemotePolicyConfig):
        d = asdict(config)
    elif hasattr(config, "__dict__"):
        d = dict(config.__dict__)
    else:
        d = dict(config)
    return msgpack.packb(d, use_bin_type=True)


def deserialize_policy_config(data: bytes) -> RemotePolicyConfig:
    """Deserializes msgpack bytes into RemotePolicyConfig."""
    d = msgpack.unpackb(data, raw=False)
    return RemotePolicyConfig(
        policy_type=d.get("policy_type", "smolvla"),
        pretrained_name_or_path=d.get(
            "pretrained_name_or_path",
            d.get("repo_id", "/home/duykhongcay/lerobot_ws/lekiwi_chess_smolvla"),
        ),
        lerobot_features=d.get("lerobot_features", {}),
        actions_per_chunk=d.get("actions_per_chunk", 30),
        device=d.get("device", "cuda"),
        rename_map=d.get("rename_map", {}),
    )
