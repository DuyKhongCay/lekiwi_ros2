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

"""ZeroMQ REQ client implementation providing robust retry and lazy-pirate socket recovery.

Wire format adheres to LeRobot 0.6.1 protocol:
- Header: 4-byte little-endian length prefix + msgpack metadata
- Payload: contiguous raw binary arrays / image bytes
"""

from __future__ import annotations

from dataclasses import asdict
import logging
import struct
import threading
import time
from typing import Any, Optional

import msgpack
import numpy as np
import zmq

from lekiwi_inference.helpers import RemotePolicyConfig, TimedAction, TimedObservation
from lekiwi_inference.transport.base import PolicyTransport

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Wire format serialization & deserialization helpers (LeRobot 0.6.1)
# ---------------------------------------------------------------------------


def serialize_observation(obs: TimedObservation) -> bytes:
    """Serializes TimedObservation to wire binary buffer (header_len + msgpack + raw bytes)."""
    raw_obs = obs.get_observation()
    array_entries: list[tuple[str, bytes, list[int], str]] = []
    scalar_entries: dict[str, Any] = {}

    for key, value in raw_obs.items():
        if isinstance(value, np.ndarray):
            array_entries.append((key, value.tobytes(), list(value.shape), str(value.dtype)))
        elif isinstance(value, bytes):
            # Compressed JPEG image bytes
            array_entries.append((key, value, [len(value)], "bytes"))
        else:
            scalar_entries[key] = value

    header = {
        "type": "observation",
        "timestep": obs.get_timestep(),
        "timestamp": obs.get_timestamp(),
        "must_go": getattr(obs, "must_go", False),
        "arrays": {entry[0]: {"shape": entry[2], "dtype": entry[3]} for entry in array_entries},
        "scalars": scalar_entries,
    }

    header_bytes = msgpack.packb(header, use_bin_type=True)
    header_len = struct.pack("<I", len(header_bytes))
    body_bytes = b"".join(entry[1] for entry in array_entries)
    return header_len + header_bytes + body_bytes


def deserialize_observation(data: bytes) -> TimedObservation:
    """Deserializes wire buffer into TimedObservation."""
    header_len = struct.unpack("<I", data[:4])[0]
    header = msgpack.unpackb(data[4 : 4 + header_len], raw=False)
    body = data[4 + header_len :]

    raw_obs: dict[str, Any] = dict(header.get("scalars", {}))
    offset = 0

    for key, meta in header.get("arrays", {}).items():
        dtype_str = meta["dtype"]
        shape = tuple(meta["shape"])
        if dtype_str == "bytes":
            nbytes = shape[0]
            val = body[offset : offset + nbytes]
            offset += nbytes
            raw_obs[key] = val
        else:
            dt = np.dtype(dtype_str)
            nbytes = int(np.prod(shape)) * dt.itemsize
            arr = np.frombuffer(body[offset : offset + nbytes], dtype=dt).reshape(shape)
            offset += nbytes
            raw_obs[key] = arr

    return TimedObservation(
        timestamp=header.get("timestamp", 0.0),
        timestep=header.get("timestep", 0),
        observation=raw_obs,
        must_go=header.get("must_go", False),
    )


def serialize_actions(actions: list[TimedAction]) -> bytes:
    """Serializes list[TimedAction] to wire binary buffer."""
    actions_meta = []
    body_parts = []
    for action in actions:
        arr = action.get_action()
        if not isinstance(arr, np.ndarray):
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
    header_len = struct.unpack("<I", data[:4])[0]
    header = msgpack.unpackb(data[4 : 4 + header_len], raw=False)
    body = data[4 + header_len :]

    actions: list[TimedAction] = []
    offset = 0
    for meta in header.get("actions", []):
        shape = tuple(meta["shape"])
        dt = np.dtype(meta["dtype"])
        nbytes = int(np.prod(shape)) * dt.itemsize
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


def serialize_policy_config(config: RemotePolicyConfig) -> bytes:
    """Serializes RemotePolicyConfig to msgpack bytes."""
    d = asdict(config)
    return msgpack.packb(d, use_bin_type=True)


def deserialize_policy_config(data: bytes) -> RemotePolicyConfig:
    """Deserializes msgpack bytes into RemotePolicyConfig."""
    d = msgpack.unpackb(data, raw=False)
    return RemotePolicyConfig(
        policy_type=d.get("policy_type", "act"),
        pretrained_name_or_path=d.get("pretrained_name_or_path", d.get("repo_id", "")),
        lerobot_features=d.get("lerobot_features", {}),
        actions_per_chunk=d.get("actions_per_chunk", 40),
        device=d.get("device", "cuda"),
        rename_map=d.get("rename_map", {}),
    )


# ---------------------------------------------------------------------------
# ZeroMQ Policy Transport
# ---------------------------------------------------------------------------


class ZmqPolicyTransport(PolicyTransport):
    """Manages ZeroMQ REQ transport with automatic reconnection and non-blocking timeout handling."""

    def __init__(
        self,
        server_address: str = "127.0.0.1:8090",
        timeout_ms: int = 500,
        max_retries: int = 3,
    ) -> None:
        """Configures transport address and communication timeout deadlines."""
        self.server_address = server_address
        self.timeout_ms = timeout_ms
        self.max_retries = max_retries

        self._ctx: Optional[zmq.Context] = None
        self._socket: Optional[zmq.Socket] = None
        self._lock = threading.Lock()
        self._connected = False
        self._last_rtt_ms = -1.0

    def connect(self) -> bool:
        """Creates socket resources and establishes connection to the remote policy server."""
        with self._lock:
            return self._connect_internal()

    def _connect_internal(self) -> bool:
        """Internal helper handling raw socket allocation without deadlock."""
        try:
            if self._socket is not None:
                self._socket.close(linger=0)
            if self._ctx is None:
                self._ctx = zmq.Context()

            self._socket = self._ctx.socket(zmq.REQ)
            self._socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
            self._socket.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
            self._socket.setsockopt(zmq.LINGER, 0)

            endpoint = (
                f"tcp://{self.server_address}"
                if not self.server_address.startswith("tcp://")
                else self.server_address
            )
            self._socket.connect(endpoint)
            self._connected = True
            logger.info("Connected ZMQ REQ socket to %s", endpoint)
            return True
        except Exception as exc:
            logger.error("Failed to connect ZMQ transport: %s", exc)
            self._connected = False
            return False

    def disconnect(self) -> None:
        """Sends clean disconnect notification and tears down all allocated socket contexts."""
        with self._lock:
            if self._socket is not None and self._connected:
                try:
                    payload = msgpack.packb({"type": "disconnect"}, use_bin_type=True)
                    self._socket.send(payload)
                    self._socket.recv()
                except Exception:
                    pass
                self._socket.close(linger=0)
                self._socket = None

            if self._ctx is not None:
                self._ctx.term()
                self._ctx = None

            self._connected = False
            logger.info("Disconnected ZMQ transport cleanly.")

    def is_connected(self) -> bool:
        """Reports live connectivity status to calling health monitors."""
        return self._connected

    def send_handshake(self) -> bool:
        """Verifies server responsiveness and resets session buffers."""
        req = {"type": "handshake"}
        resp = self._send_request(req)
        return resp is not None and resp.get("status") == "ok"

    def send_policy_config(self, config: RemotePolicyConfig) -> bool:
        """Transmits neural policy specification to initiate remote weights loading."""
        cfg_bytes = serialize_policy_config(config)
        req = {"type": "policy_config", "data": cfg_bytes}
        resp = self._send_request(req)
        return resp is not None and resp.get("status") == "ok"

    def infer(self, observation: TimedObservation) -> Optional[list[TimedAction]]:
        """Sends binary observation frame and parses predicted action horizon chunk."""
        obs_bytes = serialize_observation(observation)
        req = {"type": "infer", "data": obs_bytes}
        t0 = time.perf_counter()
        resp = self._send_request(req)
        if resp is None:
            return None

        self._last_rtt_ms = (time.perf_counter() - t0) * 1000.0
        if resp.get("status") != "ok":
            logger.error("Inference server returned error: %s", resp.get("message"))
            return None

        actions_bytes = resp.get("actions")
        if not actions_bytes:
            return None
        return deserialize_actions(actions_bytes)

    def ping(self) -> float:
        """Returns the most recent measured round-trip latency in milliseconds."""
        return self._last_rtt_ms

    def _send_request(self, request_envelope: dict) -> Optional[dict]:
        """Executes request-reply transaction with Lazy Pirate recovery on timeout."""
        with self._lock:
            if not self._connected or self._socket is None:
                if not self._connect_internal():
                    return None

            try:
                msg_bytes = msgpack.packb(request_envelope, use_bin_type=True)
                self._socket.send(msg_bytes)
                reply_bytes = self._socket.recv()
                return msgpack.unpackb(reply_bytes, raw=False)
            except zmq.Again:
                logger.warning("ZMQ timeout on %s; resetting REQ socket.", self.server_address)
                self._connect_internal()
                return None
            except Exception as exc:
                logger.error("ZMQ transmission failed: %s; resetting socket.", exc)
                self._connect_internal()
                return None
