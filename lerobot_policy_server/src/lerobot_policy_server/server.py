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

"""ZeroMQ Policy Server implementation for LeKiwi robot."""

from __future__ import annotations

import logging
import threading
import time
from typing import Any

import zmq

from lerobot_policy_server.config import InferenceEngineConfig, ZmqServerConfig
from lerobot_policy_server.engine import InferenceEngine
from lerobot_policy_server.protocol import (
    deserialize_observation,
    deserialize_policy_config,
    msgpack,
    serialize_actions,
)

logger = logging.getLogger("lerobot_policy_server.server")


class ZmqPolicyServer:
    """ZeroMQ policy server using a single REP socket for all client interactions.

    Lifecycle:
        1. Client connects and sends ``handshake`` (REQ/REP)
        2. Client sends ``policy_config`` (REQ/REP) -> server loads model weights
        3. Client streams ``infer`` requests (REQ/REP) -> server returns predicted action chunks
        4. Client sends ``disconnect`` (REQ/REP) -> server releases GPU resources
    """

    def __init__(
        self,
        config: ZmqServerConfig | None = None,
        engine: InferenceEngine | None = None,
    ) -> None:
        self.config = config or ZmqServerConfig()
        self.engine = engine or InferenceEngine(
            InferenceEngineConfig(
                fps=self.config.fps,
                inference_latency=self.config.inference_latency,
                obs_queue_timeout=self.config.obs_queue_timeout,
                device=self.config.device,
            )
        )
        self._running: bool = False
        self._server_thread: threading.Thread | None = None
        self._socket: zmq.Socket | None = None
        self._ctx: zmq.Context | None = None
        self._bound_port: int = self.config.port

    @property
    def running(self) -> bool:
        """Check if server loop is actively running."""
        return self._running and self.engine.running

    @property
    def bound_port(self) -> int:
        """Get the actual bound port (useful when bound to ephemeral port 0)."""
        return self._bound_port

    def _handle_request(self, msg: bytes, rep_socket: zmq.Socket) -> None:
        """Process an incoming request and reply via rep_socket."""
        try:
            request: dict[str, Any] = msgpack.unpackb(msg, raw=False)
        except Exception as exc:
            logger.error("Failed to decode message envelope: %s", exc)
            rep_socket.send(
                msgpack.packb({"status": "error", "message": f"Malformed envelope: {exc}"}, use_bin_type=True)
            )
            return

        msg_type = request.get("type")

        if msg_type == "handshake":
            logger.info("Handshake received from client.")
            self.engine.clear_session()
            rep_socket.send(msgpack.packb({"status": "ok"}, use_bin_type=True))

        elif msg_type == "policy_config":
            try:
                cfg_payload = request.get("data")
                if not isinstance(cfg_payload, (bytes, bytearray)):
                    cfg_payload = msgpack.packb(cfg_payload, use_bin_type=True)
                config = deserialize_policy_config(cfg_payload)
                self.engine.load_policy(config)
                logger.info("Policy config successfully applied.")
                rep_socket.send(msgpack.packb({"status": "ok"}, use_bin_type=True))
            except Exception as exc:
                logger.error("Error applying policy config: %s", exc, exc_info=True)
                rep_socket.send(
                    msgpack.packb({"status": "error", "message": str(exc)}, use_bin_type=True)
                )

        elif msg_type == "infer":
            try:
                t0 = time.perf_counter()
                obs_data = request.get("data")
                if not isinstance(obs_data, (bytes, bytearray)):
                    obs_data = bytes(obs_data)

                # Deserialize wire observation
                obs = deserialize_observation(obs_data)
                obs_ts = obs.get_timestamp()
                obs_step = obs.get_timestep()

                metrics = self.engine.fps_tracker.calculate_fps_metrics(obs_ts)

                # Run inference chunk prediction
                t_infer_start = time.perf_counter()
                action_chunk = self.engine.predict_action_chunk(obs)
                t_infer_ms = (time.perf_counter() - t_infer_start) * 1000.0

                # Serialize output actions
                t_ser_start = time.perf_counter()
                actions_bytes = serialize_actions(action_chunk)
                t_ser_ms = (time.perf_counter() - t_ser_start) * 1000.0

                total_ms = (time.perf_counter() - t0) * 1000.0
                logger.debug(
                    "Infer step #%d: total=%.2fms (infer=%.2fms, ser=%.2fms) | fps=%.1f",
                    obs_step,
                    total_ms,
                    t_infer_ms,
                    t_ser_ms,
                    metrics["avg_fps"],
                )

                rep_socket.send(
                    msgpack.packb(
                        {"status": "ok", "actions": actions_bytes},
                        use_bin_type=True,
                    )
                )
            except Exception as exc:
                logger.error("Error executing inference: %s", exc, exc_info=True)
                rep_socket.send(
                    msgpack.packb({"status": "error", "message": str(exc)}, use_bin_type=True)
                )

        elif msg_type == "disconnect":
            logger.info("Client requested disconnect. Resetting session.")
            self.engine.clear_session()
            rep_socket.send(msgpack.packb({"status": "ok"}, use_bin_type=True))

        else:
            logger.warning("Unknown message type received: '%s'", msg_type)
            rep_socket.send(
                msgpack.packb(
                    {"status": "error", "message": f"Unknown message type: {msg_type}"},
                    use_bin_type=True,
                )
            )

    def _run_loop(self, rep_socket: zmq.Socket) -> None:
        """Main event loop running on the worker thread."""
        poller = zmq.Poller()
        poller.register(rep_socket, zmq.POLLIN)

        logger.info("ZmqPolicyServer event loop started on %s:%d", self.config.host, self._bound_port)

        while self._running:
            try:
                # Poll with 100ms timeout to allow checking self._running
                socks = dict(poller.poll(timeout=100))
                if rep_socket in socks and socks[rep_socket] == zmq.POLLIN:
                    msg = rep_socket.recv()
                    self._handle_request(msg, rep_socket)
            except zmq.ZMQError as exc:
                if not self._running or exc.errno in (zmq.ETERM, zmq.ENOTSOCK):
                    break
                logger.error("ZeroMQ loop error: %s", exc)
                break
            except Exception as exc:
                logger.error("Unexpected error in ZMQ loop: %s", exc, exc_info=True)

        logger.info("ZmqPolicyServer event loop terminated.")

    def serve(self) -> None:
        """Start the ZeroMQ server in the foreground (blocking until stopped)."""
        self.start_background()
        try:
            while self._running:
                time.sleep(0.5)
        except KeyboardInterrupt:
            logger.info("Interrupt signal received. Shutting down...")
        finally:
            self.stop()

    def start_background(self) -> None:
        """Start the ZeroMQ server in a background thread."""
        if self._running:
            return

        self._ctx = zmq.Context()
        self._socket = self._ctx.socket(zmq.REP)
        self._socket.setsockopt(zmq.LINGER, 0)

        # Bind to port (port=0 enables ephemeral port selection for testing)
        endpoint = f"tcp://{self.config.host}:{self.config.port}"
        self._socket.bind(endpoint)

        # Retrieve bound port if port was set to 0
        last_endpoint = self._socket.getsockopt_string(zmq.LAST_ENDPOINT)
        self._bound_port = int(last_endpoint.rsplit(":", 1)[-1])

        self._running = True
        self._server_thread = threading.Thread(
            target=self._run_loop,
            args=(self._socket,),
            daemon=True,
            name="ZmqServerThread",
        )
        self._server_thread.start()

    def stop(self) -> None:
        """Stop server and release network sockets cleanly."""
        self._running = False
        self.engine.stop()

        if self._server_thread and self._server_thread.is_alive():
            self._server_thread.join(timeout=1.0)

        if self._socket:
            try:
                self._socket.close(linger=0)
            except Exception:
                pass
            self._socket = None

        if self._ctx:
            try:
                self._ctx.term()
            except Exception:
                pass
            self._ctx = None

        logger.info("ZmqPolicyServer shutdown complete.")
