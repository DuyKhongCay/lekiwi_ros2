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

"""CLI entry point for lerobot_policy_server."""

from __future__ import annotations

import argparse
import logging
import os
import sys

from lerobot_policy_server.config import ZmqServerConfig
from lerobot_policy_server.server import ZmqPolicyServer


def setup_logging(log_level: str) -> None:
    """Configure system-wide log formatting."""
    logging.basicConfig(
        level=getattr(logging, log_level.upper(), logging.INFO),
        format="[%(asctime)s] [%(levelname)s] [%(name)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )


def build_parser() -> argparse.ArgumentParser:
    """Construct command-line argument parser."""
    parser = argparse.ArgumentParser(
        prog="lerobot-policy-server",
        description="ZeroMQ Policy Inference Server for LeKiwi robot (LeRobot)",
    )
    parser.add_argument(
        "--host",
        type=str,
        default=os.getenv("POLICY_SERVER_HOST", "0.0.0.0"),
        help="Host address to bind ZeroMQ REP socket (default: 0.0.0.0)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.getenv("POLICY_SERVER_PORT", "8090")),
        help="TCP port to bind (default: 8090)",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=int(os.getenv("POLICY_SERVER_FPS", "30")),
        help="Target inference frames per second (default: 30)",
    )
    parser.add_argument(
        "--inference-latency",
        type=float,
        default=float(os.getenv("POLICY_SERVER_INFERENCE_LATENCY", "0.033")),
        help="Target inference latency in seconds (default: 0.033)",
    )
    parser.add_argument(
        "--obs-queue-timeout",
        type=float,
        default=float(os.getenv("POLICY_SERVER_OBS_TIMEOUT", "2.0")),
        help="Observation timeout in seconds (default: 2.0)",
    )
    parser.add_argument(
        "--device",
        type=str,
        default=os.getenv("POLICY_SERVER_DEVICE", "cuda"),
        help="Computation device ('cuda' or 'cpu', default: cuda)",
    )
    parser.add_argument(
        "--log-level",
        type=str,
        default=os.getenv("LOGLEVEL", "INFO"),
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging level (default: INFO)",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """Main CLI execution flow."""
    parser = build_parser()
    args = parser.parse_args(argv)

    setup_logging(args.log_level)
    logger = logging.getLogger("lerobot_policy_server")
    logger.info("Initializing LeRobot ZeroMQ Policy Server on %s:%d...", args.host, args.port)

    config = ZmqServerConfig(
        host=args.host,
        port=args.port,
        fps=args.fps,
        inference_latency=args.inference_latency,
        obs_queue_timeout=args.obs_queue_timeout,
        device=args.device,
    )

    server = ZmqPolicyServer(config)
    try:
        server.serve()
    except Exception as exc:
        logger.error("Fatal server error: %s", exc, exc_info=True)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
