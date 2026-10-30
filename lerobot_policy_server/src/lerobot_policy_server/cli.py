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

"""CLI entry point for lerobot_policy_server powered by Draccus."""

from __future__ import annotations

import logging
from pathlib import Path
import sys

import draccus

from lerobot.async_inference.helpers import RemotePolicyConfig
from lerobot_policy_server.config import PolicyServerConfig, get_default_config_path
from lerobot_policy_server.engine import InferenceEngine
from lerobot_policy_server.server import ZmqPolicyServer


def setup_logging(log_level: str) -> None:
    """Configure system-wide log formatting."""
    logging.basicConfig(
        level=getattr(logging, log_level.upper(), logging.INFO),
        format="[%(asctime)s] [%(levelname)s] [%(name)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )


def parse_args(
    argv: list[str] | None = None,
    config_path: str | Path | None = None,
) -> PolicyServerConfig:
    """Parse configuration from YAML file and command-line arguments via Draccus."""
    cfg_file = Path(config_path) if config_path else get_default_config_path()
    return draccus.parse(
        config_class=PolicyServerConfig,
        config_path=cfg_file,
        args=argv,
        prog="lerobot-policy-server",
    )


def main(argv: list[str] | None = None) -> int:
    """Main CLI execution flow driven by Draccus configuration."""
    try:
        config = parse_args(argv)
    except Exception as exc:
        print(f"Configuration error: {exc}", file=sys.stderr)
        return 1

    setup_logging(config.log_level)
    logger = logging.getLogger("lerobot_policy_server")
    logger.info(
        "Initializing LeRobot ZeroMQ Policy Server on %s:%d (policy: %s, device: %s)...",
        config.server.host,
        config.server.port,
        config.engine.policy_type,
        config.engine.device,
    )

    engine = InferenceEngine(config=config.engine)
    server = ZmqPolicyServer(config=config.server, engine=engine)

    if config.engine.preload:
        logger.info(
            "Preloading policy '%s' on %s...",
            config.engine.pretrained_name_or_path,
            config.engine.device,
        )
        try:
            server.engine.load_policy(
                RemotePolicyConfig(
                    policy_type=config.engine.policy_type,
                    pretrained_name_or_path=config.engine.pretrained_name_or_path,
                    lerobot_features={},
                    actions_per_chunk=config.engine.actions_per_chunk,
                    device=config.engine.device,
                )
            )
            logger.info("Policy preloaded successfully.")
        except Exception as exc:
            logger.error("Failed to preload policy: %s", exc, exc_info=True)
            return 1

    try:
        server.serve()
    except Exception as exc:
        logger.error("Fatal server error: %s", exc, exc_info=True)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
