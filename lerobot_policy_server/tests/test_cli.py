# Copyright 2026 LeKiwi Robotics Team

from lerobot_policy_server.cli import build_parser


def test_cli_parser_defaults():
    parser = build_parser()
    args = parser.parse_args([])

    assert args.host == "0.0.0.0"
    assert args.port == 8090
    assert args.fps == 30
    assert args.device == "cuda"
    assert args.log_level == "INFO"


def test_cli_parser_custom_args():
    parser = build_parser()
    args = parser.parse_args([
        "--host", "192.168.1.100",
        "--port", "5555",
        "--fps", "50",
        "--device", "cpu",
        "--log-level", "DEBUG",
    ])

    assert args.host == "192.168.1.100"
    assert args.port == 5555
    assert args.fps == 50
    assert args.device == "cpu"
    assert args.log_level == "DEBUG"
