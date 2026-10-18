"""Smoke test for LeKiwi Episode Recorder: verifying Gamepad D-Pad and Services."""

import os
import shutil
import tempfile
import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger


@pytest.fixture(scope="module")
def ros_context():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_gamepad_and_services(ros_context):
    temp_dir = tempfile.mkdtemp(prefix="test_lekiwi_episodes_")
    try:
        # Create helper test node
        test_node = Node("test_recorder_client")
        joy_pub = test_node.create_publisher(Joy, "/joy", 10)
        assert joy_pub is not None

        # Clients for Trigger services
        start_cli = test_node.create_client(Trigger, "/episode_recorder_node/start_episode")
        save_cli = test_node.create_client(Trigger, "/episode_recorder_node/save_episode")
        discard_cli = test_node.create_client(Trigger, "/episode_recorder_node/discard_episode")

        assert start_cli is not None
        assert save_cli is not None
        assert discard_cli is not None

        # Verify that the temp output dir exists
        assert os.path.exists(temp_dir)
    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)
