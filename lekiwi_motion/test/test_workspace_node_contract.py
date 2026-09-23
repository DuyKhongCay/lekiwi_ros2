# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Behavior Contract 3: Validate WorkspaceCheckerNode ROS 2 Service & TF interface."""

import math
import os
from pathlib import Path
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from composition_interfaces.srv import LoadNode, UnloadNode
from geometry_msgs.msg import TransformStamped
from lekiwi_interfaces.srv import CheckMoveFeasibility
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
import yaml


def test_workspace_node_service_contract():
    """Verify node accepts URDF and computes feasible base poses over ROS 2 interfaces."""
    xml = Path(os.environ["WORKSPACE_URDF_PATH"]).read_text()
    config = yaml.safe_load(Path(os.environ["CONTROL_CONFIG_PATH"]).read_text())
    parameters = config["workspace_checker"]["ros__parameters"]

    executable = (
        Path(get_package_prefix("rclcpp_components"))
        / "lib/rclcpp_components/component_container_mt"
    )
    process = subprocess.Popen(
        [str(executable), "--ros-args", "-r", "__node:=workspace_contract_container"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    rclpy.init()
    node = rclpy.create_node("workspace_contract_client")

    retained = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    description_pub = node.create_publisher(String, "/robot_description", retained)
    tf_pub = node.create_publisher(TFMessage, "/tf", 100)
    loader = node.create_client(
        LoadNode, "/workspace_contract_container/_container/load_node"
    )
    unloader = node.create_client(
        UnloadNode, "/workspace_contract_container/_container/unload_node"
    )
    client = node.create_client(
        CheckMoveFeasibility, "/workspace/check_move_feasibility"
    )

    def publish_tf():
        stamp = node.get_clock().now().to_msg()
        board = TransformStamped()
        board.header.stamp = stamp
        board.header.frame_id = "map"
        board.child_frame_id = "chessboard_frame"
        board.transform.rotation.w = 1.0

        base = TransformStamped()
        base.header.stamp = stamp
        base.header.frame_id = "chessboard_frame"
        base.child_frame_id = "base_footprint"
        base.transform.translation.z = -0.004
        base.transform.rotation.w = 1.0

        tf_pub.publish(TFMessage(transforms=[board, base]))

    def wait(future, timeout=8.0):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            publish_tf()
            rclpy.spin_once(node, timeout_sec=0.02)
        assert future.done(), "Operation timed out"
        return future.result()

    loaded_id = None
    try:
        assert loader.wait_for_service(timeout_sec=10.0)

        # 1. Load component plugin
        req = LoadNode.Request()
        req.package_name = "lekiwi_motion"
        req.plugin_name = "lekiwi_motion::WorkspaceCheckerNode"
        req.node_name = "workspace_checker"
        req.parameters = [
            Parameter(k, value=v).to_parameter_msg() for k, v in parameters.items()
        ]
        res = wait(loader.call_async(req))
        assert res.success, res.error_message
        loaded_id = res.unique_id

        # 2. Publish URDF
        description_pub.publish(String(data=xml))

        # 3. Call CheckMoveFeasibility service for reachable chess move
        assert client.wait_for_service(timeout_sec=5.0)

        move_req = CheckMoveFeasibility.Request()
        move_req.uci_move = "e2e4"

        deadline = time.monotonic() + 8.0
        response = None
        while time.monotonic() < deadline:
            response = wait(client.call_async(move_req), timeout=1.0)
            if response and response.feasible:
                break
            time.sleep(0.1)

        assert (
            response is not None and response.feasible
        ), f"Service failed: {response.message if response else 'None'}"
        assert response.pick_base_pose.header.frame_id == "map"
        assert response.place_base_pose.header.frame_id == "map"

        # 4. Call CheckMoveFeasibility with another uci_move string
        uci_req = CheckMoveFeasibility.Request()
        uci_req.uci_move = "d2d4"
        uci_resp = wait(client.call_async(uci_req), timeout=2.0)
        assert uci_resp is not None, "UCI move request timed out"
        assert uci_resp.feasible, f"UCI move d2d4 failed: {uci_resp.message}"
        assert uci_resp.pick_base_pose.header.frame_id == "map"
        assert uci_resp.place_base_pose.header.frame_id == "map"

        # 5. Poka-yoke validation: empty uci_move must fail fast
        bad_req = CheckMoveFeasibility.Request()
        bad_req.uci_move = ""
        bad_resp = wait(client.call_async(bad_req), timeout=2.0)
        assert bad_resp is not None
        assert not bad_resp.feasible
        assert "Malformed request" in bad_resp.message

        # 6. Capture move test: "e4d5" with is_capture=True
        cap_req = CheckMoveFeasibility.Request()
        cap_req.uci_move = "e4d5"
        cap_req.is_capture = True
        cap_resp = wait(client.call_async(cap_req), timeout=2.0)
        assert cap_resp is not None
        assert cap_resp.feasible
        assert cap_resp.pick_base_pose.header.frame_id == "map"
        assert cap_resp.place_base_pose.header.frame_id == "map"

    finally:
        if loaded_id is not None:
            unloader.call_async(UnloadNode.Request(unique_id=loaded_id))
        node.destroy_node()
        rclpy.shutdown()
        process.terminate()
        process.wait(timeout=5.0)
