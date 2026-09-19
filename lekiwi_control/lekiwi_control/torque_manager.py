# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""
Torque Manager Node for LeKiwi.

Provides the service `/set_torque_enabled` (compatible with `lekiwi_interfaces/srv/SetTorqueEnabled`)
and bridges commands to ros2_control's `robot_torque_controller` via the native `torque_enable`
command interface.
"""

from __future__ import annotations

import sys
from typing import List, Sequence

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray
from lekiwi_interfaces.srv import SetTorqueEnabled

DEFAULT_ARM_JOINTS: Sequence[str] = (
    "arm_shoulder_pan",
    "arm_shoulder_lift",
    "arm_elbow_flex",
    "arm_wrist_flex",
    "arm_wrist_roll",
    "arm_gripper",
)

DEFAULT_BASE_JOINTS: Sequence[str] = (
    "base_left_wheel",
    "base_back_wheel",
    "base_right_wheel",
)


class TorqueManagerNode(Node):
    """
    ROS 2 Service adapter for LeKiwi motor torque management.

    Decouples robot-level domain semantics (ARM vs BASE torque control) from
    the low-level hardware interface by translating `SetTorqueEnabled` service
    requests into individual joint commands published to `robot_torque_controller`.
    """

    def __init__(self) -> None:
        super().__init__("torque_manager")

        self.declare_parameter("arm_joints", list(DEFAULT_ARM_JOINTS))
        self.declare_parameter("base_joints", list(DEFAULT_BASE_JOINTS))
        self.declare_parameter(
            "torque_controller_topic", "/robot_torque_controller/commands"
        )

        arm_joints: List[str] = list(self.get_parameter("arm_joints").value)
        base_joints: List[str] = list(self.get_parameter("base_joints").value)
        self.joint_names: List[str] = arm_joints + base_joints

        self.num_arm = len(arm_joints)
        self.num_base = len(base_joints)
        self.total_joints = len(self.joint_names)

        # Track internal state for all joints: 1.0 = enabled, 0.0 = disabled
        self._current_torque_states: List[float] = [1.0] * self.total_joints

        cmd_topic = str(self.get_parameter("torque_controller_topic").value)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._cmd_pub = self.create_publisher(Float64MultiArray, cmd_topic, qos)

        self._srv = self.create_service(
            SetTorqueEnabled,
            "set_torque_enabled",
            self._handle_set_torque_enabled,
        )

        # Publish initial all-enabled state
        self._publish_torque_command()
        self.get_logger().info(
            f"TorqueManager initialized for {self.total_joints} joints "
            f"({self.num_arm} arm, {self.num_base} base). Service: 'set_torque_enabled'"
        )

    def _handle_set_torque_enabled(
        self,
        request: SetTorqueEnabled.Request,
        response: SetTorqueEnabled.Response,
    ) -> SetTorqueEnabled.Response:
        target = request.target
        enabled = request.enabled
        val = 1.0 if enabled else 0.0

        if target == SetTorqueEnabled.Request.TARGET_ALL:
            for i in range(self.total_joints):
                self._current_torque_states[i] = val
            target_str = "All"
        elif target == SetTorqueEnabled.Request.TARGET_ARM:
            for i in range(self.num_arm):
                self._current_torque_states[i] = val
            target_str = "Arm"
        elif target == SetTorqueEnabled.Request.TARGET_BASE:
            for i in range(self.num_arm, self.total_joints):
                self._current_torque_states[i] = val
            target_str = "Base"
        else:
            response.success = False
            response.message = f"Invalid target: {target} (use 0=ALL, 1=ARM, 2=BASE)"
            self.get_logger().warn(response.message)
            return response

        self._publish_torque_command()
        response.success = True
        state_str = "enabled" if enabled else "disabled"
        response.message = f"{target_str} torque {state_str} successfully"
        self.get_logger().info(response.message)
        return response

    def _publish_torque_command(self) -> None:
        msg = Float64MultiArray()
        msg.data = list(self._current_torque_states)
        self._cmd_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TorqueManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
