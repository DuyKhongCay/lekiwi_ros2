# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import SwitchController
from lekiwi_interfaces.srv import SetTorqueEnabled


class RobotArmManager:
    """Manages arm controllers and motor torque for lead-through hand-eye calibration."""

    def __init__(
        self,
        node: Node,
        switch_controller_service="/controller_manager/switch_controller",
        set_torque_service="/set_torque_enabled",
        arm_controllers=None,
        callback_group=None,
    ):
        self.node = node
        self.switch_srv_name = switch_controller_service
        self.torque_srv_name = set_torque_service
        self.arm_controllers = arm_controllers or [
            "arm_trajectory_controller",
            "arm_forward_controller",
        ]
        self.cbg = callback_group or rclpy.callback_groups.ReentrantCallbackGroup()

        self._switch_client = self.node.create_client(
            SwitchController, self.switch_srv_name, callback_group=self.cbg
        )
        self._torque_client = self.node.create_client(
            SetTorqueEnabled, self.torque_srv_name, callback_group=self.cbg
        )
        self.torque_is_enabled = True

    def disable_arm_for_manual_leadthrough(self, timeout_sec=2.0) -> tuple[bool, str]:
        """
        1. Deactivates arm controllers in controller_manager (BEST_EFFORT).
        2. Disables torque on arm servos so user can easily guide arm by hand.
        """
        self.node.get_logger().info(
            f"Disabling arm controllers and torque (Service: {self.torque_srv_name})..."
        )

        # Step 1: Switch controller
        if self._switch_client.wait_for_service(timeout_sec=timeout_sec):
            req = SwitchController.Request()
            req.deactivate_controllers = self.arm_controllers
            req.strictness = SwitchController.Request.BEST_EFFORT
            req.activate_asap = False

            future_s = self._switch_client.call_async(req)

            def _on_switch_done(f):
                try:
                    res = f.result()
                    self.node.get_logger().info(f"SwitchController result: {res.ok}")
                except Exception as e:
                    self.node.get_logger().error(f"SwitchController error: {e}")

            future_s.add_done_callback(_on_switch_done)
        else:
            self.node.get_logger().warn(
                f"Service {self.switch_srv_name} not available, proceeding to torque."
            )

        # Step 2: SetTorqueEnabled
        if not self._torque_client.wait_for_service(timeout_sec=timeout_sec):
            msg = f"Service {self.torque_srv_name} not available! Torque NOT changed."
            self.node.get_logger().error(msg)
            return False, msg

        req_t = SetTorqueEnabled.Request()
        req_t.target = SetTorqueEnabled.Request.TARGET_ARM
        req_t.enabled = False

        future_t = self._torque_client.call_async(req_t)

        def _on_torque_done(f):
            try:
                res = f.result()
                if res.success:
                    self.torque_is_enabled = False
                    self.node.get_logger().info(f"✅ Torque response: {res.message}")
                else:
                    self.node.get_logger().error(
                        f"❌ Torque failed on hardware: {res.message}"
                    )
            except Exception as e:
                self.node.get_logger().error(f"Torque call exception: {e}")

        future_t.add_done_callback(_on_torque_done)

        return True, "Arm deactivate & torque disable requested"

    def enable_arm_torque(self, timeout_sec=2.0) -> tuple[bool, str]:
        """Re-enables arm servo torque."""
        if not self._torque_client.wait_for_service(timeout_sec=timeout_sec):
            msg = f"Service {self.torque_srv_name} not available"
            self.node.get_logger().error(msg)
            return False, msg

        req_t = SetTorqueEnabled.Request()
        req_t.target = SetTorqueEnabled.Request.TARGET_ARM
        req_t.enabled = True

        future_t = self._torque_client.call_async(req_t)

        def _on_torque_done(f):
            try:
                res = f.result()
                if res.success:
                    self.torque_is_enabled = True
                    self.node.get_logger().info(f"✅ Torque response: {res.message}")
                else:
                    self.node.get_logger().error(f"❌ Torque failed: {res.message}")
            except Exception as e:
                self.node.get_logger().error(f"Torque call exception: {e}")

        future_t.add_done_callback(_on_torque_done)

        return True, "Arm torque enable requested"
