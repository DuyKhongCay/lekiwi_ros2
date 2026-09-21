#!/usr/bin/env python3
"""
Test script for LeKiwi arm MoveIt motion execution.
Plans and executes motions to Named States and Cartesian Poses.
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    Constraints,
    JointConstraint,
    PositionConstraint,
    BoundingVolume,
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped


class LeKiwiMoveitDemoNode(Node):
    """Demo client for testing MoveIt planning and execution on LeKiwi arm."""

    def __init__(self):
        super().__init__("lekiwi_moveit_demo")
        self.get_logger().info("Initializing LeKiwi MoveIt Demo Client...")

        self.move_group_client = ActionClient(self, MoveGroup, "move_action")
        self.get_logger().info("Waiting for MoveGroup action server...")
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveGroup action server not available!")
            return

        self.get_logger().info("Connected to MoveGroup action server.")

    def plan_and_execute_named_state(self, named_state: str, target_joints: dict):
        """Send joint goal to MoveGroup to reach target named state."""
        self.get_logger().info(f"Planning motion to target: {named_state}")

        req = MotionPlanRequest()
        req.group_name = "arm"
        req.num_planning_attempts = 5
        req.allowed_planning_time = 3.0
        req.max_velocity_scaling_factor = 0.5
        req.max_acceleration_scaling_factor = 0.5

        # Joint constraints
        constraints = Constraints()
        for joint_name, pos in target_joints.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = pos
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        req.goal_constraints.append(constraints)

        goal_msg = MoveGroup.Goal()
        goal_msg.request = req
        goal_msg.planning_options.plan_only = False

        self.get_logger().info(f"Sending goal for {named_state}...")
        future = self.move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f"Goal for {named_state} rejected!")
            return False

        self.get_logger().info(f"Goal accepted, executing trajectory...")
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        result = res_future.result().result

        if result.error_code.val == 1:  # SUCCESS = 1
            self.get_logger().info(f"Successfully reached {named_state}!")
            return True
        else:
            self.get_logger().error(f"Motion execution failed with error code: {result.error_code.val}")
            return False


def main(args=None):
    """Main execution sequence."""
    rclpy.init(args=args)
    node = LeKiwiMoveitDemoNode()

    # Named states joints targets
    stow_pose = {
        "arm_shoulder_pan": 0.0,
        "arm_shoulder_lift": -1.57,
        "arm_elbow_flex": 1.57,
        "arm_wrist_flex": 0.75,
        "arm_wrist_roll": 0.0,
    }

    home_pose = {
        "arm_shoulder_pan": 0.0,
        "arm_shoulder_lift": -1.57,
        "arm_elbow_flex": 1.57,
        "arm_wrist_flex": 0.0,
        "arm_wrist_roll": 0.0,
    }

    try:
        # Move to home
        node.plan_and_execute_named_state("home", home_pose)
        time.sleep(1.0)

        # Move to stow
        node.plan_and_execute_named_state("stow", stow_pose)
        time.sleep(1.0)

        # Move back to home
        node.plan_and_execute_named_state("home", home_pose)

    except Exception as e:
        node.get_logger().error(f"Exception during demo: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
