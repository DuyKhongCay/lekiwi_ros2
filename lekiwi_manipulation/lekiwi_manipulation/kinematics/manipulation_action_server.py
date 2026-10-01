"""Real Hardware Manipulation Action Server for LeKiwi Arm.

Executes multi-phase pick-and-place manipulation (e.g., chess moves)
by coordinating trajectory planning, limit verification, and
action goals to ros2_control's arm_trajectory_controller.
"""

from __future__ import annotations

import time
from typing import Dict, List, Optional

from control_msgs.action import FollowJointTrajectory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from lekiwi_interfaces.action import ExecuteChessMove
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from lekiwi_manipulation.common.arm_constants import (
    ARM_JOINTS,
    GRIPPER_CLOSED,
    GRIPPER_OPEN,
    NAMED_POSES,
    validate_joint_limits,
)
from lekiwi_manipulation.kinematics.trajectory_planner import QuinticTrajectoryPlanner


class ManipulationActionServer(Node):
    """Executes multi-phase manipulation tasks on LeKiwi arm via arm_trajectory_controller."""

    PHASES: List[str] = [
        "APPROACH_PICK",
        "DESCEND_PICK",
        "GRASP",
        "LIFT",
        "APPROACH_PLACE",
        "DESCEND_PLACE",
        "RELEASE",
        "RETRACT",
    ]

    def __init__(self, node_name: str = "manipulation_action_server") -> None:
        super().__init__(node_name)

        # Declare parameters
        self.declare_parameter("action_name", "/manipulation/execute_chess_move")
        self.declare_parameter(
            "trajectory_controller_action",
            "/arm_trajectory_controller/follow_joint_trajectory",
        )
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("phase_duration_sec", 1.2)
        self.declare_parameter("sampling_rate_hz", 50.0)

        self.action_name: str = (
            self.get_parameter("action_name").get_parameter_value().string_value
        )
        self.controller_action_name: str = (
            self.get_parameter("trajectory_controller_action")
            .get_parameter_value()
            .string_value
        )
        self.joint_states_topic: str = (
            self.get_parameter("joint_states_topic").get_parameter_value().string_value
        )
        self.phase_duration: float = (
            self.get_parameter("phase_duration_sec").get_parameter_value().double_value
        )
        self.sampling_rate_hz: float = (
            self.get_parameter("sampling_rate_hz").get_parameter_value().double_value
        )

        # Callback groups
        self.cb_group = ReentrantCallbackGroup()
        self.client_cb_group = MutuallyExclusiveCallbackGroup()

        # Planner
        self._planner = QuinticTrajectoryPlanner(
            joint_names=ARM_JOINTS,
            default_rate_hz=self.sampling_rate_hz,
            enforce_limits=True,
        )

        # Feedback & Diagnostics
        self._current_joints: Dict[str, float] = {}
        self._joint_state_sub = self.create_subscription(
            JointState,
            self.joint_states_topic,
            self._on_joint_state,
            1,
            callback_group=self.cb_group,
        )
        self._diag_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10, callback_group=self.cb_group
        )

        # Action Client for controller
        self._controller_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.controller_action_name,
            callback_group=self.client_cb_group,
        )

        # Action Server for manipulation
        self._action_server = ActionServer(
            self,
            ExecuteChessMove,
            self.action_name,
            execute_callback=self._execute_goal,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info(
            f"ManipulationActionServer online at '{self.action_name}' -> '{self.controller_action_name}'"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            if name in ARM_JOINTS:
                self._current_joints[name] = pos

    def _goal_callback(self, goal_request: ExecuteChessMove.Goal) -> GoalResponse:
        self.get_logger().info(
            f"Received manipulation goal: '{goal_request.instruction}' "
            f"({goal_request.from_square} -> {goal_request.to_square})"
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        self.get_logger().warn("Manipulation cancel requested.")
        return CancelResponse.ACCEPT

    def _get_current_positions(self) -> List[float]:
        return [self._current_joints.get(j, 0.0) for j in ARM_JOINTS]

    async def _send_trajectory_and_wait(
        self, traj_msg: JointTrajectory
    ) -> tuple[bool, str]:
        if not self._controller_client.wait_for_server(timeout_sec=2.0):
            return False, f"Controller action server '{self.controller_action_name}' unavailable"

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj_msg

        send_future = await self._controller_client.send_goal_async(goal_msg)
        if not send_future.accepted:
            return False, "Trajectory rejected by arm_trajectory_controller"

        res = await send_future.get_result_async()
        if res.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            return True, "Success"
        return False, f"Controller failed (error_code={res.result.error_code})"

    def _extract_hint_or_fallback(
        self, hint: JointState, fallback_pose: List[float], gripper_state: float
    ) -> List[float]:
        """Extract valid joint positions from IK hint, or fallback to default safe pose."""
        positions = list(fallback_pose)
        if hint and len(hint.name) > 0:
            hint_dict = dict(zip(hint.name, hint.position))
            is_valid, _ = validate_joint_limits(hint_dict)
            if is_valid:
                for idx, jname in enumerate(ARM_JOINTS):
                    if jname in hint_dict:
                        positions[idx] = hint_dict[jname]
        # Explicit gripper setting
        positions[-1] = gripper_state
        return positions

    async def _execute_goal(
        self, goal_handle: ServerGoalHandle
    ) -> ExecuteChessMove.Result:
        start_time = time.monotonic()
        goal: ExecuteChessMove.Goal = goal_handle.request
        feedback = ExecuteChessMove.Feedback()
        result = ExecuteChessMove.Result()

        self.get_logger().info(f"Executing move: {goal.instruction}")

        # Compute key configurations for phases
        home_pose = NAMED_POSES["home"]
        ready_pose = NAMED_POSES["ready"]

        # Pick poses
        pick_approach = self._extract_hint_or_fallback(
            goal.pick_ik_hint, ready_pose, GRIPPER_OPEN
        )
        pick_descend = list(pick_approach)
        pick_descend[1] -= 0.15  # Shoulder down slightly to grasp
        pick_grasp = list(pick_descend)
        pick_grasp[-1] = GRIPPER_CLOSED

        # Lift pose
        pick_lift = list(pick_approach)
        pick_lift[-1] = GRIPPER_CLOSED

        # Place poses
        place_approach = self._extract_hint_or_fallback(
            goal.place_ik_hint, ready_pose, GRIPPER_CLOSED
        )
        place_descend = list(place_approach)
        place_descend[1] -= 0.15
        place_release = list(place_descend)
        place_release[-1] = GRIPPER_OPEN

        retract_pose = list(home_pose)
        retract_pose[-1] = GRIPPER_CLOSED

        phase_targets = {
            "APPROACH_PICK": pick_approach,
            "DESCEND_PICK": pick_descend,
            "GRASP": pick_grasp,
            "LIFT": pick_lift,
            "APPROACH_PLACE": place_approach,
            "DESCEND_PLACE": place_descend,
            "RELEASE": place_release,
            "RETRACT": retract_pose,
        }

        total_phases = len(self.PHASES)
        for idx, phase_name in enumerate(self.PHASES):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = f"Canceled during phase {phase_name}."
                result.execution_time_sec = float(time.monotonic() - start_time)
                return result

            feedback.current_phase = phase_name
            feedback.progress_percent = (idx / float(total_phases)) * 100.0
            goal_handle.publish_feedback(feedback)
            self._publish_diagnostic(phase_name, feedback.progress_percent)

            # Target for this phase
            target_pos = phase_targets[phase_name]
            current_pos = self._get_current_positions()

            # Plan smooth quintic transition
            try:
                traj = self._planner.plan_trajectory(
                    start_positions=current_pos,
                    target_positions=target_pos,
                    duration_sec=self.phase_duration,
                    rate_hz=self.sampling_rate_hz,
                )
            except Exception as exc:
                goal_handle.abort()
                result.success = False
                result.message = f"Planning failed in {phase_name}: {exc}"
                result.execution_time_sec = float(time.monotonic() - start_time)
                return result

            # Execute via arm_trajectory_controller
            ok, msg = await self._send_trajectory_and_wait(traj)
            if not ok:
                goal_handle.abort()
                result.success = False
                result.message = f"Execution failed in {phase_name}: {msg}"
                result.execution_time_sec = float(time.monotonic() - start_time)
                return result

        goal_handle.succeed()
        result.success = True
        result.message = f"Completed move: {goal.instruction}"
        result.execution_time_sec = float(time.monotonic() - start_time)
        self.get_logger().info(f"Move finished in {result.execution_time_sec:.2f}s")
        return result

    def _publish_diagnostic(self, phase: str, progress: float) -> None:
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "lekiwi_manipulation: action_server"
        status.level = DiagnosticStatus.OK
        status.message = f"Phase {phase}"
        status.values = [
            KeyValue(key="phase", value=phase),
            KeyValue(key="progress", value=f"{progress:.1f}%"),
        ]
        msg.status.append(status)
        self._diag_pub.publish(msg)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = ManipulationActionServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
