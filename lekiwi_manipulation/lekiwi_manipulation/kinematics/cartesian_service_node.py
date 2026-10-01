"""Cartesian and Joint-space Service Node for LeKiwi Arm.

Provides high-level ROS 2 services and topics to command the arm smoothly
to named poses (home, stow, ready, observe) or target joint configurations,
delegating actual trajectory execution to arm_trajectory_controller.
"""

from typing import Dict, List, Optional
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory

from lekiwi_manipulation.common.arm_constants import (
    ARM_JOINTS,
    GRIPPER_CLOSED,
    GRIPPER_OPEN,
    NAMED_POSES,
    validate_joint_limits,
)
from lekiwi_manipulation.kinematics.trajectory_planner import (
    QuinticTrajectoryPlanner,
    create_named_pose_trajectory,
)


class CartesianServiceNode(Node):
    """ROS 2 Node providing named pose services and safe joint motion dispatch."""

    def __init__(self, node_name: str = "cartesian_service_node") -> None:
        super().__init__(node_name)

        # Declare parameters
        self.declare_parameter(
            "action_server_name", "/arm_trajectory_controller/follow_joint_trajectory"
        )
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("default_duration_sec", 2.5)
        self.declare_parameter("sampling_rate_hz", 50.0)

        self.action_server_name: str = (
            self.get_parameter("action_server_name").get_parameter_value().string_value
        )
        self.joint_states_topic: str = (
            self.get_parameter("joint_states_topic").get_parameter_value().string_value
        )
        self.default_duration_sec: float = (
            self.get_parameter("default_duration_sec").get_parameter_value().double_value
        )
        self.sampling_rate_hz: float = (
            self.get_parameter("sampling_rate_hz").get_parameter_value().double_value
        )

        # Threading / callback groups
        self.cb_group = ReentrantCallbackGroup()
        self.client_cb_group = MutuallyExclusiveCallbackGroup()

        # FollowJointTrajectory Action Client
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.action_server_name,
            callback_group=self.client_cb_group,
        )

        # State tracking
        self._current_joints: Dict[str, float] = {}
        self._has_joint_feedback: bool = False

        # Subscriber for joint states
        sensor_qos = QoSProfile(depth=1)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self._joint_state_sub = self.create_subscription(
            JointState,
            self.joint_states_topic,
            self._on_joint_state,
            sensor_qos,
            callback_group=self.cb_group,
        )

        # Trajectory planner instance
        self._planner = QuinticTrajectoryPlanner(
            joint_names=ARM_JOINTS,
            default_rate_hz=self.sampling_rate_hz,
            enforce_limits=True,
        )

        # Service servers for named poses
        self._srv_home = self.create_service(
            Trigger, "~/go_to_home", self._handle_go_to_home, callback_group=self.cb_group
        )
        self._srv_stow = self.create_service(
            Trigger, "~/go_to_stow", self._handle_go_to_stow, callback_group=self.cb_group
        )
        self._srv_ready = self.create_service(
            Trigger, "~/go_to_ready", self._handle_go_to_ready, callback_group=self.cb_group
        )
        self._srv_observe = self.create_service(
            Trigger,
            "~/go_to_observe",
            self._handle_go_to_observe,
            callback_group=self.cb_group,
        )
        self._srv_gripper = self.create_service(
            SetBool, "~/set_gripper", self._handle_set_gripper, callback_group=self.cb_group
        )

        # Topic subscribers for direct commands
        self._named_pose_sub = self.create_subscription(
            String,
            "~/command_named_pose",
            self._on_command_named_pose,
            10,
            callback_group=self.cb_group,
        )
        self._joint_cmd_sub = self.create_subscription(
            JointState,
            "~/command_joint_state",
            self._on_command_joint_state,
            10,
            callback_group=self.cb_group,
        )

        self.get_logger().info(
            f"CartesianServiceNode initialized. Action target: {self.action_server_name}"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        """Cache latest known joint positions for planning starting points."""
        for name, pos in zip(msg.name, msg.position):
            if name in ARM_JOINTS:
                self._current_joints[name] = pos
        if all(j in self._current_joints for j in ARM_JOINTS):
            self._has_joint_feedback = True

    def _get_current_positions_or_default(self) -> List[float]:
        """Return current arm joint positions, or zero pose if no feedback yet."""
        if not self._has_joint_feedback:
            self.get_logger().warn(
                "Joint states feedback not received yet. Assuming current positions at zero."
            )
            return [0.0] * len(ARM_JOINTS)
        return [self._current_joints[j] for j in ARM_JOINTS]

    async def _execute_trajectory(
        self, traj_msg: JointTrajectory
    ) -> tuple[bool, str]:
        """Send trajectory goal to arm_trajectory_controller action server."""
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            err = f"Action server '{self.action_server_name}' not available."
            self.get_logger().error(err)
            return False, err

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj_msg

        self.get_logger().info("Sending trajectory goal to controller...")
        send_goal_future = await self._action_client.send_goal_async(goal_msg)
        if not send_goal_future.accepted:
            err = "Trajectory goal was rejected by arm_trajectory_controller."
            self.get_logger().warn(err)
            return False, err

        result = await send_goal_future.get_result_async()
        if result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            msg = "Trajectory motion succeeded."
            self.get_logger().info(msg)
            return True, msg
        else:
            err = (
                f"Trajectory motion failed with error_code={result.result.error_code}: "
                f"{result.result.error_string}"
            )
            self.get_logger().error(err)
            return False, err

    async def _execute_named_pose(
        self, pose_name: str, duration_sec: Optional[float] = None
    ) -> tuple[bool, str]:
        """Plan and execute a motion to a named pose."""
        if pose_name not in NAMED_POSES:
            return False, f"Unknown named pose '{pose_name}'."

        start_positions = self._get_current_positions_or_default()
        duration = duration_sec or self.default_duration_sec

        try:
            traj = create_named_pose_trajectory(
                start_positions=start_positions,
                pose_name=pose_name,
                duration_sec=duration,
                rate_hz=self.sampling_rate_hz,
            )
        except Exception as exc:
            return False, f"Failed to plan trajectory for '{pose_name}': {exc}"

        return await self._execute_trajectory(traj)

    async def _handle_go_to_home(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        ok, msg = await self._execute_named_pose("home")
        response.success = ok
        response.message = msg
        return response

    async def _handle_go_to_stow(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        ok, msg = await self._execute_named_pose("stow")
        response.success = ok
        response.message = msg
        return response

    async def _handle_go_to_ready(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        ok, msg = await self._execute_named_pose("ready")
        response.success = ok
        response.message = msg
        return response

    async def _handle_go_to_observe(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        ok, msg = await self._execute_named_pose("observe")
        response.success = ok
        response.message = msg
        return response

    async def _handle_set_gripper(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        target_gripper = GRIPPER_OPEN if request.data else GRIPPER_CLOSED
        start_positions = self._get_current_positions_or_default()
        target_positions = list(start_positions)
        # Gripper is the last joint (index 5)
        target_positions[-1] = target_gripper

        try:
            traj = self._planner.plan_trajectory(
                start_positions=start_positions,
                target_positions=target_positions,
                duration_sec=1.0,
                rate_hz=self.sampling_rate_hz,
            )
        except Exception as exc:
            response.success = False
            response.message = f"Gripper motion rejected: {exc}"
            return response

        ok, msg = await self._execute_trajectory(traj)
        response.success = ok
        response.message = msg
        return response

    async def _on_command_named_pose(self, msg: String) -> None:
        pose_name = msg.data.strip().lower()
        ok, res_msg = await self._execute_named_pose(pose_name)
        if not ok:
            self.get_logger().warn(f"Command named pose '{pose_name}' failed: {res_msg}")

    async def _on_command_joint_state(self, msg: JointState) -> None:
        start_positions = self._get_current_positions_or_default()
        target_dict: Dict[str, float] = {
            ARM_JOINTS[i]: start_positions[i] for i in range(len(ARM_JOINTS))
        }

        for name, pos in zip(msg.name, msg.position):
            if name in target_dict:
                target_dict[name] = pos

        is_valid, err = validate_joint_limits(target_dict)
        if not is_valid:
            self.get_logger().error(f"Command joint state rejected: {err}")
            return

        target_positions = [target_dict[j] for j in ARM_JOINTS]
        try:
            traj = self._planner.plan_trajectory(
                start_positions=start_positions,
                target_positions=target_positions,
                duration_sec=self.default_duration_sec,
                rate_hz=self.sampling_rate_hz,
            )
        except Exception as exc:
            self.get_logger().error(f"Planning error: {exc}")
            return

        await self._execute_trajectory(traj)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = CartesianServiceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
