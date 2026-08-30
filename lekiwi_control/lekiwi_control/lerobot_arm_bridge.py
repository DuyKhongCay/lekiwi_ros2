"""Bridge LeRobot arm messages to FollowJointTrajectory using joint YAML."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math

import yaml
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


RAW_POSITION_SPAN = 4095.0
RADIANS_PER_RAW = 2.0 * math.pi / RAW_POSITION_SPAN
ARM_JOINTS = (
    'arm_shoulder_pan',
    'arm_shoulder_lift',
    'arm_elbow_flex',
    'arm_wrist_flex',
    'arm_wrist_roll',
    'arm_gripper',
)


# Keeps raw-position conversion independent of the removed description package.
@dataclass(frozen=True)
class JointCalibration:
    """Stores the raw midpoint used by LeRobot for one arm joint."""

    midpoint: float


# Loads only the calibration data that the arm bridge needs at runtime.
def load_joint_config(joint_config_file: str) -> dict[str, JointCalibration]:
    """Validate the YAML arm entries and derive each raw-position midpoint."""
    path = Path(joint_config_file)
    try:
        document = yaml.safe_load(path.read_text(encoding='utf-8'))
    except OSError as exc:
        raise ValueError(f'Cannot read joint_config_file {path}: {exc}') from exc
    except yaml.YAMLError as exc:
        raise ValueError(f'Invalid joint YAML in {path}: {exc}') from exc

    joints = document.get('joints') if isinstance(document, dict) else None
    if not isinstance(joints, dict):
        raise ValueError("joint_config_file must contain a top-level 'joints' map")

    calibration: dict[str, JointCalibration] = {}
    for joint_name in ARM_JOINTS:
        entry = joints.get(joint_name)
        if not isinstance(entry, dict):
            raise ValueError(f"joint_config_file is missing arm joint '{joint_name}'")
        try:
            range_min = float(entry['range_min'])
            range_max = float(entry['range_max'])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(
                f"Joint '{joint_name}' requires numeric range_min and range_max"
            ) from exc
        if range_min >= range_max:
            raise ValueError(f"Joint '{joint_name}' must have range_min < range_max")
        calibration[joint_name] = JointCalibration(
            midpoint=(range_min + range_max) / 2.0
        )
    return calibration


class LeRobotArmBridge(Node):
    """Translate named LeRobot arm observations/actions without owning a serial device."""

    # Initializes transport endpoints from the YAML-backed calibration contract.
    def __init__(self):
        super().__init__('lerobot_arm_bridge')
        self.declare_parameter('joint_config_file', '')
        self.declare_parameter('action_topic', '/lerobot/arm_action')
        self.declare_parameter('observation_topic', '/lerobot/arm_observation')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('action_units', 'raw')
        self.declare_parameter('trajectory_duration_sec', 0.25)
        joint_config_file = self.get_parameter('joint_config_file').value
        self._calibration = (
            load_joint_config(joint_config_file)
            if joint_config_file
            else {name: JointCalibration(midpoint=2048.0) for name in ARM_JOINTS}
        )
        self._arm_names = ARM_JOINTS
        self._units = self.get_parameter('action_units').value
        if self._units not in {'raw', 'radians', 'degrees'}:
            raise ValueError('action_units must be raw, radians, or degrees')
        duration = float(self.get_parameter('trajectory_duration_sec').value)
        if duration <= 0.0:
            raise ValueError('trajectory_duration_sec must be positive')
        self._duration = Duration(seconds=duration).to_msg()
        action_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._trajectory_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory'
        )
        self._observation_pub = self.create_publisher(
            JointState, self.get_parameter('observation_topic').value, action_qos
        )
        self.create_subscription(
            JointState,
            self.get_parameter('action_topic').value,
            self._on_action,
            action_qos,
        )
        self.create_subscription(
            JointState,
            self.get_parameter('joint_states_topic').value,
            self._on_joint_states,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

    # Submits trajectories asynchronously so subscription callbacks cannot block the executor.
    def _on_action(self, message: JointState):
        """Convert a complete named LeRobot action into one trajectory goal."""
        values = self._named_values(message)
        if values is None:
            return
        if not self._trajectory_client.server_is_ready():
            self.get_logger().warning(
                'arm_controller FollowJointTrajectory server is unavailable'
            )
            return
        point = JointTrajectoryPoint()
        point.positions = [
            self._to_radians(name, values[name]) for name in self._arm_names
        ]
        point.time_from_start = self._duration
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self._arm_names)
        goal.trajectory.points = [point]
        future = self._trajectory_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    # Reports a rejected goal after completion without synchronously waiting for it.
    def _on_goal_response(self, future):
        """Report rejected trajectory goals without blocking a ROS callback."""
        try:
            if not future.result().accepted:
                self.get_logger().error('arm_controller rejected LeRobot trajectory goal')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Could not submit LeRobot trajectory goal: {exc}')

    # Republishes joint feedback in LeRobot raw coordinates using named joints only.
    def _on_joint_states(self, message: JointState):
        """Publish arm observations in raw calibration coordinates."""
        values = self._named_values(message)
        if values is None:
            return
        observation = JointState()
        observation.header = message.header
        observation.name = list(self._arm_names)
        observation.position = [
            self._calibration[name].midpoint + values[name] / RADIANS_PER_RAW
            for name in self._arm_names
        ]
        self._observation_pub.publish(observation)

    # Rejects partial data because JointState ordering is publisher-defined.
    def _named_values(self, message: JointState):
        """Return every required arm value by name rather than by array index."""
        if len(message.name) != len(message.position):
            self.get_logger().error('JointState name and position arrays have different sizes')
            return None
        values = dict(zip(message.name, message.position))
        missing = [name for name in self._arm_names if name not in values]
        if missing:
            self.get_logger().warning(f'Ignoring partial arm message; missing {missing}')
            return None
        return values

    # Normalizes the three supported incoming action representations into radians.
    def _to_radians(self, joint_name: str, value: float) -> float:
        """Convert the configured LeRobot action representation to ROS radians."""
        if self._units == 'radians':
            return value
        if self._units == 'degrees':
            return value * math.pi / 180.0
        return (value - self._calibration[joint_name].midpoint) * RADIANS_PER_RAW


# Keeps the bridge usable as a normal standalone ROS executable.
def main(args=None):
    """Run the bridge as a standalone ROS process."""
    import rclpy

    rclpy.init(args=args)
    node = LeRobotArmBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
