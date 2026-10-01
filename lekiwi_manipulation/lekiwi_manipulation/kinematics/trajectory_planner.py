"""Quintic Spline Trajectory Planner for LeKiwi Arm with C2 continuity and Limit Validation.

Implements closed-form 5th-order polynomial interpolation:
s(u) = 6*u^5 - 15*u^4 + 10*u^3
s'(u) = 30*u^4 - 60*u^3 + 30*u^2
s''(u) = 120*u^3 - 180*u^2 + 60*u

Guarantees:
- Zero start and end velocity (v0 = vf = 0)
- Zero start and end acceleration (a0 = af = 0)
- Fail-fast validation against hardware calibration limits before point generation
"""

import math
from typing import Dict, List, Optional, Sequence, Union

from builtin_interfaces.msg import Duration as RosDuration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from lekiwi_manipulation.common.arm_constants import (
    ARM_JOINTS,
    NAMED_POSES,
    validate_joint_limits,
)


class QuinticTrajectoryPlanner:
    """Generates C2-continuous smooth trajectories for joint-space arm motions."""

    def __init__(
        self,
        joint_names: Optional[List[str]] = None,
        default_rate_hz: float = 50.0,
        enforce_limits: bool = True,
    ) -> None:
        self.joint_names: List[str] = joint_names if joint_names is not None else list(ARM_JOINTS)
        self.default_rate_hz: float = default_rate_hz
        self.enforce_limits: bool = enforce_limits

    def plan_trajectory(
        self,
        start_positions: Union[Sequence[float], Dict[str, float]],
        target_positions: Union[Sequence[float], Dict[str, float]],
        duration_sec: float,
        rate_hz: Optional[float] = None,
    ) -> JointTrajectory:
        """Plan a smooth C2 trajectory from start to target.

        Args:
            start_positions: Current joint values (list or dict).
            target_positions: Goal joint values (list or dict).
            duration_sec: Execution duration in seconds (> 0.0).
            rate_hz: Sampling rate in Hz (default: self.default_rate_hz).

        Returns:
            trajectory_msgs.msg.JointTrajectory

        Raises:
            ValueError: If duration is invalid or joint targets violate hardware limits.
        """
        if duration_sec <= 0.0:
            raise ValueError(f"Trajectory duration must be positive, got {duration_sec}s.")

        dt = 1.0 / (rate_hz if rate_hz is not None else self.default_rate_hz)
        num_points = max(2, int(math.ceil(duration_sec / dt)) + 1)

        # Normalize start and target positions into ordered lists matching self.joint_names
        q_start = self._normalize_positions(start_positions)
        q_target = self._normalize_positions(target_positions)

        # L1 Safety Barrier: Validate target limits before generating points
        if self.enforce_limits:
            target_dict = {self.joint_names[i]: q_target[i] for i in range(len(self.joint_names))}
            is_valid, err_msg = validate_joint_limits(target_dict)
            if not is_valid:
                raise ValueError(f"Trajectory planning rejected: {err_msg}")

        traj_msg = JointTrajectory()
        traj_msg.joint_names = list(self.joint_names)

        num_joints = len(self.joint_names)
        delta_q = [q_target[i] - q_start[i] for i in range(num_joints)]

        for step in range(num_points):
            t = min(step * dt, duration_sec)
            u = t / duration_sec

            # Quintic polynomial profile: s(0) = 0, s(1) = 1
            u2 = u * u
            u3 = u2 * u
            u4 = u3 * u
            u5 = u4 * u

            s = 6.0 * u5 - 15.0 * u4 + 10.0 * u3
            ds_du = 30.0 * u4 - 60.0 * u3 + 30.0 * u2
            d2s_du2 = 120.0 * u3 - 180.0 * u2 + 60.0 * u

            # Scaling derivatives by duration
            ds_dt = ds_du / duration_sec
            d2s_dt2 = d2s_du2 / (duration_sec * duration_sec)

            point = JointTrajectoryPoint()
            point.positions = [q_start[i] + s * delta_q[i] for i in range(num_joints)]
            point.velocities = [ds_dt * delta_q[i] for i in range(num_joints)]
            point.accelerations = [d2s_dt2 * delta_q[i] for i in range(num_joints)]

            # Precise duration breakdown
            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            point.time_from_start = RosDuration(sec=sec, nanosec=nanosec)

            traj_msg.points.append(point)

        # Ensure exact end condition at the final point
        last_point = traj_msg.points[-1]
        last_point.positions = list(q_target)
        last_point.velocities = [0.0] * num_joints
        last_point.accelerations = [0.0] * num_joints
        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)
        last_point.time_from_start = RosDuration(sec=sec, nanosec=nanosec)

        return traj_msg

    def _normalize_positions(
        self, positions: Union[Sequence[float], Dict[str, float]]
    ) -> List[float]:
        """Convert input array or dictionary to an ordered list matching joint_names."""
        if isinstance(positions, dict):
            normalized = []
            for jn in self.joint_names:
                if jn not in positions:
                    raise ValueError(f"Missing joint '{jn}' in provided positions dict.")
                normalized.append(float(positions[jn]))
            return normalized

        pos_list = list(positions)
        if len(pos_list) != len(self.joint_names):
            raise ValueError(
                f"Expected {len(self.joint_names)} joint positions, got {len(pos_list)}."
            )
        return [float(p) for p in pos_list]


def create_named_pose_trajectory(
    start_positions: Sequence[float],
    pose_name: str,
    duration_sec: float = 2.0,
    rate_hz: float = 50.0,
) -> JointTrajectory:
    """Utility helper to generate a trajectory targeting a predefined named pose."""
    if pose_name not in NAMED_POSES:
        raise KeyError(
            f"Unknown named pose '{pose_name}'. Available: {list(NAMED_POSES.keys())}"
        )

    planner = QuinticTrajectoryPlanner()
    target = NAMED_POSES[pose_name]
    return planner.plan_trajectory(
        start_positions=start_positions,
        target_positions=target,
        duration_sec=duration_sec,
        rate_hz=rate_hz,
    )
