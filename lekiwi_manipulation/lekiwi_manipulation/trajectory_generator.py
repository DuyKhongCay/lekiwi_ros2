# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""
Strategy Pattern: Mathematical Trajectory Generation & Interpolation for LeKiwi Arm.

Provides smooth, C2-continuous (Zero-Jerk) quintic polynomial trajectories between
joint poses across discrete chess manipulation phases.
Pure mathematical module: 100% testable without ROS node dependencies.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from enum import Enum
import math
from typing import Dict, List, Sequence, Tuple

from builtin_interfaces.msg import Duration as RosDuration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS_DEFAULT: Sequence[str] = (
    "arm_shoulder_pan",
    "arm_shoulder_lift",
    "arm_elbow_flex",
    "arm_wrist_flex",
    "arm_wrist_roll",
    "arm_gripper",
)

# Canonical reference joint angles (radians)
DEFAULT_HOME_POSE: Dict[str, float] = {
    "arm_shoulder_pan": 0.0,
    "arm_shoulder_lift": -0.8,
    "arm_elbow_flex": -1.2,
    "arm_wrist_flex": 0.5,
    "arm_wrist_roll": 0.0,
    "arm_gripper": 0.0,  # Open
}

DEFAULT_STOW_POSE: Dict[str, float] = {
    "arm_shoulder_pan": 0.0,
    "arm_shoulder_lift": -1.5,
    "arm_elbow_flex": -1.5,
    "arm_wrist_flex": 0.0,
    "arm_wrist_roll": 0.0,
    "arm_gripper": 0.0,
}

# Chassis bin pose for captured opponent pieces (radians)
DEFAULT_BIN_DROP_POSE: Dict[str, float] = {
    "arm_shoulder_pan": 1.57,  # Rotated to robot left chassis
    "arm_shoulder_lift": -0.4,
    "arm_elbow_flex": -0.8,
    "arm_wrist_flex": -0.4,
    "arm_wrist_roll": 0.0,
    "arm_gripper": 0.0,
}

DEFAULT_GRIPPER_OPEN_RAD = 0.0
DEFAULT_GRIPPER_CLOSED_RAD = 0.65  # Gripping standard chess piece


class ManipulationPhase(str, Enum):
    """Discrete operational phases of a chess pick-and-place command."""

    IDLE = "IDLE"
    APPROACH_PICK = "APPROACH_PICK"
    DESCEND_PICK = "DESCEND_PICK"
    GRASP = "GRASP"
    LIFT = "LIFT"
    TRANSIT_PLACE = "TRANSIT_PLACE"
    DESCEND_PLACE = "DESCEND_PLACE"
    RELEASE = "RELEASE"
    RETRACT_STOW = "RETRACT_STOW"


class ITrajectoryStrategy(ABC):
    """Abstract Strategy interface for joint path interpolation."""

    @abstractmethod
    def interpolate_scalar(
        self, s0: float, s1: float, tau: float, duration: float
    ) -> Tuple[float, float, float]:
        """
        Interpolate position, velocity, and acceleration for normalized time tau in [0, 1].

        Returns
        -------
        Tuple[float, float, float]
            (position, velocity, acceleration)
        """
        pass


class QuinticSplineStrategy(ITrajectoryStrategy):
    """
    Quintic (5th order) polynomial interpolation ensuring C2-continuous Zero-Jerk motion.
    Boundary conditions: v(0)=v(T)=0, a(0)=a(T)=0.
    """

    def interpolate_scalar(
        self, s0: float, s1: float, tau: float, duration: float
    ) -> Tuple[float, float, float]:
        if duration <= 0.0:
            return s1, 0.0, 0.0

        # Clamp tau to [0, 1]
        t = max(0.0, min(1.0, float(tau)))
        diff = s1 - s0

        # Quintic polynomial: s(t) = s0 + (s1 - s0) * (10 t^3 - 15 t^4 + 6 t^5)
        pos = s0 + diff * (10.0 * (t**3) - 15.0 * (t**4) + 6.0 * (t**5))
        vel = (diff / duration) * (30.0 * (t**2) - 60.0 * (t**3) + 30.0 * (t**4))
        acc = (diff / (duration**2)) * (60.0 * t - 180.0 * (t**2) + 120.0 * (t**3))

        return pos, vel, acc


class LinearBlendStrategy(ITrajectoryStrategy):
    """Linear interpolation fallback with zero terminal velocities."""

    def interpolate_scalar(
        self, s0: float, s1: float, tau: float, duration: float
    ) -> Tuple[float, float, float]:
        if duration <= 0.0:
            return s1, 0.0, 0.0

        t = max(0.0, min(1.0, float(tau)))
        diff = s1 - s0
        pos = s0 + diff * t
        vel = diff / duration if (0.0 < t < 1.0) else 0.0
        acc = 0.0

        return pos, vel, acc


def seconds_to_duration_msg(seconds: float) -> RosDuration:
    """Convert float seconds to builtin_interfaces/msg/Duration."""
    sec = int(seconds)
    nanosec = int((seconds - sec) * 1e9)
    d = RosDuration()
    d.sec = sec
    d.nanosec = nanosec
    return d


class ChessTrajectoryGenerator:
    """
    Constructs safe, sequential joint trajectory goals for LeKiwi chess manipulation.
    """

    def __init__(
        self,
        strategy: Optional[ITrajectoryStrategy] = None,
        joint_names: Sequence[str] = ARM_JOINTS_DEFAULT,
    ) -> None:
        self._strategy = strategy if strategy is not None else QuinticSplineStrategy()
        self._joint_names = list(joint_names)

    @property
    def strategy(self) -> ITrajectoryStrategy:
        return self._strategy

    def create_pre_grasp_pose(
        self, grasp_pose: Dict[str, float], clearance_pitch_offset: float = 0.2
    ) -> Dict[str, float]:
        """Generate a pre-grasp pose slightly elevated above target piece."""
        pose = dict(grasp_pose)
        # Lift arm_shoulder_lift or arm_elbow_flex to retreat vertically
        if "arm_shoulder_lift" in pose:
            pose["arm_shoulder_lift"] -= clearance_pitch_offset
        if "arm_gripper" in pose:
            pose["arm_gripper"] = DEFAULT_GRIPPER_OPEN_RAD
        return pose

    def generate_single_phase_trajectory(
        self,
        q_start: Dict[str, float],
        q_target: Dict[str, float],
        duration_sec: float,
        num_samples: int = 10,
        start_time_offset: float = 0.0,
    ) -> List[JointTrajectoryPoint]:
        """Generate interpolated waypoint samples for a single phase motion."""
        points: List[JointTrajectoryPoint] = []
        if num_samples < 2:
            num_samples = 2

        for i in range(1, num_samples + 1):
            tau = float(i) / float(num_samples)
            t_elapsed = start_time_offset + (tau * duration_sec)

            pt = JointTrajectoryPoint()
            pt.time_from_start = seconds_to_duration_msg(t_elapsed)

            positions: List[float] = []
            velocities: List[float] = []
            accelerations: List[float] = []

            for name in self._joint_names:
                s0 = q_start.get(name, 0.0)
                s1 = q_target.get(name, 0.0)
                pos, vel, acc = self._strategy.interpolate_scalar(
                    s0, s1, tau, duration_sec
                )
                positions.append(pos)
                velocities.append(vel)
                accelerations.append(acc)

            pt.positions = positions
            pt.velocities = velocities
            pt.accelerations = accelerations
            points.append(pt)

        return points

    def build_full_chess_move_trajectory(
        self,
        q_current: Dict[str, float],
        q_pick: Dict[str, float],
        q_place: Dict[str, float],
        is_capture: bool = False,
        phase_duration: float = 1.0,
        samples_per_phase: int = 5,
    ) -> Tuple[JointTrajectory, List[Tuple[ManipulationPhase, float]]]:
        """
        Build a multi-phase JointTrajectory executing a full chess pick and place sequence.

        Phases:
        1. APPROACH_PICK: from current -> pre-grasp (gripper open)
        2. DESCEND_PICK: pre-grasp -> pick grasp pose
        3. GRASP: close gripper
        4. LIFT: pick grasp pose -> pre-grasp (gripper closed)
        5. TRANSIT_PLACE: pre-grasp -> place pre-pose (or chassis bin if capture)
        6. DESCEND_PLACE: pre-pose -> place release pose
        7. RELEASE: open gripper
        8. RETRACT_STOW: return to stow/home pose

        Returns
        -------
        Tuple[JointTrajectory, List[Tuple[ManipulationPhase, float]]]
            The constructed trajectory and list of (phase, elapsed_timestamp) for progress tracking.
        """
        traj = JointTrajectory()
        traj.joint_names = list(self._joint_names)

        q_pre_pick = self.create_pre_grasp_pose(q_pick)
        q_pick_open = dict(q_pick)
        q_pick_open["arm_gripper"] = DEFAULT_GRIPPER_OPEN_RAD

        q_pick_closed = dict(q_pick)
        q_pick_closed["arm_gripper"] = DEFAULT_GRIPPER_CLOSED_RAD

        q_lift_pick = dict(q_pre_pick)
        q_lift_pick["arm_gripper"] = DEFAULT_GRIPPER_CLOSED_RAD

        # Destination target (Place square or Onboard Bin)
        if is_capture:
            q_dest_pre = dict(DEFAULT_BIN_DROP_POSE)
            q_dest_pre["arm_gripper"] = DEFAULT_GRIPPER_CLOSED_RAD
            q_dest_down = dict(DEFAULT_BIN_DROP_POSE)
            q_dest_down["arm_gripper"] = DEFAULT_GRIPPER_CLOSED_RAD
            q_dest_release = dict(DEFAULT_BIN_DROP_POSE)
            q_dest_release["arm_gripper"] = DEFAULT_GRIPPER_OPEN_RAD
        else:
            q_dest_pre = self.create_pre_grasp_pose(q_place)
            q_dest_pre["arm_gripper"] = DEFAULT_GRIPPER_CLOSED_RAD
            q_dest_down = dict(q_place)
            q_dest_down["arm_gripper"] = DEFAULT_GRIPPER_CLOSED_RAD
            q_dest_release = dict(q_place)
            q_dest_release["arm_gripper"] = DEFAULT_GRIPPER_OPEN_RAD

        q_stow = dict(DEFAULT_STOW_POSE)
        q_stow["arm_gripper"] = DEFAULT_GRIPPER_OPEN_RAD

        # Phase pipeline definitions: (phase_enum, start_pose, target_pose, duration)
        pipeline = [
            (ManipulationPhase.APPROACH_PICK, q_current, q_pre_pick, phase_duration),
            (
                ManipulationPhase.DESCEND_PICK,
                q_pre_pick,
                q_pick_open,
                phase_duration * 0.7,
            ),
            (ManipulationPhase.GRASP, q_pick_open, q_pick_closed, phase_duration * 0.5),
            (ManipulationPhase.LIFT, q_pick_closed, q_lift_pick, phase_duration * 0.7),
            (
                ManipulationPhase.TRANSIT_PLACE,
                q_lift_pick,
                q_dest_pre,
                phase_duration * 1.2,
            ),
            (
                ManipulationPhase.DESCEND_PLACE,
                q_dest_pre,
                q_dest_down,
                phase_duration * 0.7,
            ),
            (
                ManipulationPhase.RELEASE,
                q_dest_down,
                q_dest_release,
                phase_duration * 0.5,
            ),
            (ManipulationPhase.RETRACT_STOW, q_dest_release, q_stow, phase_duration),
        ]

        all_points: List[JointTrajectoryPoint] = []
        phase_timeline: List[Tuple[ManipulationPhase, float]] = []
        current_time = 0.0

        for phase, p_from, p_to, dur in pipeline:
            pts = self.generate_single_phase_trajectory(
                p_from, p_to, dur, samples_per_phase, current_time
            )
            all_points.extend(pts)
            current_time += dur
            phase_timeline.append((phase, current_time))

        traj.points = all_points
        return traj, phase_timeline
