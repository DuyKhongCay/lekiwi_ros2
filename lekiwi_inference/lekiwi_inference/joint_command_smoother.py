# Copyright 2026 LeKiwi Robotics Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Joint command smoothing and interpolation pipeline for LeKiwi arm actuators."""

from typing import Optional
import numpy as np


def apply_soft_deadband(
    delta_xyz: np.ndarray,
    xy_threshold_m: float = 0.012,
    z_threshold_m: float = 0.012,
) -> np.ndarray:
    """Attenuate micro-deviations smoothly near target to eliminate hunting limit cycles.

    When the end-effector is within the threshold zone of the subtask target,
    the relative delta coordinates are scaled down continuously towards zero.
    This eliminates high-frequency sign oscillations in the 12D observation vector.

    Args:
        delta_xyz: Array of shape (3,) representing (target_xyz - ee_xyz) in meters.
        xy_threshold_m: Threshold in XY plane below which soft attenuation applies.
        z_threshold_m: Threshold along Z axis below which soft attenuation applies.

    Returns:
        Deadband-attenuated delta_xyz array of shape (3,).
    """
    result = delta_xyz.astype(np.float32, copy=True)
    dist_xy = float(np.linalg.norm(delta_xyz[:2]))
    if 1e-6 < dist_xy < xy_threshold_m:
        scale_xy = dist_xy / xy_threshold_m
        result[0] *= scale_xy
        result[1] *= scale_xy

    abs_z = abs(float(delta_xyz[2]))
    if 1e-6 < abs_z < z_threshold_m:
        scale_z = abs_z / z_threshold_m
        result[2] *= scale_z

    return result


class JointCommandSmoother:
    """Decoupled 50Hz Actuation Smoother for 25Hz Policy Action Chunks.

    Design Principles (SRP & SoC):
    1. Sub-step midpoint linear interpolation (at 20ms offset) between consecutive
       25Hz policy action targets to match the 50Hz controller_manager rate.
    2. Exponential Moving Average (EMA) low-pass filtering on arm joints 0..4 (alpha=0.75),
       eliminating high-frequency staircase velocity shocks to the arm servos.
    3. Direct unfiltered passthrough for gripper joint 5 (alpha=1.0) to maintain crisp,
       instantaneous grasp/release dynamics.
    """

    def __init__(self, ema_alpha: float = 0.75, num_joints: int = 6) -> None:
        """Initialize the command smoother.

        Args:
            ema_alpha: Exponential moving average coefficient (0.0 < alpha <= 1.0).
            num_joints: Total number of arm joints (Kipling arm: 6 joints).
        """
        self._alpha: float = float(ema_alpha)
        self._num_joints: int = num_joints
        self._prev_target: Optional[np.ndarray] = None
        self._curr_target: Optional[np.ndarray] = None
        self._filtered_state: Optional[np.ndarray] = None
        self._sub_step: int = 0

    @property
    def is_initialized(self) -> bool:
        """Returns True if the smoother has been seeded with joint positions."""
        return self._curr_target is not None

    def reset(self, initial_joints: np.ndarray) -> None:
        """Reset internal targets and filtered state to physical robot joints.

        Args:
            initial_joints: Array of shape (6,) representing current robot joints.
        """
        joints = np.asarray(initial_joints, dtype=np.float64)
        self._prev_target = joints.copy()
        self._curr_target = joints.copy()
        self._filtered_state = joints.copy()
        self._sub_step = 0

    def update_policy_target(self, new_target: np.ndarray) -> None:
        """Called at 25Hz when a new action is popped from the ActionQueue.

        Args:
            new_target: Array of shape (6,) containing target joint angles.
        """
        target = np.asarray(new_target, dtype=np.float64)
        if self._curr_target is None:
            self.reset(target)
            return

        self._prev_target = self._curr_target.copy()
        self._curr_target = target.copy()
        self._sub_step = 0

    def step_actuation(self) -> Optional[np.ndarray]:
        """Called at 50Hz by the actuation timer to produce the next output command.

        Returns:
            Array of shape (6,) with smoothed joint positions, or None if uninitialized.
        """
        if self._curr_target is None or self._prev_target is None or self._filtered_state is None:
            return None

        # 1. Midpoint linear interpolation
        # Sub-step 0 (t = 0ms): midpoint between previous target and current target
        # Sub-step 1 (t = 20ms): full current target reached
        if self._sub_step == 0:
            interp = 0.5 * (self._prev_target + self._curr_target)
            self._sub_step = 1
        else:
            interp = self._curr_target.copy()

        # 2. Selective EMA low-pass filtering
        out = np.zeros(self._num_joints, dtype=np.float64)
        # Filter arm joints 0..4 (shoulder pan, shoulder lift, elbow flex, wrist flex, wrist roll)
        for j in range(min(5, self._num_joints)):
            out[j] = self._alpha * interp[j] + (1.0 - self._alpha) * self._filtered_state[j]

        # Preserve unfiltered raw command on gripper joint 5
        if self._num_joints >= 6:
            out[5] = interp[5]

        self._filtered_state = out.copy()
        return out
