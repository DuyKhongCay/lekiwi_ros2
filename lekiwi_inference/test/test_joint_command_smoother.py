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

"""Unit tests for JointCommandSmoother and Soft Deadband filtering."""

import numpy as np
import pytest

from lekiwi_inference.joint_command_smoother import JointCommandSmoother, apply_soft_deadband


def test_soft_deadband_attenuation():
    """Validates smooth polynomial/linear attenuation inside threshold and passthrough outside."""
    threshold = 0.012

    # Case 1: Point well outside deadband (> 0.012m) -> Passthrough exactly
    outside_delta = np.array([0.030, 0.040, 0.050], dtype=np.float32)
    filtered = apply_soft_deadband(outside_delta, xy_threshold_m=threshold, z_threshold_m=threshold)
    np.testing.assert_allclose(filtered, outside_delta, atol=1e-6)

    # Case 2: Zero delta -> Exactly zeros
    zero_delta = np.zeros(3, dtype=np.float32)
    filtered_zero = apply_soft_deadband(zero_delta, xy_threshold_m=threshold, z_threshold_m=threshold)
    np.testing.assert_allclose(filtered_zero, np.zeros(3), atol=1e-6)

    # Case 3: Point inside deadband (e.g. dist_xy = 0.006m, half threshold)
    # Expected: scaled down by (0.006 / 0.012) = 0.5
    inside_xy = np.array([0.006, 0.0, 0.050], dtype=np.float32)
    filtered_xy = apply_soft_deadband(inside_xy, xy_threshold_m=threshold, z_threshold_m=threshold)
    # XY scaled by 0.5: 0.006 * 0.5 = 0.003
    assert abs(filtered_xy[0] - 0.003) < 1e-5
    assert filtered_xy[1] == 0.0
    # Z was 0.050 (> 0.012), unchanged
    assert abs(filtered_xy[2] - 0.050) < 1e-6

    # Case 4: Z inside deadband
    inside_z = np.array([0.050, 0.0, 0.006], dtype=np.float32)
    filtered_z = apply_soft_deadband(inside_z, xy_threshold_m=threshold, z_threshold_m=threshold)
    assert abs(filtered_z[0] - 0.050) < 1e-6
    assert abs(filtered_z[2] - 0.003) < 1e-5


def test_joint_command_smoother_initialization():
    """Validates smoother uninitialized state and physical robot seeding."""
    smoother = JointCommandSmoother(ema_alpha=0.75, num_joints=6)
    assert not smoother.is_initialized
    assert smoother.step_actuation() is None

    seed_joints = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6], dtype=np.float64)
    smoother.reset(seed_joints)
    assert smoother.is_initialized

    # Immediate step without new policy target holds initial state
    out = smoother.step_actuation()
    np.testing.assert_allclose(out, seed_joints, atol=1e-6)


def test_midpoint_interpolation_and_selective_ema():
    """Validates 50Hz sub-step midpoint interpolation and EMA smoothing with gripper passthrough."""
    smoother = JointCommandSmoother(ema_alpha=0.75, num_joints=6)

    # Initial robot state: all zeros
    initial = np.zeros(6, dtype=np.float64)
    smoother.reset(initial)

    # Policy pops target: all ones [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    target_1 = np.ones(6, dtype=np.float64)
    smoother.update_policy_target(target_1)

    # --- Sub-step 0 (t = 0ms): Midpoint interpolation ---
    # Midpoint interp = 0.5 * (0.0 + 1.0) = 0.5
    # For joints 0..4 (EMA alpha=0.75): out = 0.75 * 0.5 + 0.25 * 0.0 = 0.375
    # For joint 5 (gripper, unfiltered): out = 0.5
    step_0 = smoother.step_actuation()
    assert step_0 is not None
    for j in range(5):
        assert abs(step_0[j] - 0.375) < 1e-6, f"Joint {j} expected 0.375, got {step_0[j]}"
    assert abs(step_0[5] - 0.5) < 1e-6, f"Gripper joint 5 expected 0.5, got {step_0[5]}"

    # --- Sub-step 1 (t = 20ms): Full target interpolation ---
    # Full interp = 1.0
    # For joints 0..4: out = 0.75 * 1.0 + 0.25 * 0.375 = 0.75 + 0.09375 = 0.84375
    # For joint 5 (gripper, unfiltered): out = 1.0 (reaches full target immediately!)
    step_1 = smoother.step_actuation()
    assert step_1 is not None
    for j in range(5):
        assert abs(step_1[j] - 0.84375) < 1e-6, f"Joint {j} expected 0.84375, got {step_1[j]}"
    assert abs(step_1[5] - 1.0) < 1e-6, f"Gripper joint 5 expected 1.0, got {step_1[5]}"
