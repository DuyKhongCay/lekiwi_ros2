// Copyright 2026 LeKiwi Labs
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file joint_motion_filter.hpp
 * @brief Zero-heap-allocation online low-pass filter and numerical velocity estimator for joint streams.
 * @author LeKiwi Labs
 * @copyright Apache-2.0
 */

#ifndef LEKIWI_MOTION__ARM__JOINT_MOTION_FILTER_HPP_
#define LEKIWI_MOTION__ARM__JOINT_MOTION_FILTER_HPP_

#include <algorithm>
#include <array>
#include <cstddef>
#include <utility>

namespace lekiwi_motion::arm
{

/**
 * @struct FilteredJointState
 * @brief Filtered joint positions and estimated joint velocities.
 */
template <size_t N = 6>
struct FilteredJointState
{
  std::array<double, N> positions{};
  std::array<double, N> velocities{};
};

/**
 * @class JointMotionFilter
 * @brief Exponential Moving Average (EMA) low-pass filter and backward-difference velocity differentiator.
 * @tparam N Number of active joint axes (default 6: 5 arm joints + 1 gripper).
 */
template <size_t N = 6>
class JointMotionFilter
{
public:
  /**
   * @brief Constructs JointMotionFilter with default EMA smoothing factor alpha.
   * @param[in] alpha Smoothing factor in [0.0, 1.0] where 1.0 = pass-through (no filtering).
   */
  explicit JointMotionFilter(double alpha = 0.85) noexcept
  : alpha_(std::clamp(alpha, 0.0, 1.0))
  {
  }

  /**
   * @brief Resets filter history (e.g. on deadman release or discontinuous teleport).
   */
  void reset() noexcept
  {
    initialized_ = false;
    filtered_positions_.fill(0.0);
    previous_positions_.fill(0.0);
    velocities_.fill(0.0);
  }

  /**
   * @brief Updates filter with new target positions and step duration dt.
   * @param[in] target_positions Raw target positions.
   * @param[in] dt Time step elapsed since last update (seconds).
   * @return FilteredJointState containing smoothed positions and differentiated velocities.
   */
  FilteredJointState<N> update(const std::array<double, N> & target_positions, double dt) noexcept
  {
    if (!initialized_ || alpha_ >= 0.999) {
      filtered_positions_ = target_positions;
      previous_positions_ = target_positions;
      velocities_.fill(0.0);
      initialized_ = true;
    } else {
      for (size_t i = 0; i < N; ++i) {
        filtered_positions_[i] =
          alpha_ * target_positions[i] + (1.0 - alpha_) * filtered_positions_[i];
      }

      if (dt > 0.001 && dt <= 0.2) {
        const double inv_dt = 1.0 / dt;
        for (size_t i = 0; i < N; ++i) {
          velocities_[i] = (filtered_positions_[i] - previous_positions_[i]) * inv_dt;
        }
      } else {
        velocities_.fill(0.0);
      }
      previous_positions_ = filtered_positions_;
    }

    return {filtered_positions_, velocities_};
  }

  /**
   * @brief Sets smoothing factor alpha.
   * @param[in] alpha Value in [0.0, 1.0].
   */
  void set_alpha(double alpha) noexcept
  {
    alpha_ = std::clamp(alpha, 0.0, 1.0);
  }

  [[nodiscard]] double alpha() const noexcept { return alpha_; }
  [[nodiscard]] bool is_initialized() const noexcept { return initialized_; }
  [[nodiscard]] const std::array<double, N> & current_positions() const noexcept
  {
    return filtered_positions_;
  }

private:
  double alpha_{0.85};
  bool initialized_{false};
  std::array<double, N> filtered_positions_{};
  std::array<double, N> previous_positions_{};
  std::array<double, N> velocities_{};
};

}  // namespace lekiwi_motion::arm

#endif  // LEKIWI_MOTION__ARM__JOINT_MOTION_FILTER_HPP_

