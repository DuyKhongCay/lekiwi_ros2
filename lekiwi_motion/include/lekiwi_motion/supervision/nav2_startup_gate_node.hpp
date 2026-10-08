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
 * @file nav2_startup_gate_node.hpp
 * @brief Startup gatekeeper ensuring TF readiness before activating Nav2 stack.
 * @details Monitors the transform connectivity from map to base_footprint and
 *          triggers Nav2 lifecycle manager startup once localization is stable.
 * @author LeKiwi Labs
 * @copyright Apache-2.0
 */

#ifndef LEKIWI_MOTION__NAV2_STARTUP_GATE_NODE_HPP_
#define LEKIWI_MOTION__NAV2_STARTUP_GATE_NODE_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <string>

#include <nav2_msgs/srv/manage_lifecycle_nodes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace lekiwi_motion
{

  /**
   * @class Nav2StartupGateNode
   * @brief Node delaying Nav2 activation until localization TF is confirmed.
   * @details Debounces transform availability across consecutive sampling cycles
   *          to prevent Nav2 nodes from entering active state with broken TF trees.
   */
  class Nav2StartupGateNode : public rclcpp::Node
  {
  public:
    /**
     * @brief Constructs gatekeeper node and registers TF listener and lifecycle client.
     * @param[in] options Node options passed to base rclcpp::Node.
     */
    explicit Nav2StartupGateNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    /**
     * @brief Default virtual destructor.
     */
    ~Nav2StartupGateNode() override = default;

    /**
     * @brief Checks whether Nav2 lifecycle startup has successfully completed.
     * @return True if Nav2 transition to ACTIVE succeeded.
     */
    bool is_started() const noexcept { return started_.load(); }

    /**
     * @brief Retrieves current consecutive successful TF lookup count.
     * @return Number of consecutive successful checks.
     */
    int get_consecutive_count() const noexcept { return consecutive_count_; }

  private:
    /**
     * @brief Periodically verifies TF chain and sends startup trigger upon meeting threshold.
     */
    void check_and_startup();

    /**
     * @brief Asynchronous callback receiving Nav2 lifecycle manager transition response.
     * @param[in] future Future holding ManageLifecycleNodes response.
     */
    void on_startup_response(
        rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedFuture future);

    /// Parameter configuration.
    std::string map_frame_;
    std::string base_frame_;
    std::string lifecycle_service_;
    double check_frequency_hz_;
    int consecutive_success_threshold_;
    bool exit_on_success_;

    /// Internal state tracking.
    std::atomic<bool> started_{false};
    bool request_in_flight_{false};
    int consecutive_count_{0};

    /// ROS 2 execution primitives.
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr client_;

    /// TF2 buffer and listener.
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  };

} // namespace lekiwi_motion

#endif // LEKIWI_MOTION__NAV2_STARTUP_GATE_NODE_HPP_
