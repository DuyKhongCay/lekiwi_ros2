/**
 * @file follower_dispatcher.hpp
 * @brief Follower Controller Dispatcher & Safety Filter for teleoperation.
 *
 * Encapsulates:
 * - ROS 2 Controller command publishers (JointTrajectory, Float64MultiArray)
 * - Low-Pass Filter (LPF) for trajectory smoothing
 * - Dispatching mode logic (joint_trajectory, forward_position, joint_states_only)
 * - Safety reset on disconnect / timeout
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace teleop_zhongli_servo_hw
{

  enum class DispatchMode
  {
    JOINT_TRAJECTORY,
    FORWARD_POSITION,
    JOINT_STATES_ONLY
  };

  struct DispatcherConfig
  {
    std::string arm_mode{"joint_trajectory"};
    std::string follower_jtc_topic{"/arm_trajectory_controller/joint_trajectory"};
    std::string follower_fwd_topic{"/arm_forward_controller/commands"};
    double point_dt_s{0.04};
    double lpf_alpha{1.0};
    std::vector<std::string> follower_joint_names;
  };

  class FollowerDispatcher
  {
  public:
    FollowerDispatcher() = default;
    ~FollowerDispatcher() = default;

    /**
     * @brief Initialize publishers and configurations using the host ROS 2 node.
     * @param node Pointer to host rclcpp::Node.
     * @param config Dispatcher configuration.
     */
    void init(rclcpp::Node *node, const DispatcherConfig &config);

    /**
     * @brief Filter positions and publish command to configured follower controller.
     * @param follower_positions_rad Unfiltered follower joint angles in radians.
     * @param stamp Message timestamp.
     */
    void dispatch(const std::vector<double> &follower_positions_rad, const rclcpp::Time &stamp);

    /**
     * @brief Reset filter state on disconnect or timeout to avoid step jumps upon reconnect.
     */
    void reset();

    [[nodiscard]] DispatchMode get_mode() const noexcept { return mode_; }
    [[nodiscard]] const DispatcherConfig &get_config() const noexcept { return config_; }
    [[nodiscard]] const std::vector<double> &get_filtered_positions() const noexcept { return lpf_filtered_positions_; }
    [[nodiscard]] bool is_lpf_initialized() const noexcept { return lpf_initialized_; }

    static DispatchMode string_to_mode(const std::string &mode_str);
    static std::string mode_to_string(DispatchMode mode);

  private:
    DispatcherConfig config_;
    DispatchMode mode_{DispatchMode::JOINT_TRAJECTORY};

    std::vector<double> lpf_filtered_positions_;
    bool lpf_initialized_{false};

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr forward_pub_;
    rclcpp::Logger logger_{rclcpp::get_logger("follower_dispatcher")};
    rclcpp::Clock::SharedPtr clock_;
  };

} // namespace teleop_zhongli_servo_hw
