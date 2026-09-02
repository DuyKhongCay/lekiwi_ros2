/**
 * @file follower_dispatcher.cpp
 * @brief Implementation of FollowerDispatcher.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include "teleop_zhongli_servo_hw/follower_dispatcher.hpp"

#include <algorithm>
#include <cctype>

namespace teleop_zhongli_servo_hw
{

  void FollowerDispatcher::init(rclcpp::Node *node, const DispatcherConfig &config)
  {
    if (!node)
    {
      throw std::invalid_argument("Node pointer cannot be null in FollowerDispatcher::init");
    }

    config_ = config;
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    if (config_.lpf_alpha <= 0.0 || config_.lpf_alpha > 1.0)
    {
      RCLCPP_WARN(
          logger_,
          "Parameter 'lpf_alpha' (%.3f) out of range (0.0, 1.0], clamping to 1.0",
          config_.lpf_alpha);
      config_.lpf_alpha = 1.0;
    }

    mode_ = string_to_mode(config_.arm_mode);

    // Controller command publishers use Reliable QoS
    trajectory_pub_ = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        config_.follower_jtc_topic, rclcpp::QoS(10).reliable());

    forward_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
        config_.follower_fwd_topic, rclcpp::QoS(10).reliable());

    lpf_initialized_ = false;
    lpf_filtered_positions_.clear();

    RCLCPP_INFO(
        logger_,
        "FollowerDispatcher initialized. Mode: '%s', JTC: '%s', FWD: '%s' (dt=%.3fs, lpf_alpha=%.2f, joints=%zu)",
        mode_to_string(mode_).c_str(),
        config_.follower_jtc_topic.c_str(),
        config_.follower_fwd_topic.c_str(),
        config_.point_dt_s,
        config_.lpf_alpha,
        config_.follower_joint_names.size());
  }

  void FollowerDispatcher::dispatch(
      const std::vector<double> &follower_positions_rad,
      const rclcpp::Time &stamp)
  {
    if (mode_ == DispatchMode::JOINT_STATES_ONLY)
    {
      return;
    }

    const size_t num_joints = follower_positions_rad.size();
    if (num_joints == 0)
    {
      return;
    }

    // Low-Pass Filter (LPF)
    if (!lpf_initialized_ || config_.lpf_alpha >= 0.9999 || lpf_filtered_positions_.size() != num_joints)
    {
      lpf_filtered_positions_ = follower_positions_rad;
      lpf_initialized_ = true;
    }
    else
    {
      for (size_t i = 0; i < num_joints; ++i)
      {
        lpf_filtered_positions_[i] =
            config_.lpf_alpha * follower_positions_rad[i] +
            (1.0 - config_.lpf_alpha) * lpf_filtered_positions_[i];
      }
    }

    if (mode_ == DispatchMode::JOINT_TRAJECTORY)
    {
      trajectory_msgs::msg::JointTrajectory jt;
      jt.header.stamp = stamp;
      jt.joint_names = config_.follower_joint_names;

      trajectory_msgs::msg::JointTrajectoryPoint pt;
      pt.positions = lpf_filtered_positions_;

      const int sec = static_cast<int>(config_.point_dt_s);
      const uint32_t nsec = static_cast<uint32_t>((config_.point_dt_s - sec) * 1e9);
      pt.time_from_start.sec = sec;
      pt.time_from_start.nanosec = nsec;

      jt.points.push_back(std::move(pt));
      trajectory_pub_->publish(jt);
    }
    else if (mode_ == DispatchMode::FORWARD_POSITION)
    {
      std_msgs::msg::Float64MultiArray cmd;
      cmd.data = lpf_filtered_positions_;
      forward_pub_->publish(cmd);
    }
    else
    {
      RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 5000,
          "Unknown dispatch mode '%s'.",
          config_.arm_mode.c_str());
    }
  }

  void FollowerDispatcher::reset()
  {
    lpf_initialized_ = false;
    lpf_filtered_positions_.clear();
  }

  DispatchMode FollowerDispatcher::string_to_mode(const std::string &mode_str)
  {
    std::string lower;
    lower.reserve(mode_str.size());
    for (char c : mode_str)
    {
      lower.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }

    if (lower == "forward_position")
    {
      return DispatchMode::FORWARD_POSITION;
    }
    if (lower == "joint_states_only")
    {
      return DispatchMode::JOINT_STATES_ONLY;
    }
    return DispatchMode::JOINT_TRAJECTORY;
  }

  std::string FollowerDispatcher::mode_to_string(DispatchMode mode)
  {
    switch (mode)
    {
    case DispatchMode::FORWARD_POSITION:
      return "forward_position";
    case DispatchMode::JOINT_STATES_ONLY:
      return "joint_states_only";
    case DispatchMode::JOINT_TRAJECTORY:
    default:
      return "joint_trajectory";
    }
  }

} // namespace teleop_zhongli_servo_hw
