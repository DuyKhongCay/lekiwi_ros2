// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include "lekiwi_motion/tf_gatekeeper_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rclcpp_components/register_node_macro.hpp>

using namespace std::chrono_literals;

namespace lekiwi_motion
{

  TfGatekeeperNode::TfGatekeeperNode(const rclcpp::NodeOptions &options)
      : Node("tf_gatekeeper_node", options),
        diagnostic_updater_(this)
  {
    auto declare = [this](const auto &name, const auto &value)
    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.description = std::string("Readiness configuration: ") + name;
      declare_parameter(name, value, desc);
    };

    declare("max_pos_variance", 0.0012);
    declare("max_yaw_variance", 0.0030);
    declare("max_transform_age_sec", 0.30);
    declare("max_stop_velocity", 0.03);
    declare("max_stop_angular_vel", 0.08);
    declare("check_frequency_hz", 10.0);

    declare("map_frame", std::string("map"));
    declare("odom_frame", std::string("odom"));
    declare("base_frame", std::string("base_footprint"));
    declare("ee_frame", std::string("gripperframe"));
    declare("board_frame", std::string("chessboard_frame"));
    declare("required_arm_joints", std::vector<std::string>{
                                       "arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex",
                                       "arm_wrist_flex", "arm_wrist_roll", "arm_gripper"});

    max_pos_var_ = get_parameter("max_pos_variance").as_double();
    max_yaw_var_ = get_parameter("max_yaw_variance").as_double();
    max_transform_age_sec_ = get_parameter("max_transform_age_sec").as_double();
    max_stop_velocity_ = get_parameter("max_stop_velocity").as_double();
    max_stop_angular_vel_ = get_parameter("max_stop_angular_vel").as_double();
    check_frequency_hz_ = get_parameter("check_frequency_hz").as_double();

    map_frame_ = get_parameter("map_frame").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    ee_frame_ = get_parameter("ee_frame").as_string();
    board_frame_ = get_parameter("board_frame").as_string();
    required_arm_joints_ = get_parameter("required_arm_joints").as_string_array();

    // Validate parameters
    const std::vector<double> positive = {
        max_pos_var_, max_yaw_var_, max_transform_age_sec_, check_frequency_hz_};
    for (double val : positive)
    {
      if (!std::isfinite(val) || val <= 0.0)
      {
        throw std::invalid_argument("Readiness thresholds and frequency must be finite and positive");
      }
    }
    if (!std::isfinite(max_stop_velocity_) || !std::isfinite(max_stop_angular_vel_) ||
        max_stop_velocity_ < 0.0 || max_stop_angular_vel_ < 0.0 ||
        check_frequency_hz_ > 1000.0 || required_arm_joints_.empty() ||
        map_frame_.empty() || odom_frame_.empty() || base_frame_.empty() ||
        ee_frame_.empty() || board_frame_.empty())
    {
      throw std::invalid_argument("Invalid readiness limits, frames or joint list");
    }

    // TF2 Buffer + Listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    // Subscriptions
    auto sensor_qos = rclcpp::QoS(10).best_effort();
    local_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", sensor_qos,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg)
        { on_local_odom(msg); });

    global_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/global", sensor_qos,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg)
        { on_global_odom(msg); });

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", sensor_qos,
        [this](sensor_msgs::msg::JointState::ConstSharedPtr msg)
        { on_joint_states(msg); });

    // Latched publisher: late subscribers receive current readiness state immediately
    auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
    tf_ready_pub_ = create_publisher<std_msgs::msg::Bool>("/system/tf_ready", latched_qos);
    publish_tf_ready(false);

    // Query service
    readiness_srv_ = create_service<std_srvs::srv::Trigger>(
        "/system/check_tf_readiness",
        [this](
            const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
            std::shared_ptr<std_srvs::srv::Trigger::Response> res)
        {
          handle_readiness_query(req, res);
        });

    // Diagnostics
    diagnostic_updater_.setHardwareID("lekiwi_tf_gatekeeper");
    diagnostic_updater_.add("TF Tree Readiness", this, &TfGatekeeperNode::produce_diagnostics);

    // Periodic evaluation timer
    const auto period = std::chrono::duration<double>(1.0 / check_frequency_hz_);
    eval_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&TfGatekeeperNode::evaluate_readiness, this));

    RCLCPP_INFO(get_logger(),
                "[TF Gatekeeper] Initialized (%.1f Hz) — map=%s, odom=%s, base=%s, ee=%s, board=%s",
                check_frequency_hz_, map_frame_.c_str(), odom_frame_.c_str(),
                base_frame_.c_str(), ee_frame_.c_str(), board_frame_.c_str());
  }

  void TfGatekeeperNode::on_local_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    last_local_odom_ = msg;
  }

  void TfGatekeeperNode::on_global_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    last_global_odom_ = msg;
  }

  void TfGatekeeperNode::on_joint_states(sensor_msgs::msg::JointState::ConstSharedPtr msg)
  {
    last_joint_time_sec_ = rclcpp::Time(msg->header.stamp).seconds();
    for (const auto &name : msg->name)
    {
      received_joints_.insert(name);
    }
  }

  bool TfGatekeeperNode::check_ekf_readiness(
      double now_sec,
      double &pos_var_out,
      double &yaw_var_out,
      bool &is_fresh_out) const
  {
    is_fresh_out = false;
    pos_var_out = std::numeric_limits<double>::infinity();
    yaw_var_out = std::numeric_limits<double>::infinity();

    if (!last_global_odom_)
    {
      return false;
    }

    const auto &stamp = last_global_odom_->header.stamp;
    const double ekf_stamp_sec = stamp.sec + stamp.nanosec * 1e-9;
    is_fresh_out = (last_global_odom_->header.frame_id == map_frame_) &&
                   (last_global_odom_->child_frame_id == base_frame_) &&
                   policy::fresh(now_sec, ekf_stamp_sec, max_transform_age_sec_);

    if (!is_fresh_out)
    {
      return false;
    }

    const auto &cov = last_global_odom_->pose.covariance;
    pos_var_out = cov[0] + cov[7]; // var(x) + var(y)
    yaw_var_out = cov[35];         // var(yaw)

    return policy::converged(cov[0], cov[7], cov[35], max_pos_var_, max_yaw_var_);
  }

  bool TfGatekeeperNode::check_robot_stationary(double &speed_out) const
  {
    if (!last_local_odom_ || last_local_odom_->header.frame_id != odom_frame_ ||
        last_local_odom_->child_frame_id != base_frame_ ||
        !policy::fresh(get_clock()->now().seconds(),
                       rclcpp::Time(last_local_odom_->header.stamp).seconds(),
                       max_transform_age_sec_))
    {
      speed_out = 0.0;
      return false;
    }
    const auto &twist = last_local_odom_->twist.twist;
    speed_out = std::hypot(twist.linear.x, twist.linear.y);
    return policy::stationary(twist.linear.x, twist.linear.y, twist.angular.z,
                              max_stop_velocity_, max_stop_angular_vel_);
  }

  bool TfGatekeeperNode::check_joints_complete() const
  {
    if (last_joint_time_sec_ <= 0.0)
    {
      return false;
    }
    const double now_sec = get_clock()->now().seconds();
    if (!policy::fresh(now_sec, last_joint_time_sec_, max_transform_age_sec_))
    {
      return false;
    }
    for (const auto &j : required_arm_joints_)
    {
      if (received_joints_.find(j) == received_joints_.end())
      {
        return false;
      }
    }
    return true;
  }

  std::vector<std::string> TfGatekeeperNode::get_missing_joints() const
  {
    std::vector<std::string> missing;
    for (const auto &j : required_arm_joints_)
    {
      if (received_joints_.find(j) == received_joints_.end())
      {
        missing.push_back(j);
      }
    }
    return missing;
  }

  bool TfGatekeeperNode::is_transform_fresh(
      const std::string &target,
      const std::string &source,
      double now_sec,
      bool is_static) const
  {
    try
    {
      auto tf = tf_buffer_->lookupTransform(target, source, tf2::TimePointZero);
      if (is_static)
      {
        return true;
      }
      const double stamp_sec = rclcpp::Time(tf.header.stamp).seconds();
      return policy::fresh(now_sec, stamp_sec, max_transform_age_sec_);
    }
    catch (const tf2::TransformException &)
    {
      return false;
    }
  }

  bool TfGatekeeperNode::check_tf_chains_fresh(double now_sec) const
  {
    return is_transform_fresh(map_frame_, odom_frame_, now_sec, false) &&
           is_transform_fresh(odom_frame_, base_frame_, now_sec, false) &&
           is_transform_fresh(base_frame_, ee_frame_, now_sec, false) &&
           is_transform_fresh(map_frame_, board_frame_, now_sec, true);
  }

  void TfGatekeeperNode::evaluate_readiness()
  {
    const double now_sec = get_clock()->now().seconds();
    if (last_eval_time_sec_ > 0.0 && now_sec < last_eval_time_sec_)
    {
      tf_buffer_->clear();
    }
    last_eval_time_sec_ = now_sec;

    // 1. Joint completeness + freshness (Zero heap-allocation on hot path)
    const bool joints_ok = check_joints_complete();

    // 2. Stationary check (local odometry)
    double speed{0.0};
    const bool is_stationary = check_robot_stationary(speed);

    // 3. EKF covariance convergence & freshness (Single-pass evaluation)
    double pos_var{std::numeric_limits<double>::infinity()};
    double yaw_var{std::numeric_limits<double>::infinity()};
    bool ekf_fresh{false};
    const bool ekf_converged = check_ekf_readiness(now_sec, pos_var, yaw_var, ekf_fresh);

    // 4. TF chain completeness & freshness
    const bool tf_ok = check_tf_chains_fresh(now_sec);

    const bool system_ready = joints_ok && is_stationary && ekf_converged && ekf_fresh && tf_ok;

    diag_pos_var_ = pos_var;
    diag_yaw_var_ = yaw_var;
    diag_speed_ = speed;

    if (system_ready != is_tf_ready_)
    {
      is_tf_ready_ = system_ready;
      if (is_tf_ready_)
      {
        RCLCPP_INFO(get_logger(),
                    ">>> [TF GATEKEEPER] SYSTEM READY! "
                    "speed=%.3fm/s | pos_var=%.6f (lim=%.4f) | yaw_var=%.6f (lim=%.4f)",
                    speed, pos_var, max_pos_var_, yaw_var, max_yaw_var_);
      }
      else
      {
        RCLCPP_WARN(get_logger(),
                    "[TF GATEKEEPER] SYSTEM UNREADY → "
                    "stationary=%d | ekf_conv=%d | ekf_fresh=%d | joints=%d | tf=%d",
                    is_stationary, ekf_converged, ekf_fresh, joints_ok, tf_ok);
        if (!joints_ok)
        {
          auto missing_joints = get_missing_joints();
          std::string missing_str;
          for (const auto &j : missing_joints)
          {
            missing_str += j + " ";
          }
          RCLCPP_WARN(get_logger(), "[TF GATEKEEPER] Missing joints: [%s]", missing_str.c_str());
        }
      }
    }

    publish_tf_ready(is_tf_ready_);
    diagnostic_updater_.force_update();
  }

  void TfGatekeeperNode::publish_tf_ready(bool ready)
  {
    auto msg = std_msgs::msg::Bool();
    msg.data = ready;
    tf_ready_pub_->publish(msg);
  }

  void TfGatekeeperNode::produce_diagnostics(
      diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    using DS = diagnostic_msgs::msg::DiagnosticStatus;
    if (is_tf_ready_)
    {
      stat.summary(DS::OK, "System localized, stationary — ready for chess");
    }
    else if (std::isinf(diag_pos_var_) || std::isinf(diag_yaw_var_))
    {
      stat.summary(DS::WARN, "EKF diverging or no odometry — check /odometry/global");
    }
    else
    {
      stat.summary(DS::WARN, "EKF converging, robot moving or TF chain not fresh");
    }

    stat.add("tf_ready", is_tf_ready_ ? "true" : "false");

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6) << diag_pos_var_ << " (limit: "
       << std::setprecision(4) << max_pos_var_ << " m^2)";
    stat.add("pos_variance", ss.str());

    ss.str("");
    ss.clear();
    ss << std::fixed << std::setprecision(6) << diag_yaw_var_ << " (limit: "
       << std::setprecision(4) << max_yaw_var_ << " rad^2)";
    stat.add("yaw_variance", ss.str());

    ss.str("");
    ss.clear();
    ss << std::fixed << std::setprecision(4) << diag_speed_ << " (limit: "
       << std::setprecision(3) << max_stop_velocity_ << " m/s)";
    stat.add("linear_speed", ss.str());
  }

  void TfGatekeeperNode::handle_readiness_query(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    res->success = is_tf_ready_;
    res->message = is_tf_ready_
                       ? "TF verified, EKF converged, robot stationary — ready for chess"
                       : "NOT ready: check /diagnostics for details (topic /system/tf_ready)";
  }

} // namespace lekiwi_motion

RCLCPP_COMPONENTS_REGISTER_NODE(lekiwi_motion::TfGatekeeperNode)
