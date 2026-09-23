// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.
//
// TF Tree Readiness Gatekeeper — C++ Node / Composable Component
//
// Nhiệm vụ 1: Giám sát toàn vẹn chuỗi TF tree và độ tươi dữ liệu
// Nhiệm vụ 2: Kiểm tra EKF toàn cục hội tụ phương sai và robot dừng ổn định
// Nhiệm vụ 3: Cung cấp tín hiệu /system/tf_ready và service /system/check_tf_readiness
//
// Quyền kích hoạt/reset EKF toàn cục thuộc về Gamepad Teleop hoặc Orchestrator.

#ifndef LEKIWI_MOTION__TF_GATEKEEPER_NODE_HPP_
#define LEKIWI_MOTION__TF_GATEKEEPER_NODE_HPP_

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace lekiwi_motion
{

  // ================= Pure Mathematical Readiness Policies =================
  namespace policy
  {
    // Reject missing, stale and future samples against the same ROS clock.
    inline bool fresh(double now, double stamp, double max_age)
    {
      return std::isfinite(now) && std::isfinite(stamp) && stamp > 0.0 &&
             now >= stamp && now - stamp <= max_age;
    }

    // Check measured planar velocities only after the caller verifies freshness.
    inline bool stationary(double vx, double vy, double wz, double linear, double angular)
    {
      return std::isfinite(vx) && std::isfinite(vy) && std::isfinite(wz) &&
             std::hypot(vx, vy) <= linear && std::abs(wz) <= angular;
    }

    // Validate each variance before summing so negative terms cannot cancel.
    inline bool converged(double x, double y, double yaw, double max_pos, double max_yaw)
    {
      return std::isfinite(x) && std::isfinite(y) && std::isfinite(yaw) &&
             x >= 0.0 && y >= 0.0 && yaw >= 0.0 && x + y <= max_pos && yaw <= max_yaw;
    }
  } // namespace policy

  // ================= TfGatekeeperNode =================
  class TfGatekeeperNode : public rclcpp::Node
  {
  public:
    explicit TfGatekeeperNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~TfGatekeeperNode() override = default;

  private:
    // Core evaluation callback (check_frequency_hz)
    void evaluate_readiness();

    // Subscription callbacks
    void on_local_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void on_global_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void on_joint_states(sensor_msgs::msg::JointState::ConstSharedPtr msg);

    // Verification predicates
    bool check_ekf_readiness(
        double now_sec,
        double &pos_var_out,
        double &yaw_var_out,
        bool &is_fresh_out) const;
    bool check_robot_stationary(double &speed_out) const;
    bool check_joints_complete() const;
    std::vector<std::string> get_missing_joints() const;
    bool check_tf_chains_fresh(double now_sec) const;
    bool is_transform_fresh(
        const std::string &target,
        const std::string &source,
        double now_sec,
        bool is_static = false) const;

    // Output & diagnostics
    void publish_tf_ready(bool ready);
    void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void handle_readiness_query(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    // Parameters
    double max_pos_var_{0.0012};
    double max_yaw_var_{0.0030};
    double max_transform_age_sec_{0.30};
    double max_stop_velocity_{0.03};
    double max_stop_angular_vel_{0.08};
    double check_frequency_hz_{10.0};
    std::string map_frame_{"map"};
    std::string odom_frame_{"odom"};
    std::string base_frame_{"base_footprint"};
    std::string ee_frame_{"gripperframe"};
    std::string board_frame_{"chessboard_frame"};
    std::vector<std::string> required_arm_joints_;

    // Runtime state
    bool is_tf_ready_{false};
    nav_msgs::msg::Odometry::ConstSharedPtr last_local_odom_;
    nav_msgs::msg::Odometry::ConstSharedPtr last_global_odom_;
    std::set<std::string> received_joints_;
    double last_joint_time_sec_{0.0};
    double last_eval_time_sec_{0.0};

    // Diagnostics snapshot
    double diag_pos_var_{std::numeric_limits<double>::infinity()};
    double diag_yaw_var_{std::numeric_limits<double>::infinity()};
    double diag_speed_{0.0};

    // ROS entities
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr global_odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tf_ready_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr readiness_srv_;
    rclcpp::TimerBase::SharedPtr eval_timer_;
    diagnostic_updater::Updater diagnostic_updater_;
  };

} // namespace lekiwi_motion

#endif // LEKIWI_MOTION__TF_GATEKEEPER_NODE_HPP_
