/**
 * @file teleop_uarm_node.hpp
 * @brief ROS 2 Publisher, Teleop Bridge, and Diagnostics Node for uArm Leader Arm.
 *
 * Reads 7 Zhongli serial bus servos, computes SI Radians kinematics (including
 * coupled differential wrist), publishes /leader/joint_states at 50 Hz,
 * and reports health/telemetry via diagnostic_updater to /diagnostics.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "teleop_zhongli_servo_hw/follower_dispatcher.hpp"
#include "teleop_zhongli_servo_hw/kinematic_remapper.hpp"
#include "teleop_zhongli_servo_hw/zhongli_protocol.hpp"
#include "teleop_zhongli_servo_hw/zhongli_types.hpp"

namespace teleop_zhongli_servo_hw
{

  class TeleopUarmNode : public rclcpp::Node
  {
  public:
    explicit TeleopUarmNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~TeleopUarmNode() override;

  private:
    void declare_and_load_parameters();
    void setup_default_kinematics();
    bool load_calibration_file(const std::string &filepath);

    bool connect_hardware();
    void disconnect_hardware();

    void loop_callback();
    void telemetry_diagnostics_callback();

    void produce_connection_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void produce_telemetry_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void produce_frequency_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

    void publish_follower_command(
        const std::vector<JointStateData> &joint_data,
        const rclcpp::Time &stamp);
    void setup_joint_mapping();

    // Runtime Parameter Configurations
    std::string port_{"/dev/uarm_leader"};
    int baudrate_{115200};
    int timeout_ms_{50};
    double publish_rate_hz_{50.0};
    int command_delay_us_{3000};
    std::string leader_topic_{"/leader/joint_states"};
    double diagnostics_rate_hz_{1.0};
    std::string calibration_file_{""};

    // Follower Dispatcher & Remapping Components
    KinematicRemapper remapper_;
    FollowerDispatcher dispatcher_;
    std::vector<std::string> follower_arm_joints_;
    std::vector<int> follower_to_leader_idx_;

    // Hardware & Kinematic Mapping
    std::vector<uint8_t> servo_ids_;
    std::unordered_map<uint8_t, ServoConfig> servo_configs_;
    std::vector<JointKinematicConfig> joint_configs_;

    // Driver & Connection State
    ZhongliProtocol protocol_;
    bool connected_{false};
    size_t reconnect_attempts_{0};
    size_t consecutive_read_failures_{0};
    size_t total_cycles_{0};
    size_t successful_cycles_{0};
    rclcpp::Time last_reconnect_attempt_{0, 0, RCL_ROS_TIME};

    // Telemetry Cache & Timing
    std::unordered_map<uint8_t, ServoFeedback> servo_feedbacks_;
    rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
    double measured_rate_hz_{0.0};

    // ROS 2 Entities
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::TimerBase::SharedPtr loop_timer_;
    rclcpp::TimerBase::SharedPtr telemetry_timer_;
    diagnostic_updater::Updater diagnostic_updater_;
  };

} // namespace teleop_zhongli_servo_hw
