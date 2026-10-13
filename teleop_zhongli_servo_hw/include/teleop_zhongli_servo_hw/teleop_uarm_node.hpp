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

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <urdf/model.h>
#include <yaml-cpp/yaml.h>

#include "teleop_zhongli_servo_hw/follower_dispatcher.hpp"
#include "teleop_zhongli_servo_hw/kinematic_remapper.hpp"
#include "teleop_zhongli_servo_hw/zhongli_bus_worker.hpp"
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

    void produce_connection_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void produce_telemetry_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void produce_frequency_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

    void publish_follower_command(
        const std::vector<JointStateData> &joint_data,
        const rclcpp::Time &stamp);
    void setup_joint_mapping();

    // Dynamic Robot Description & Follower Limits
    void on_robot_description(const std_msgs::msg::String::ConstSharedPtr msg);
    bool parse_and_apply_urdf(const std::string &urdf_xml);
    void update_remapping_configuration();

    // Helper methods for configuration and remapping
    void parse_servos_yaml(const YAML::Node &servos_node);
    void parse_joints_yaml(const YAML::Node &joints_node);
    JointRemapConfig build_single_remap_config(
        const std::string &joint_name,
        const std::unordered_map<std::string, const JointKinematicConfig *> &leader_cfg_map) const;

    // Runtime Parameter Configurations (115200 baud defaults)
    std::string port_{"/dev/uarm_leader"};
    int baudrate_{default_config::kBaudRate};
    double publish_rate_hz_{default_config::kPublishRateHz};
    std::string leader_topic_{"/leader/joint_states"};
    std::string calibration_file_{""};

    // Dynamic Follower URDF / Joint Limits Management
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
    std::atomic<bool> follower_limits_ready_{false};
    std::unordered_map<std::string, std::string> custom_remap_modes_;
    std::unordered_map<std::string, std::pair<double, double>> follower_limits_;

    // Follower Dispatcher & Remapping Components
    KinematicRemapper remapper_;
    FollowerDispatcher dispatcher_;
    std::vector<std::string> follower_arm_joints_;
    std::vector<int> follower_to_leader_idx_;

    // Hardware & Kinematic Mapping
    std::vector<uint8_t> servo_ids_;
    std::unordered_map<uint8_t, ServoConfig> servo_configs_;
    std::vector<JointKinematicConfig> joint_configs_;

    // Async Background Bus Worker Engine
    std::unique_ptr<ZhongliBusWorker> bus_worker_;
    bool connected_{false};
    size_t reconnect_attempts_{0};
    rclcpp::Time last_reconnect_attempt_{0, 0, RCL_ROS_TIME};

    // Pre-allocated cache buffers (Zero heap allocation in 50 Hz loop)
    sensor_msgs::msg::JointState joint_state_msg_;
    std::vector<JointStateData> joint_data_scratch_;
    std::vector<double> leader_positions_scratch_;

    // Timing & Metrics
    rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
    double measured_rate_hz_{0.0};

    // ROS 2 Entities
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::TimerBase::SharedPtr loop_timer_;
    diagnostic_updater::Updater diagnostic_updater_;
  };

} // namespace teleop_zhongli_servo_hw
