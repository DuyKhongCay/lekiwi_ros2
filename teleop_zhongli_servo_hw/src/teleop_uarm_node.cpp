/**
 * @file teleop_uarm_node.cpp
 * @brief Implementation of TeleopUarmNode for Zhongli / uArm Leader Arm.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include "teleop_zhongli_servo_hw/teleop_uarm_node.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <yaml-cpp/yaml.h>

namespace teleop_zhongli_servo_hw
{

  TeleopUarmNode::TeleopUarmNode(const rclcpp::NodeOptions &options)
      : Node("teleop_uarm_node", options),
        diagnostic_updater_(this)
  {
    declare_and_load_parameters();

    // Create publisher with SensorDataQoS (BEST_EFFORT, KEEP_LAST depth 5)
    // for direct compatibility with LeRobot and low-latency logging transport.
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        leader_topic_, rclcpp::SensorDataQoS());

    // Pre-allocate joint_state_msg_ and scratch vectors (zero heap allocations in 50 Hz loop)
    joint_state_msg_.header.frame_id = "";
    joint_state_msg_.name.clear();
    joint_state_msg_.name.reserve(joint_configs_.size());
    for (const auto &jcfg : joint_configs_)
    {
      joint_state_msg_.name.push_back(jcfg.joint_name);
    }
    joint_state_msg_.position.assign(joint_configs_.size(), 0.0);
    leader_positions_scratch_.assign(follower_arm_joints_.size(), 0.0);

    // Setup diagnostic updater
    diagnostic_updater_.setHardwareID("uarm_leader_zhongli");
    diagnostic_updater_.add("Serial Connection", this, &TeleopUarmNode::produce_connection_diagnostics);
    diagnostic_updater_.add("Servo Telemetry", this, &TeleopUarmNode::produce_telemetry_diagnostics);
    diagnostic_updater_.add("Publish Frequency", this, &TeleopUarmNode::produce_frequency_diagnostics);

    RCLCPP_INFO(get_logger(), "Initializing uArm Leader Teleop Node on %s (%d baud)",
                port_.c_str(), baudrate_);

    // Initial connection attempt via async bus worker
    connect_hardware();

    // 50 Hz main kinematic polling loop (wait-free read from RealtimeBuffer)
    const auto loop_period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    loop_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(loop_period),
        std::bind(&TeleopUarmNode::loop_callback, this));

    RCLCPP_INFO(get_logger(), "uArm Leader Node started. Publishing %zu joints to %s at %.1f Hz",
                joint_configs_.size(), leader_topic_.c_str(), publish_rate_hz_);
    RCLCPP_INFO(get_logger(), "Follower joints configured: %zu. Kinematic remapping strategies:",
                follower_arm_joints_.size());
    for (const auto &cfg : remapper_.get_configs())
    {
      RCLCPP_INFO(get_logger(), "  [%s] Mode: %s, Leader: [%.3f, %.3f], Follower: [%.3f, %.3f], Scale: %.3f, Offset: %.3f",
                  cfg.joint_name.c_str(),
                  KinematicRemapper::mode_to_string(cfg.mode).c_str(),
                  cfg.leader_min, cfg.leader_max,
                  cfg.follower_min, cfg.follower_max,
                  cfg.scale, cfg.offset);
    }
  }

  TeleopUarmNode::~TeleopUarmNode()
  {
    disconnect_hardware();
  }

  void TeleopUarmNode::setup_default_kinematics()
  {
    servo_ids_ = {0, 1, 2, 3, 4, 5, 6};
    servo_configs_.clear();
    const double default_rad_per_pwm = (270.0 * M_PI / 180.0) / 2000.0;

    for (const uint8_t id : servo_ids_)
    {
      ServoConfig cfg;
      cfg.id = id;
      cfg.min_pwm = 500;
      cfg.max_pwm = 2500;
      cfg.rad_per_pwm = default_rad_per_pwm;
      servo_configs_[id] = cfg;
    }

    // Canonical uArm Leader joint kinematic mapping (unified arm_* naming):
    // - arm_shoulder_pan: servo 0 (sign -1.0)
    // - arm_shoulder_lift: servo 1 (sign 1.0)
    // - arm_elbow_flex: servo 2 (sign 1.0)
    // - arm_wrist_flex: servo 4 (sign -1.0)
    // - arm_wrist_roll: differential pair (servo 5 and servo 3, weights 0.5, signs -1.0)
    // - arm_gripper: servo 6 (sign 1.0)
    joint_configs_ = {
        {"arm_shoulder_pan", {{0, 1.0, -1.0}}, -M_PI, M_PI},
        {"arm_shoulder_lift", {{1, 1.0, 1.0}}, -M_PI, M_PI},
        {"arm_elbow_flex", {{2, 1.0, 1.0}}, -M_PI, M_PI},
        {"arm_wrist_flex", {{4, 1.0, -1.0}}, -M_PI, M_PI},
        {"arm_wrist_roll", {{5, 0.5, -1.0}, {3, 0.5, -1.0}}, -M_PI, M_PI},
        {"arm_gripper", {{6, 1.0, 1.0}}, -M_PI, M_PI},
    };
  }

  bool TeleopUarmNode::load_calibration_file(const std::string &filepath)
  {
    if (filepath.empty())
    {
      return false;
    }
    try
    {
      YAML::Node config = YAML::LoadFile(filepath);
      if (!config || config.IsNull())
      {
        RCLCPP_WARN(get_logger(), "Failed to parse YAML calibration file: %s", filepath.c_str());
        return false;
      }

      // Load physical servo configs
      if (config["servos"])
      {
        const auto &servos_node = config["servos"];
        if (servos_node["ids"] && servos_node["ids"].IsSequence())
        {
          servo_ids_.clear();
          for (const auto &id_node : servos_node["ids"])
          {
            servo_ids_.push_back(static_cast<uint8_t>(id_node.as<int>()));
          }
        }

        for (const auto id : servo_ids_)
        {
          const std::string s_key = "servo_" + std::to_string(id);
          if (servos_node[s_key])
          {
            const auto &s = servos_node[s_key];
            ServoConfig cfg;
            cfg.id = id;
            cfg.min_pwm = s["min_pwm"] ? s["min_pwm"].as<int>() : 500;
            cfg.max_pwm = s["max_pwm"] ? s["max_pwm"].as<int>() : 2500;
            cfg.rad_per_pwm = s["rad_per_pwm"] ? s["rad_per_pwm"].as<double>()
                                               : (270.0 * M_PI / 180.0) / 2000.0;
            servo_configs_[id] = cfg;
          }
        }
      }

      // Load unified kinematic joint & remapping definitions
      if (config["joints"])
      {
        const auto &joints_node = config["joints"];
        std::vector<JointKinematicConfig> new_joints;
        std::vector<JointRemapConfig> remap_configs;

        for (auto it = joints_node.begin(); it != joints_node.end(); ++it)
        {
          const std::string jname = it->first.as<std::string>();
          const auto &jval = it->second;
          if (!jval.IsMap())
            continue;

          // 1. Leader Joint Hardware & Kinematic Limits
          JointKinematicConfig jcfg;
          jcfg.joint_name = jname;

          double l_min = -M_PI;
          double l_max = M_PI;
          if (jval["leader_range"] && jval["leader_range"].IsSequence() && jval["leader_range"].size() == 2)
          {
            l_min = jval["leader_range"][0].as<double>();
            l_max = jval["leader_range"][1].as<double>();
          }
          jcfg.min_limit_rad = l_min;
          jcfg.max_limit_rad = l_max;

          if (jval["sources"] && jval["sources"].IsSequence())
          {
            const auto &sources = jval["sources"];
            const auto &weights = jval["weights"];
            const auto &signs = jval["signs"];
            for (size_t i = 0; i < sources.size(); ++i)
            {
              ServoContribution sc;
              sc.servo_id = static_cast<uint8_t>(sources[i].as<int>());
              sc.weight = (weights && i < weights.size()) ? weights[i].as<double>() : 1.0;
              sc.sign = (signs && i < signs.size()) ? signs[i].as<double>() : 1.0;
              jcfg.sources.push_back(sc);
            }
          }
          new_joints.push_back(jcfg);

          // 2. Follower Kinematic Remapping
          JointRemapConfig rcfg;
          rcfg.joint_name = jname;
          rcfg.leader_min = l_min;
          rcfg.leader_max = l_max;

          if (jval["mode"])
          {
            rcfg.mode = KinematicRemapper::string_to_mode(jval["mode"].as<std::string>());
          }

          if (jval["follower_range"] && jval["follower_range"].IsSequence() && jval["follower_range"].size() == 2)
          {
            rcfg.follower_min = jval["follower_range"][0].as<double>();
            rcfg.follower_max = jval["follower_range"][1].as<double>();
          }

          if (jval["scale"])
            rcfg.scale = jval["scale"].as<double>();
          if (jval["offset"])
            rcfg.offset = jval["offset"].as<double>();

          remap_configs.push_back(rcfg);
        }

        if (!new_joints.empty())
        {
          joint_configs_ = std::move(new_joints);
        }
        if (!remap_configs.empty())
        {
          remapper_.configure(remap_configs);
          RCLCPP_INFO(get_logger(), "Configured %zu unified leader/follower joints from '%s'",
                      remap_configs.size(), filepath.c_str());
        }
      }

      RCLCPP_INFO(get_logger(), "Successfully loaded calibration from '%s'", filepath.c_str());
      return true;
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(get_logger(), "Could not load calibration file '%s': %s (using defaults)",
                  filepath.c_str(), e.what());
      return false;
    }
  }

  void TeleopUarmNode::declare_and_load_parameters()
  {
    port_ = declare_parameter<std::string>("port", "/dev/uarm_leader");
    baudrate_ = declare_parameter<int>("baudrate", default_config::kBaudRate);
    timeout_ms_ = declare_parameter<int>("timeout_ms", default_config::kTimeoutMs);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", default_config::kPublishRateHz);
    command_delay_us_ = declare_parameter<int>("command_delay_us", default_config::kCommandDelayUs);
    leader_topic_ = declare_parameter<std::string>("leader_topic", "/leader/joint_states");
    calibration_file_ = declare_parameter<std::string>("calibration_file", "");

    // Follower Controller & Dispatch Parameters
    DispatcherConfig disp_cfg;
    disp_cfg.arm_mode = declare_parameter<std::string>("arm_mode", "joint_trajectory");
    disp_cfg.follower_jtc_topic = declare_parameter<std::string>(
        "follower_jtc_topic", "/arm_trajectory_controller/joint_trajectory");
    disp_cfg.follower_fwd_topic = declare_parameter<std::string>(
        "follower_fwd_topic", "/arm_forward_controller/commands");
    disp_cfg.point_dt_s = declare_parameter<double>("point_dt_s", 0.04);
    disp_cfg.lpf_alpha = declare_parameter<double>("lpf_alpha", 1.0);
    disp_cfg.follower_joint_names = declare_parameter<std::vector<std::string>>(
        "follower_arm_joints",
        std::vector<std::string>{
            "arm_shoulder_pan",
            "arm_shoulder_lift",
            "arm_elbow_flex",
            "arm_wrist_flex",
            "arm_wrist_roll",
            "arm_gripper"});

    follower_arm_joints_ = disp_cfg.follower_joint_names;

    // 1. Initialise canonical uArm kinematics defaults
    setup_default_kinematics();

    // 2. Default direct 1:1 kinematic remapping
    std::vector<JointRemapConfig> default_configs;
    default_configs.reserve(follower_arm_joints_.size());
    for (const auto &jname : follower_arm_joints_)
    {
      JointRemapConfig cfg;
      cfg.joint_name = jname;
      cfg.mode = RemapMode::DIRECT;
      default_configs.push_back(cfg);
    }
    remapper_.configure(default_configs);

    // 3. Load unified calibration and kinematic remapping from YAML if provided
    if (!calibration_file_.empty())
    {
      load_calibration_file(calibration_file_);
    }

    dispatcher_.init(this, disp_cfg);

    // 4. Strict fail-fast validation of follower-to-leader joint mapping
    if (dispatcher_.get_mode() != DispatchMode::JOINT_STATES_ONLY)
    {
      setup_joint_mapping();
    }
  }

  bool TeleopUarmNode::connect_hardware()
  {
    last_reconnect_attempt_ = now();
    reconnect_attempts_++;

    disconnect_hardware();

    bus_worker_ = std::make_unique<ZhongliBusWorker>();
    std::string err;

    if (!bus_worker_->init(servo_ids_, joint_configs_, servo_configs_, &err))
    {
      RCLCPP_ERROR(get_logger(), "Failed to initialize bus worker: %s", err.c_str());
      connected_ = false;
      return false;
    }

    if (!bus_worker_->configure(port_, baudrate_, timeout_ms_, command_delay_us_, &err))
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Cannot open 1 Mbps serial port '%s': %s (Check udev rules or permissions)",
          port_.c_str(), err.c_str());
      connected_ = false;
      return false;
    }

    if (!bus_worker_->activate(&err))
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Failed to activate async bus worker on '%s': %s", port_.c_str(), err.c_str());
      bus_worker_->cleanup();
      bus_worker_.reset();
      connected_ = false;
      return false;
    }

    connected_ = true;
    RCLCPP_INFO(get_logger(), "Successfully connected to uArm Leader Arm on '%s' at %d baud",
                port_.c_str(), baudrate_);
    return true;
  }

  void TeleopUarmNode::disconnect_hardware()
  {
    if (bus_worker_)
    {
      bus_worker_->deactivate();
      bus_worker_->cleanup();
      bus_worker_.reset();
    }
    connected_ = false;
    dispatcher_.reset();
  }

  void TeleopUarmNode::loop_callback()
  {
    if (!connected_ || !bus_worker_ || !bus_worker_->is_running())
    {
      // Attempt reconnect every 2 seconds
      if ((now() - last_reconnect_attempt_).seconds() >= 2.0)
      {
        connect_hardware();
      }
      return;
    }

    // Wait-free read (< 50 ns) from RealtimeBuffer populated by async worker
    const auto *snapshot = bus_worker_->read_state_rt();

    if (snapshot != nullptr && snapshot->valid)
    {
      // Build and publish JointState
      joint_state_msg_.header.stamp = now();
      joint_state_msg_.position = snapshot->joint_positions_rad;

      joint_pub_->publish(joint_state_msg_);

      // Dispatch real-time commands to follower arm
      if (dispatcher_.get_mode() != DispatchMode::JOINT_STATES_ONLY)
      {
        const size_t num_joints = follower_arm_joints_.size();
        for (size_t i = 0; i < num_joints; ++i)
        {
          leader_positions_scratch_[i] = snapshot->joint_positions_rad[follower_to_leader_idx_[i]];
        }
        const auto follower_positions = remapper_.remap_all(leader_positions_scratch_);
        dispatcher_.dispatch(follower_positions, joint_state_msg_.header.stamp);
      }

      // Update measured publish rate
      const auto current_time = now();
      if (last_publish_time_.nanoseconds() > 0)
      {
        const double dt = (current_time - last_publish_time_).seconds();
        if (dt > 0.001)
        {
          const double instant_rate = 1.0 / dt;
          measured_rate_hz_ = 0.95 * measured_rate_hz_ + 0.05 * instant_rate;
        }
      }
      else
      {
        measured_rate_hz_ = publish_rate_hz_;
      }
      last_publish_time_ = current_time;
    }
  }

  void TeleopUarmNode::setup_joint_mapping()
  {
    std::unordered_map<std::string, int> leader_indices;
    leader_indices.reserve(joint_configs_.size());
    for (size_t i = 0; i < joint_configs_.size(); ++i)
    {
      leader_indices[joint_configs_[i].joint_name] = static_cast<int>(i);
    }

    follower_to_leader_idx_.assign(follower_arm_joints_.size(), -1);
    for (size_t i = 0; i < follower_arm_joints_.size(); ++i)
    {
      const auto &fj = follower_arm_joints_[i];

      auto it = leader_indices.find(fj);
      if (it == leader_indices.end())
      {
        RCLCPP_FATAL(
            get_logger(),
            "Joint mapping error: Joint '%s' was not found in leader kinematics configuration!",
            fj.c_str());
        throw std::invalid_argument(
            "Invalid joint configuration: Joint '" + fj + "' has no matching Leader joint");
      }

      follower_to_leader_idx_[i] = it->second;
    }

    RCLCPP_INFO(
        get_logger(),
        "Direct 1:1 joint mapping validated: %zu joints matched identically between Leader and Follower",
        follower_arm_joints_.size());
  }

  void TeleopUarmNode::produce_connection_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    const bool is_active = (bus_worker_ && bus_worker_->is_running());
    const uint64_t err_count = bus_worker_ ? bus_worker_->consecutive_errors() : 0;
    const uint64_t total = bus_worker_ ? bus_worker_->total_cycles() : 0;
    const uint64_t success = bus_worker_ ? bus_worker_->successful_cycles() : 0;

    if (!is_active)
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Serial worker disconnected");
    }
    else if (err_count > 0)
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Intermittent serial timeout errors");
    }
    else
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Serial connection active (1 Mbps)");
    }

    stat.add("Serial Port", port_);
    stat.add("Baudrate", baudrate_);
    stat.add("Is Connected", is_active ? "true" : "false");
    stat.add("Consecutive Read Errors", err_count);
    stat.add("Reconnect Attempts", reconnect_attempts_);
    stat.add("Total Cycles", total);
    stat.add("Successful Cycles", success);

    const double success_ratio = (total > 0)
                                     ? (100.0 * static_cast<double>(success) / static_cast<double>(total))
                                     : 0.0;
    stat.add("Success Rate (%)", success_ratio);
  }

  void TeleopUarmNode::produce_telemetry_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    if (!bus_worker_ || !bus_worker_->is_running())
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Disconnected; telemetry unavailable");
      return;
    }

    // Read cached telemetry snapshot without touching serial bus
    const auto *snapshot = bus_worker_->read_state_non_rt();
    if (snapshot == nullptr || snapshot->servo_feedbacks.empty())
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Telemetry snapshot pending");
      return;
    }

    double max_temp = 0.0;
    double min_volt = 999.0;
    double max_volt = 0.0;

    for (const auto &fb : snapshot->servo_feedbacks)
    {
      if (fb.temperature_c > max_temp)
      {
        max_temp = fb.temperature_c;
      }
      if (fb.voltage_v > 0.1)
      {
        if (fb.voltage_v < min_volt)
          min_volt = fb.voltage_v;
        if (fb.voltage_v > max_volt)
          max_volt = fb.voltage_v;
      }

      const std::string s_id = std::to_string(fb.id);
      stat.add("Servo " + s_id + " Temp (C)", fb.temperature_c);
      stat.add("Servo " + s_id + " Voltage (V)", fb.voltage_v);
      stat.add("Servo " + s_id + " Raw PWM", fb.raw_pwm);
    }

    if (min_volt > 900.0)
    {
      min_volt = 0.0;
    }

    stat.add("Max Temperature (C)", max_temp);
    stat.add("Min Voltage (V)", min_volt);
    stat.add("Max Voltage (V)", max_volt);

    if (max_temp >= 75.0 || (min_volt > 0.1 && min_volt < 5.0))
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Critical temperature or undervoltage");
    }
    else if (max_temp >= 60.0 || (min_volt > 0.1 && min_volt < 6.0))
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Elevated temperature or low voltage");
    }
    else
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Servo telemetry normal");
    }
  }

  void TeleopUarmNode::produce_frequency_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    stat.add("Target Publish Rate (Hz)", publish_rate_hz_);
    stat.add("Measured Publish Rate (Hz)", measured_rate_hz_);

    if (!connected_ || !bus_worker_ || !bus_worker_->is_running())
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Not publishing (disconnected)");
      return;
    }

    if (std::abs(measured_rate_hz_ - publish_rate_hz_) < 10.0)
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Publish rate nominal");
    }
    else
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Publish rate deviated from target");
    }
  }

} // namespace teleop_zhongli_servo_hw

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<teleop_zhongli_servo_hw::TeleopUarmNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
