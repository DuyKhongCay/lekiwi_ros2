// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include "lekiwi_control/workspace_checker_node.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <urdf_parser/urdf_parser.h>

namespace lekiwi_control
{

  namespace
  {
    using FeasibilityResponse = lekiwi_interfaces::srv::CheckMoveFeasibility::Response;
    static_assert(workspace::PLAN_ZERO_NAV == FeasibilityResponse::PLAN_ZERO_NAV);
    static_assert(workspace::PLAN_SINGLE_BASE == FeasibilityResponse::PLAN_SINGLE_BASE);
    static_assert(workspace::PLAN_DUAL_BASE == FeasibilityResponse::PLAN_DUAL_BASE);

    template <typename T>
    T declare_fixed(rclcpp::Node &node, const std::string &name, const T &value, const std::string &desc)
    {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.read_only = true;
      descriptor.description = desc;
      return node.declare_parameter<T>(name, value, descriptor);
    }

    double declare_number(
        rclcpp::Node &node, const std::string &name, double value,
        double minimum, double maximum, const std::string &desc)
    {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.read_only = true;
      descriptor.description = desc;
      rcl_interfaces::msg::FloatingPointRange range;
      range.from_value = minimum;
      range.to_value = maximum;
      descriptor.floating_point_range.push_back(range);
      double result = node.declare_parameter<double>(name, value, descriptor);
      if (!std::isfinite(result) || result < minimum || result > maximum)
      {
        throw std::invalid_argument("Invalid parameter: " + name);
      }
      return result;
    }

    double extract_planar_yaw(const geometry_msgs::msg::Quaternion &rot, double tolerance)
    {
      double norm = std::sqrt(rot.x * rot.x + rot.y * rot.y + rot.z * rot.z + rot.w * rot.w);
      if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3)
      {
        throw std::runtime_error("TF quaternion must be normalized");
      }
      double roll = std::atan2(2.0 * (rot.w * rot.x + rot.y * rot.z), 1.0 - 2.0 * (rot.x * rot.x + rot.y * rot.y));
      double pitch = std::asin(std::max(-1.0, std::min(1.0, 2.0 * (rot.w * rot.y - rot.z * rot.x))));
      if (std::max(std::abs(roll), std::abs(pitch)) > tolerance)
      {
        throw std::runtime_error("TF rotation exceeds the planar model tilt tolerance");
      }
      return std::atan2(2.0 * (rot.w * rot.z + rot.x * rot.y), 1.0 - 2.0 * (rot.y * rot.y + rot.z * rot.z));
    }
  } // namespace

  WorkspaceCheckerNode::WorkspaceCheckerNode(const rclcpp::NodeOptions &options)
      : Node("workspace_checker", options)
  {
    load_parameters();
    init_workspace_bounds();
    init_kinematics();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    service_ = create_service<lekiwi_interfaces::srv::CheckMoveFeasibility>(
        "workspace/check_move_feasibility",
        std::bind(&WorkspaceCheckerNode::handle_check_move_feasibility, this,
                  std::placeholders::_1, std::placeholders::_2));

    diag_updater_ = std::make_unique<diagnostic_updater::Updater>(this);
    diag_updater_->setHardwareID("lekiwi_workspace_checker");
    diag_updater_->add("Workspace Checker Feasibility", this, &WorkspaceCheckerNode::produce_diagnostics);

    RCLCPP_INFO(get_logger(), "WorkspaceCheckerNode (Lean) initialized successfully");
  }

  void WorkspaceCheckerNode::load_parameters()
  {
    map_frame_ = declare_fixed<std::string>(*this, "map_frame", "map", "Navigation output frame");
    board_frame_ = declare_fixed<std::string>(*this, "board_frame", "chessboard_frame", "Chessboard reference frame");
    base_frame_ = declare_fixed<std::string>(*this, "base_frame", "base_footprint", "URDF mobile base footprint frame");
    tip_frame_ = declare_fixed<std::string>(*this, "tip_frame", "gripperframe", "Arm TCP link");

    for (const auto &frame : {map_frame_, board_frame_, base_frame_, tip_frame_})
    {
      if (frame.empty() || frame.front() == '/' || frame.find_first_of(" \t\n") != std::string::npos)
      {
        throw std::invalid_argument("Frame names must be nonempty and contain no leading slash or whitespace");
      }
    }

    max_tf_age_sec_ = declare_number(*this, "tf.max_age_sec", 0.30, 1e-6, 60.0, "Maximum TF age in seconds");
    planar_tolerance_rad_ = declare_number(*this, "tf.planar_tolerance_rad", 0.01, 1e-6, 0.1, "Maximum planar tilt tolerance (rad)");

    // Board dimensions
    declare_fixed<std::vector<double>>(*this, "board_size", {0.390, 0.390}, "Board width and height in meters [w, h]");
    declare_number(*this, "board.w", 0.390, 1e-6, 10.0, "Legacy board width fallback");
    declare_number(*this, "board.h", 0.390, 1e-6, 10.0, "Legacy board height fallback");

    // Optional tag arrays fallback for backwards compatibility
    declare_fixed<std::vector<double>>(*this, "tag_positions_x", {}, "Optional tag X positions");
    declare_fixed<std::vector<double>>(*this, "tag_positions_y", {}, "Optional tag Y positions");

    // Arm & Planning parameters
    declare_fixed<std::string>(*this, "robot_description", "", "URDF XML string if provided as parameter");
    arm_joint_names_ = declare_fixed<std::vector<std::string>>(
        *this, "kinematics.joint_names",
        {"arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex", "arm_wrist_flex", "arm_wrist_roll"},
        "Five joints in base-to-TCP order");

    if (arm_joint_names_.size() != 5)
    {
      throw std::invalid_argument("kinematics.joint_names requires exactly five joint names");
    }

    safety_margin_rad_ = declare_number(*this, "planning.safety_margin_rad", 0.05, 0.0, M_PI_4, "Joint limit safety margin (rad)");
    workspace_.edge_clearance = declare_number(*this, "planning.edge_clearance", 0.147, 1e-6, 5.0, "Standoff clearance from board edge (m)");
    workspace_.sample_step = declare_number(*this, "planning.sample_step", 0.025, 1e-4, 1.0, "Candidate standoff search step (m)");

    rcl_interfaces::msg::ParameterDescriptor sample_desc;
    sample_desc.read_only = true;
    sample_desc.description = "Maximum candidate evaluations per move query";
    rcl_interfaces::msg::IntegerRange sample_range;
    sample_range.from_value = 1;
    sample_range.to_value = 1001;
    sample_desc.integer_range.push_back(sample_range);
    workspace_.max_samples = static_cast<int>(declare_parameter<int64_t>("planning.max_samples", 257, sample_desc));

    workspace_.default_pitch = declare_number(*this, "planning.default_pitch", -M_PI_2, -M_PI, M_PI, "Approach pitch (rad)");
    workspace_.default_roll = declare_number(*this, "planning.default_roll", 0.0, -M_PI, M_PI, "Approach roll (rad)");
  }

  void WorkspaceCheckerNode::init_workspace_bounds()
  {
    auto size = get_parameter("board_size").as_double_array();
    if (size.size() == 2 && size[0] > 0.0 && size[1] > 0.0)
    {
      workspace_.half_w = size[0] * 0.5;
      workspace_.half_h = size[1] * 0.5;
    }
    else
    {
      workspace_.half_w = get_parameter("board.w").as_double() * 0.5;
      workspace_.half_h = get_parameter("board.h").as_double() * 0.5;
    }

    if (!workspace_.is_valid())
    {
      throw std::invalid_argument("Invalid workspace configuration bounds");
    }

    RCLCPP_INFO(get_logger(), "Workspace bounds: half_w=%.4f m, half_h=%.4f m (envelope: %.3f x %.3f m)",
                workspace_.half_w, workspace_.half_h, workspace_.half_w * 2.0, workspace_.half_h * 2.0);
  }

  void WorkspaceCheckerNode::init_kinematics()
  {
    std::string urdf_xml = get_parameter("robot_description").as_string();
    if (!urdf_xml.empty())
    {
      accept_robot_description(urdf_xml, "parameter");
      return;
    }

    rclcpp::QoS qos_profile(1);
    qos_profile.transient_local();
    qos_profile.reliable();

    robot_desc_sub_ = create_subscription<std_msgs::msg::String>(
        "robot_description", qos_profile,
        std::bind(&WorkspaceCheckerNode::on_robot_description, this, std::placeholders::_1));
  }

  void WorkspaceCheckerNode::on_robot_description(const std_msgs::msg::String::ConstSharedPtr msg)
  {
    if (msg)
    {
      accept_robot_description(msg->data, "topic");
    }
  }

  void WorkspaceCheckerNode::accept_robot_description(const std::string &xml, const std::string &source)
  {
    model_source_ = source;
    kinematics_.reset();

    try
    {
      auto urdf_model = urdf::parseURDF(xml);
      if (!urdf_model)
      {
        throw std::runtime_error("Failed to parse robot_description XML");
      }

      workspace::KinematicsModel model;
      std::string err;
      if (!workspace::extract_kinematics_from_urdf(*urdf_model, base_frame_, tip_frame_,
                                                   arm_joint_names_, model, err, safety_margin_rad_))
      {
        throw std::runtime_error("Kinematics extraction failed: " + err);
      }

      kinematics_ = model;
      model_status_ = "URDF model ready: " + base_frame_ + " -> " + tip_frame_;
      RCLCPP_INFO(get_logger(), "%s (source: %s, reach: %.3f m)",
                  model_status_.c_str(), source.c_str(), model.reach_bound);
    }
    catch (const std::exception &e)
    {
      model_status_ = e.what();
      RCLCPP_ERROR(get_logger(), "Kinematics unavailable: %s", e.what());
    }
    last_feasible_ = kinematics_.has_value();
    last_status_ = model_status_;
  }

  workspace::Point3D WorkspaceCheckerNode::transform_point(
      const geometry_msgs::msg::Point &pt,
      const std::string &src_frame,
      const rclcpp::Time &stamp)
  {
    if (src_frame == board_frame_)
    {
      return workspace::Point3D{pt.x, pt.y, pt.z};
    }

    geometry_msgs::msg::PointStamped in_pt;
    in_pt.header.frame_id = src_frame;
    in_pt.header.stamp = stamp;
    in_pt.point = pt;

    auto tf = tf_buffer_->lookupTransform(board_frame_, src_frame, stamp);
    geometry_msgs::msg::PointStamped out_pt;
    tf2::doTransform(in_pt, out_pt, tf);
    return workspace::Point3D{out_pt.point.x, out_pt.point.y, out_pt.point.z};
  }

  geometry_msgs::msg::PoseStamped WorkspaceCheckerNode::make_pose_stamped(
      const workspace::BasePose &base,
      const geometry_msgs::msg::TransformStamped &board_to_map,
      const rclcpp::Time &stamp)
  {
    geometry_msgs::msg::PoseStamped board_pose;
    board_pose.header.frame_id = board_frame_;
    board_pose.header.stamp = stamp;
    board_pose.pose.position.x = base.x;
    board_pose.pose.position.y = base.y;
    board_pose.pose.position.z = base.z;
    board_pose.pose.orientation.z = std::sin(base.yaw * 0.5);
    board_pose.pose.orientation.w = std::cos(base.yaw * 0.5);

    geometry_msgs::msg::PoseStamped map_pose;
    tf2::doTransform(board_pose, map_pose, board_to_map);
    map_pose.header.stamp = stamp;
    return map_pose;
  }

  sensor_msgs::msg::JointState WorkspaceCheckerNode::make_joint_state(
      const std::array<double, 5> &joints,
      const rclcpp::Time &stamp)
  {
    sensor_msgs::msg::JointState msg;
    msg.header.frame_id = base_frame_;
    msg.header.stamp = stamp;
    msg.name.assign(kinematics_->joint_names.begin(), kinematics_->joint_names.end());
    msg.position.assign(joints.begin(), joints.end());
    return msg;
  }

  void WorkspaceCheckerNode::handle_check_move_feasibility(
      const std::shared_ptr<lekiwi_interfaces::srv::CheckMoveFeasibility::Request> request,
      std::shared_ptr<lekiwi_interfaces::srv::CheckMoveFeasibility::Response> response)
  {
    try
    {
      if (!kinematics_)
      {
        throw std::runtime_error(model_status_);
      }

      auto now = get_clock()->now();
      auto base_tf = tf_buffer_->lookupTransform(board_frame_, base_frame_, tf2::TimePointZero);
      rclcpp::Time stamp = base_tf.header.stamp;
      double age = (now - stamp).seconds();

      if (stamp.nanoseconds() == 0 || age < 0.0 || age > max_tf_age_sec_)
      {
        throw std::runtime_error("Base TF is stale (age: " + std::to_string(age) + "s)");
      }

      auto board_to_map = tf_buffer_->lookupTransform(map_frame_, board_frame_, stamp);
      extract_planar_yaw(board_to_map.transform.rotation, planar_tolerance_rad_);

      double base_yaw = extract_planar_yaw(base_tf.transform.rotation, planar_tolerance_rad_);
      double base_z = base_tf.transform.translation.z;

      workspace::BasePose current_base{
          base_tf.transform.translation.x,
          base_tf.transform.translation.y,
          base_z,
          base_yaw};

      std::string src = request->target_frame.empty() ? board_frame_ : request->target_frame;
      workspace::Point3D pick = transform_point(request->pick_point, src, stamp);
      workspace::Point3D place = request->is_capture ? pick : transform_point(request->place_point, src, stamp);
      double pitch = (request->required_pitch_angle != 0.0) ? request->required_pitch_angle : workspace_.default_pitch;

      workspace::PlanningRequest query{pick, place, pitch, request->is_capture};
      workspace::PlanningContext context{current_base};

      workspace::PlanResult plan = workspace::plan_move(query, context, base_z, *kinematics_, workspace_);

      if (!plan.feasible)
      {
        throw std::runtime_error(plan.message);
      }

      response->feasible = true;
      response->plan_type = plan.plan_type;
      response->pick_base_pose = make_pose_stamped(plan.pick_base, board_to_map, stamp);
      response->place_base_pose = make_pose_stamped(plan.place_base, board_to_map, stamp);
      response->pick_ik_solution = make_joint_state(plan.pick_joints, stamp);
      if (plan.place_joints.has_value())
      {
        response->place_ik_solution = make_joint_state(plan.place_joints.value(), stamp);
      }
      response->message = plan.message;
    }
    catch (const std::exception &err)
    {
      response->feasible = false;
      response->message = err.what();
    }

    last_feasible_ = response->feasible;
    last_status_ = response->message;
  }

  void WorkspaceCheckerNode::produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    stat.summary(
        !kinematics_ ? diagnostic_msgs::msg::DiagnosticStatus::ERROR : (last_feasible_ ? diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::WARN),
        kinematics_ ? last_status_ : model_status_);
    stat.add("Kinematics Loaded", kinematics_ ? "true" : "false");
    stat.add("Model Source", model_source_);
    stat.add("Model Status", model_status_);
    stat.add("Base Frame", base_frame_);
    stat.add("TCP Frame", tip_frame_);
    stat.add("Board Half Width (m)", std::to_string(workspace_.half_w));
    stat.add("Board Half Height (m)", std::to_string(workspace_.half_h));
    stat.add("Edge Clearance (m)", std::to_string(workspace_.edge_clearance));
  }

} // namespace lekiwi_control

RCLCPP_COMPONENTS_REGISTER_NODE(lekiwi_control::WorkspaceCheckerNode)
