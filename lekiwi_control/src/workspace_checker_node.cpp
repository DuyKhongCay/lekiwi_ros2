// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include "lekiwi_control/workspace_checker_node.hpp"

#include <algorithm>
#include <cmath>
#include <exception>

#include <rclcpp_components/register_node_macro.hpp>
#include <urdf_parser/urdf_parser.h>

#include "lekiwi_control/workspace/kinematics_engine.hpp"
#include "lekiwi_control/workspace/urdf_kinematics_loader.hpp"
#include "lekiwi_control/workspace/workspace_planner.hpp"

namespace lekiwi_control
{

  namespace
  {
    double extract_planar_yaw(const geometry_msgs::msg::Quaternion &rot, double tolerance)
    {
      double norm = std::sqrt(rot.x * rot.x + rot.y * rot.y + rot.z * rot.z + rot.w * rot.w);
      if (std::abs(norm - 1.0) > 1e-3)
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
        std::bind(
            &WorkspaceCheckerNode::handle_check_move_feasibility, this,
            std::placeholders::_1, std::placeholders::_2));

    diag_updater_ = std::make_unique<diagnostic_updater::Updater>(this);
    diag_updater_->setHardwareID("lekiwi_workspace_checker");
    diag_updater_->add("Workspace Checker Feasibility", this, &WorkspaceCheckerNode::produce_diagnostics);

    RCLCPP_INFO(get_logger(), "WorkspaceCheckerNode (C++) initialized successfully");
  }

  void WorkspaceCheckerNode::load_parameters()
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    // Supports both "board_frame" and "board.frame" for backwards compatibility
    declare_parameter<std::string>("board.frame", "chessboard_frame");
    board_frame_ = declare_parameter<std::string>("board_frame", get_parameter("board.frame").as_string());
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");

    max_tf_age_sec_ = declare_parameter<double>("tf.max_age_sec", 0.30);
    planar_tolerance_rad_ = declare_parameter<double>("tf.planar_tolerance_rad", 0.01);

    declare_parameter<double>("board.w", 0.390);
    declare_parameter<double>("board.h", 0.390);
    declare_parameter<double>("tag_border_margin", 0.0012);
    declare_parameter<std::vector<double>>("tag_positions_x", {-0.1926, 0.1938, 0.1912, -0.1924});
    declare_parameter<std::vector<double>>("tag_positions_y", {-0.1932, -0.1928, 0.1934, 0.1926});

    declare_parameter<std::string>("robot_description", "");

    declare_parameter<double>("planning.edge_clearance", 0.085);
    declare_parameter<double>("planning.sample_step", 0.025);
    declare_parameter<int>("planning.max_samples", 25);
    declare_parameter<double>("planning.default_pitch", -1.57079632679);
    declare_parameter<double>("planning.default_roll", 0.0);
  }

  void WorkspaceCheckerNode::init_workspace_bounds()
  {
    auto xs = get_parameter("tag_positions_x").as_double_array();
    auto ys = get_parameter("tag_positions_y").as_double_array();
    double margin = get_parameter("tag_border_margin").as_double();

    double max_x = 0.0;
    double max_y = 0.0;
    for (double x : xs)
    {
      max_x = std::max(max_x, std::abs(x));
    }
    for (double y : ys)
    {
      max_y = std::max(max_y, std::abs(y));
    }

    if (max_x > 0.01 && max_y > 0.01)
    {
      workspace_.half_w = max_x + margin;
      workspace_.half_h = max_y + margin;
    }
    else
    {
      workspace_.half_w = get_parameter("board.w").as_double() / 2.0;
      workspace_.half_h = get_parameter("board.h").as_double() / 2.0;
    }

    workspace_.edge_clearance = get_parameter("planning.edge_clearance").as_double();
    workspace_.sample_step = get_parameter("planning.sample_step").as_double();
    workspace_.max_samples = static_cast<int>(get_parameter("planning.max_samples").as_int());
    workspace_.default_pitch = get_parameter("planning.default_pitch").as_double();
    workspace_.default_roll = get_parameter("planning.default_roll").as_double();

    RCLCPP_INFO(
        get_logger(),
        "Workspace bounds deduced from calib tags: half_w=%.4f m, half_h=%.4f m (envelope: %.3f x %.3f m)",
        workspace_.half_w, workspace_.half_h, workspace_.half_w * 2.0, workspace_.half_h * 2.0);
  }

  void WorkspaceCheckerNode::init_kinematics()
  {
    // Default LeKiwi baseline
    kinematics_ = workspace::get_default_lekiwi_kinematics();
    kinematics_loaded_ = true;

    std::string urdf_xml = get_parameter("robot_description").as_string();
    if (!urdf_xml.empty())
    {
      auto model = urdf::parseURDF(urdf_xml);
      if (model)
      {
        try
        {
          kinematics_ = workspace::extract_kinematics_from_urdf(*model);
          RCLCPP_INFO(get_logger(), "Successfully loaded arm kinematics directly from robot_description parameter");
          return;
        }
        catch (const std::exception &e)
        {
          RCLCPP_WARN(get_logger(), "Failed to extract kinematics from robot_description: %s. Using default LeKiwi model.", e.what());
        }
      }
    }

    // Subscribe to /robot_description with TRANSIENT_LOCAL durability to update dynamically
    rclcpp::QoS qos_profile(1);
    qos_profile.transient_local();
    qos_profile.reliable();

    robot_desc_sub_ = create_subscription<std_msgs::msg::String>(
        "/robot_description", qos_profile,
        std::bind(&WorkspaceCheckerNode::on_robot_description, this, std::placeholders::_1));
  }

  void WorkspaceCheckerNode::on_robot_description(const std_msgs::msg::String::ConstSharedPtr msg)
  {
    if (!msg || msg->data.empty())
    {
      return;
    }
    auto model = urdf::parseURDF(msg->data);
    if (!model)
    {
      RCLCPP_WARN(get_logger(), "Failed to parse URDF model from /robot_description topic");
      return;
    }
    try
    {
      kinematics_ = workspace::extract_kinematics_from_urdf(*model);
      RCLCPP_INFO(get_logger(), "Updated arm kinematics from /robot_description topic");
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(get_logger(), "Failed to extract kinematics from /robot_description topic: %s", e.what());
    }
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
    board_pose.pose.orientation.z = std::sin(base.yaw / 2.0);
    board_pose.pose.orientation.w = std::cos(base.yaw / 2.0);

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
    msg.name = kinematics_.joint_names;
    msg.position.assign(joints.begin(), joints.end());
    return msg;
  }

  void WorkspaceCheckerNode::handle_check_move_feasibility(
      const std::shared_ptr<lekiwi_interfaces::srv::CheckMoveFeasibility::Request> request,
      std::shared_ptr<lekiwi_interfaces::srv::CheckMoveFeasibility::Response> response)
  {
    try
    {
      auto now = get_clock()->now();
      auto base_tf = tf_buffer_->lookupTransform(board_frame_, base_frame_, tf2::TimePointZero);
      rclcpp::Time stamp = base_tf.header.stamp;
      double age = (now - stamp).seconds();

      if (stamp.nanoseconds() == 0 || age < 0.0 || age > max_tf_age_sec_)
      {
        throw std::runtime_error("Current base TF is stale or future-dated (age: " + std::to_string(age) + "s)");
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

      workspace::PlanResult plan = workspace::plan_move(query, context, base_z, kinematics_, workspace_);

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
        last_feasible_ ? diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::WARN,
        last_status_);
    stat.add("Kinematics Loaded", kinematics_loaded_ ? "true" : "false");
    stat.add("Board Half Width (m)", std::to_string(workspace_.half_w));
    stat.add("Board Half Height (m)", std::to_string(workspace_.half_h));
    stat.add("Edge Clearance (m)", std::to_string(workspace_.edge_clearance));
  }

} // namespace lekiwi_control

RCLCPP_COMPONENTS_REGISTER_NODE(lekiwi_control::WorkspaceCheckerNode)
