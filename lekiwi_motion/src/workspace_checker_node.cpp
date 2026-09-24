// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include "lekiwi_motion/workspace_checker_node.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <urdf_parser/urdf_parser.h>

namespace lekiwi_motion
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
    grasp_z_ = declare_number(*this, "planning.grasp_z", DEFAULT_PIECE_GRASP_Z_M, 0.001, 0.50, "Grasp z elevation above board (m)");
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

    const double board_w = workspace_.half_w * 2.0;
    const double board_h = workspace_.half_h * 2.0;
    chessboard_mapper_.emplace(board_w, board_h, grasp_z_, true);

    RCLCPP_INFO(get_logger(), "Workspace bounds: half_w=%.4f m, half_h=%.4f m (envelope: %.3f x %.3f m, grasp_z: %.4f m)",
                workspace_.half_w, workspace_.half_h, board_w, board_h, grasp_z_);
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
    try
    {
      auto urdf_model = urdf::parseURDF(xml);
      if (!urdf_model)
      {
        throw std::runtime_error("Failed to parse robot_description XML");
      }

      auto new_model = std::make_shared<workspace::KinematicsModel>();
      std::string err;
      if (!workspace::extract_kinematics_from_urdf(*urdf_model, base_frame_, tip_frame_,
                                                   arm_joint_names_, *new_model, err, safety_margin_rad_))
      {
        throw std::runtime_error("Kinematics extraction failed: " + err);
      }

      auto solver = std::make_shared<workspace::SO101AnalyticalSolver>(*new_model);
      auto new_planner = std::make_shared<workspace::WorkspacePlanner>(new_model, solver, workspace_);

      double reach = solver->reach_bound();
      std::string status = "URDF model ready: " + base_frame_ + " -> " + tip_frame_;
      {
        std::unique_lock<std::shared_mutex> lock(kinematics_mutex_);
        kinematics_model_ = new_model;
        workspace_planner_ = std::move(new_planner);
        model_source_ = source;
        model_status_ = status;
        last_feasible_ = true;
        last_status_ = status;
      }
      RCLCPP_INFO(get_logger(), "%s (source: %s, reach: %.3f m)",
                  status.c_str(), source.c_str(), reach);
    }
    catch (const std::exception &e)
    {
      std::string err_msg = e.what();
      {
        std::unique_lock<std::shared_mutex> lock(kinematics_mutex_);
        kinematics_model_.reset();
        workspace_planner_.reset();
        model_source_ = source;
        model_status_ = err_msg;
        last_feasible_ = false;
        last_status_ = err_msg;
      }
      RCLCPP_ERROR(get_logger(), "Kinematics unavailable: %s", err_msg.c_str());
    }
  }

  geometry_msgs::msg::PoseStamped WorkspaceCheckerNode::make_pose_stamped(
      const workspace::BasePose &base,
      const geometry_msgs::msg::TransformStamped &board_to_map,
      const rclcpp::Time &stamp) const
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

  std::shared_ptr<const workspace::WorkspacePlanner> WorkspaceCheckerNode::get_planner_snapshot() const
  {
    std::shared_lock<std::shared_mutex> lock(kinematics_mutex_);
    return workspace_planner_;
  }

  void WorkspaceCheckerNode::set_error_response(
      lekiwi_interfaces::srv::CheckMoveFeasibility::Response &response,
      workspace::FeasibilityStatus status,
      const std::string &message)
  {
    response.feasible = false;
    response.message = message;
    last_feasible_ = false;
    last_status_ = std::string(workspace::to_string(status)) + ": " + message;
  }

  std::optional<std::string> WorkspaceCheckerNode::validate_request(
      const lekiwi_interfaces::srv::CheckMoveFeasibility::Request &request) const noexcept
  {
    const auto &move = request.move;
    if (move.from_square.empty() || move.to_square.empty())
    {
      return "Malformed request: from_square and to_square cannot be empty";
    }
    if (!ChessboardMapper::is_valid_square(move.from_square))
    {
      return "Malformed request: invalid from_square '" + move.from_square + "'";
    }
    if (!ChessboardMapper::is_valid_square(move.to_square))
    {
      return "Malformed request: invalid to_square '" + move.to_square + "'";
    }
    if (move.is_capture)
    {
      const std::string &cap_sq = move.captured_square.empty() ? move.to_square : move.captured_square;
      if (!ChessboardMapper::is_valid_square(cap_sq))
      {
        return "Malformed request: invalid captured_square '" + cap_sq + "'";
      }
    }
    return std::nullopt;
  }

  std::optional<WorkspaceCheckerNode::TransformContext> WorkspaceCheckerNode::resolve_base_transforms(
      const rclcpp::Time &now,
      workspace::FeasibilityStatus &out_status,
      std::string &out_error) const
  {
    geometry_msgs::msg::TransformStamped base_tf;
    try
    {
      base_tf = tf_buffer_->lookupTransform(
          board_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1));
    }
    catch (const tf2::TransformException &ex)
    {
      out_status = workspace::FeasibilityStatus::TF_STALE;
      out_error = std::string("Base TF lookup failed: ") + ex.what();
      return std::nullopt;
    }

    rclcpp::Time stamp = base_tf.header.stamp;
    double age = (now - stamp).seconds();

    if (stamp.nanoseconds() == 0 || age < -CLOCK_JITTER_TOLERANCE_SEC || age > max_tf_age_sec_)
    {
      out_status = workspace::FeasibilityStatus::TF_STALE;
      out_error = "Base TF is stale (age: " + std::to_string(age) + "s)";
      return std::nullopt;
    }

    geometry_msgs::msg::TransformStamped board_to_map;
    try
    {
      board_to_map = tf_buffer_->lookupTransform(
          map_frame_, board_frame_, stamp, tf2::durationFromSec(0.05));
    }
    catch (const tf2::TransformException &)
    {
      try
      {
        board_to_map = tf_buffer_->lookupTransform(
            map_frame_, board_frame_, tf2::TimePointZero);
      }
      catch (const tf2::TransformException &ex)
      {
        out_status = workspace::FeasibilityStatus::TF_STALE;
        out_error = std::string("Board-to-map TF lookup failed: ") + ex.what();
        return std::nullopt;
      }
    }

    double base_yaw{0.0};
    try
    {
      extract_planar_yaw(board_to_map.transform.rotation, planar_tolerance_rad_);
      base_yaw = extract_planar_yaw(base_tf.transform.rotation, planar_tolerance_rad_);
    }
    catch (const std::exception &ex)
    {
      out_status = workspace::FeasibilityStatus::MALFORMED_REQUEST;
      out_error = ex.what();
      return std::nullopt;
    }

    double base_z = base_tf.transform.translation.z;
    workspace::BasePose current_base{
        base_tf.transform.translation.x,
        base_tf.transform.translation.y,
        base_z,
        base_yaw};

    return TransformContext{base_tf, board_to_map, current_base, base_z, stamp};
  }

  std::optional<WorkspaceCheckerNode::TargetPoints> WorkspaceCheckerNode::resolve_target_points(
      const lekiwi_interfaces::srv::CheckMoveFeasibility::Request &request,
      std::string &out_error)
  {
    TargetPoints targets;
    targets.is_capture = request.move.is_capture;

    try
    {
      if (!chessboard_mapper_)
      {
        throw std::runtime_error("Chessboard mapper not initialized");
      }
      auto pick_coord = chessboard_mapper_->square_to_metric(request.move.from_square);
      auto place_coord = chessboard_mapper_->square_to_metric(request.move.to_square);

      targets.pick = workspace::Point3D{pick_coord.x, pick_coord.y, pick_coord.z};
      targets.place = workspace::Point3D{place_coord.x, place_coord.y, place_coord.z};
      targets.pick_pt_msg = pick_coord.to_point_msg();
      targets.place_pt_msg = place_coord.to_point_msg();

      if (request.move.is_capture)
      {
        const std::string &cap_sq = request.move.captured_square.empty()
                                        ? request.move.to_square
                                        : request.move.captured_square;
        auto clear_coord = chessboard_mapper_->square_to_metric(cap_sq);
        targets.clear = workspace::Point3D{clear_coord.x, clear_coord.y, clear_coord.z};
        targets.clear_pt_msg = clear_coord.to_point_msg();
      }
      else
      {
        targets.clear = targets.place;
        targets.clear_pt_msg = targets.place_pt_msg;
      }
    }
    catch (const std::exception &ex)
    {
      out_error = ex.what();
      return std::nullopt;
    }

    return targets;
  }

  void WorkspaceCheckerNode::populate_success_response(
      const workspace::PlanResult &plan,
      const TargetPoints &targets,
      const TransformContext &tf_ctx,
      lekiwi_interfaces::srv::CheckMoveFeasibility::Response &response) const
  {
    response.feasible = true;
    response.plan_type = plan.plan_type;
    response.clear_base_pose = make_pose_stamped(plan.clear_base, tf_ctx.board_to_map, tf_ctx.stamp);
    response.pick_base_pose = make_pose_stamped(plan.pick_base, tf_ctx.board_to_map, tf_ctx.stamp);
    response.place_base_pose = make_pose_stamped(plan.place_base, tf_ctx.board_to_map, tf_ctx.stamp);
    response.clear_point = targets.clear_pt_msg;
    response.pick_point = targets.pick_pt_msg;
    response.place_point = targets.place_pt_msg;
    response.message = plan.message;
  }

  void WorkspaceCheckerNode::handle_check_move_feasibility(
      const std::shared_ptr<lekiwi_interfaces::srv::CheckMoveFeasibility::Request> request,
      std::shared_ptr<lekiwi_interfaces::srv::CheckMoveFeasibility::Response> response)
  {
    auto planner = get_planner_snapshot();
    if (!planner)
    {
      std::string current_model_status;
      {
        std::shared_lock<std::shared_mutex> lock(kinematics_mutex_);
        current_model_status = model_status_;
      }
      set_error_response(*response, workspace::FeasibilityStatus::MODEL_NOT_READY, current_model_status);
      return;
    }

    if (auto err = validate_request(*request); err.has_value())
    {
      set_error_response(*response, workspace::FeasibilityStatus::MALFORMED_REQUEST, *err);
      return;
    }

    workspace::FeasibilityStatus tf_status{workspace::FeasibilityStatus::TF_STALE};
    std::string tf_err;
    auto tf_ctx = resolve_base_transforms(get_clock()->now(), tf_status, tf_err);
    if (!tf_ctx)
    {
      set_error_response(*response, tf_status, tf_err);
      return;
    }

    std::string target_err;
    auto targets = resolve_target_points(*request, target_err);
    if (!targets)
    {
      set_error_response(*response, workspace::FeasibilityStatus::MALFORMED_REQUEST, target_err);
      return;
    }

    double pitch = workspace_.default_pitch;
    workspace::PlanningRequest query{
        targets->clear, targets->pick, targets->place, pitch, request->move.is_capture};
    workspace::PlanningContext context{tf_ctx->current_base};

    workspace::PlanResult plan = planner->plan(query, context, tf_ctx->base_z);
    if (!plan.feasible)
    {
      set_error_response(*response, plan.status, plan.message);
      return;
    }

    populate_success_response(plan, *targets, *tf_ctx, *response);
    last_feasible_ = true;
    last_status_ = response->message;
  }

  void WorkspaceCheckerNode::produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    std::shared_ptr<const workspace::KinematicsModel> model;
    std::string current_model_status;
    std::string current_model_source;
    bool is_feasible{false};
    std::string status_msg;
    {
      std::shared_lock<std::shared_mutex> lock(kinematics_mutex_);
      model = kinematics_model_;
      current_model_status = model_status_;
      current_model_source = model_source_;
      is_feasible = last_feasible_;
      status_msg = last_status_;
    }

    stat.summary(
        !model ? diagnostic_msgs::msg::DiagnosticStatus::ERROR : (is_feasible ? diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::WARN),
        model ? status_msg : current_model_status);
    stat.add("Kinematics Loaded", model ? "true" : "false");
    stat.add("Model Source", current_model_source);
    stat.add("Model Status", current_model_status);
    stat.add("Base Frame", base_frame_);
    stat.add("TCP Frame", tip_frame_);
    stat.add("Board Half Width (m)", std::to_string(workspace_.half_w));
    stat.add("Board Half Height (m)", std::to_string(workspace_.half_h));
    stat.add("Edge Clearance (m)", std::to_string(workspace_.edge_clearance));
  }

} // namespace lekiwi_motion

RCLCPP_COMPONENTS_REGISTER_NODE(lekiwi_motion::WorkspaceCheckerNode)
