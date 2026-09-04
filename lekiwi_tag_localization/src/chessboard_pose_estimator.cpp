/**
 * @file chessboard_pose_estimator.cpp
 * @brief Implementation of AprilTag chessboard pose estimation, TF broadcasting, and map anchor locking.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include "lekiwi_tag_localization/chessboard_pose_estimator.hpp"

#include <cmath>
#include <limits>
#include <utility>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace lekiwi_tag_localization
{

  std::vector<cv::Point3d> PoseSolver::compute_tag_corners(
      const cv::Point3d &center, double size, double yaw_rad)
  {
    const double h = size / 2.0;
    // Local tag coordinates ordered CCW: bottom-left, bottom-right, top-right, top-left
    const std::vector<std::pair<double, double>> local_pts = {
        {-h, -h},
        {h, -h},
        {h, h},
        {-h, h}};

    const double cos_yaw = std::cos(yaw_rad);
    const double sin_yaw = std::sin(yaw_rad);

    std::vector<cv::Point3d> corners;
    corners.reserve(4);
    for (const auto &p : local_pts)
    {
      const double rx = p.first * cos_yaw - p.second * sin_yaw;
      const double ry = p.first * sin_yaw + p.second * cos_yaw;
      corners.emplace_back(center.x + rx, center.y + ry, center.z);
    }
    return corners;
  }

  void PoseSolver::compute_board_from_single_tag(
      const cv::Mat &rvec_tag,
      const cv::Mat &tvec_tag,
      const cv::Point3d &tag_center_board,
      double tag_yaw_board,
      cv::Mat &rvec_board,
      cv::Mat &tvec_board)
  {
    // T_cam^tag = [R_cam_tag | t_cam_tag]
    cv::Mat R_cam_tag;
    cv::Rodrigues(rvec_tag, R_cam_tag);

    // T_board^tag = [R_board_tag | t_board_tag]
    const double cos_yaw = std::cos(tag_yaw_board);
    const double sin_yaw = std::sin(tag_yaw_board);
    cv::Mat R_board_tag = (cv::Mat_<double>(3, 3) << cos_yaw, -sin_yaw, 0.0,
                           sin_yaw, cos_yaw, 0.0,
                           0.0, 0.0, 1.0);
    cv::Mat t_board_tag = (cv::Mat_<double>(3, 1) << tag_center_board.x, tag_center_board.y, tag_center_board.z);

    // T_cam^board = T_cam^tag * (T_board^tag)^-1
    // R_cam_board = R_cam_tag * R_board_tag^T
    // t_cam_board = t_cam_tag - R_cam_board * t_board_tag
    cv::Mat R_cam_board = R_cam_tag * R_board_tag.t();
    cv::Mat t_cam_board = tvec_tag - R_cam_board * t_board_tag;

    cv::Rodrigues(R_cam_board, rvec_board);
    tvec_board = t_cam_board.clone();
  }

  bool PoseSolver::estimate_board_pose(
      const std::vector<apriltag_msgs::msg::AprilTagDetection> &detections,
      const std::map<int, TagConfig> &tag_configs,
      const cv::Mat &camera_matrix,
      const cv::Mat &dist_coeffs,
      cv::Mat &rvec,
      cv::Mat &tvec,
      int &used_tags_count)
  {
    used_tags_count = 0;
    if (detections.empty() || camera_matrix.empty())
    {
      return false;
    }

    std::vector<cv::Point3d> object_points;
    std::vector<cv::Point2d> image_points;

    for (const auto &det : detections)
    {
      auto it = tag_configs.find(det.id);
      if (it != tag_configs.end() && it->second.corners_board.size() == 4U)
      {
        const auto &cfg = it->second;
        for (size_t k = 0; k < 4U; ++k)
        {
          object_points.push_back(cfg.corners_board[k]);
          image_points.emplace_back(det.corners[k].x, det.corners[k].y);
        }
        used_tags_count++;
      }
    }

    if (object_points.size() < 4U)
    {
      return false;
    }

    bool success = false;
    try
    {
      success = cv::solvePnP(
          object_points, image_points, camera_matrix, dist_coeffs,
          rvec, tvec, false, cv::SOLVEPNP_SQPNP);
    }
    catch (const cv::Exception &)
    {
      success = false;
    }

    if (!success)
    {
      success = cv::solvePnP(
          object_points, image_points, camera_matrix, dist_coeffs,
          rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
    }

    if (success && tvec.rows == 3 && tvec.cols == 1)
    {
      return (tvec.at<double>(2) > 0.01);
    }
    return success;
  }

  geometry_msgs::msg::PolygonStamped PoseSolver::create_keepout_polygon(
      const std::string &frame_id,
      const rclcpp::Time &stamp,
      double board_size,
      double margin)
  {
    geometry_msgs::msg::PolygonStamped poly_msg;
    poly_msg.header.stamp = stamp;
    poly_msg.header.frame_id = frame_id;

    // 4 corners of rectangle enclosing board + margin in chessboard_frame
    const double min_x = -margin;
    const double max_x = board_size + margin;
    const double min_y = -margin;
    const double max_y = board_size + margin;

    geometry_msgs::msg::Point32 p1, p2, p3, p4;
    p1.x = static_cast<float>(min_x);
    p1.y = static_cast<float>(min_y);
    p1.z = 0.0F;
    p2.x = static_cast<float>(max_x);
    p2.y = static_cast<float>(min_y);
    p2.z = 0.0F;
    p3.x = static_cast<float>(max_x);
    p3.y = static_cast<float>(max_y);
    p3.z = 0.0F;
    p4.x = static_cast<float>(min_x);
    p4.y = static_cast<float>(max_y);
    p4.z = 0.0F;

    poly_msg.polygon.points.push_back(p1);
    poly_msg.polygon.points.push_back(p2);
    poly_msg.polygon.points.push_back(p3);
    poly_msg.polygon.points.push_back(p4);

    return poly_msg;
  }

  // ================= ChessboardPoseEstimator Node =================

  ChessboardPoseEstimator::ChessboardPoseEstimator(const rclcpp::NodeOptions &options)
      : rclcpp::Node("chessboard_pose_estimator", options)
  {
    load_parameters();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Publishers
    board_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/chessboard/board_pose", rclcpp::QoS(10));
    robot_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/chessboard/robot_pose", rclcpp::QoS(10));
    costmap_polygon_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
        "/chessboard/costmap_polygon", rclcpp::QoS(1).transient_local());

    // Services
    lock_anchor_srv_ = create_service<std_srvs::srv::Trigger>(
        "/chessboard/lock_anchor",
        std::bind(&ChessboardPoseEstimator::on_lock_anchor, this, std::placeholders::_1, std::placeholders::_2));
    reset_anchor_srv_ = create_service<std_srvs::srv::Trigger>(
        "/chessboard/reset_anchor",
        std::bind(&ChessboardPoseEstimator::on_reset_anchor, this, std::placeholders::_1, std::placeholders::_2));

    // Subscriptions
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/cameras/stereo_left/camera_info",
        rclcpp::SensorDataQoS(),
        std::bind(&ChessboardPoseEstimator::on_camera_info, this, std::placeholders::_1));

    tag_detections_sub_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
        "/tag_detections",
        rclcpp::SensorDataQoS(),
        std::bind(&ChessboardPoseEstimator::on_tag_detections, this, std::placeholders::_1));

    // Periodic timer for Keepout Polygon, Static TF, and map->odom broadcaster
    const auto keepout_period = std::chrono::duration<double>(1.0 / keepout_rate_);
    keepout_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(keepout_period),
        std::bind(&ChessboardPoseEstimator::publish_keepout_and_static_tf, this));

    // Publish initial static transform and keepout polygon immediately
    publish_keepout_and_static_tf();

    RCLCPP_INFO(get_logger(), "ChessboardPoseEstimator initialized (Family: %s, Tag size: %.3fm, Board size: %.3fm)",
                tag_family_.c_str(), tag_size_, tag_distance_);
    RCLCPP_INFO(get_logger(), "Services ready: /chessboard/lock_anchor and /chessboard/reset_anchor");
  }

  void ChessboardPoseEstimator::load_parameters()
  {
    tag_family_ = declare_parameter<std::string>("tag_family", "36h11");
    tag_size_ = declare_parameter<double>("tag_size", 0.02);
    tag_distance_ = declare_parameter<double>("tag_distance", 0.38);

    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    chessboard_frame_ = declare_parameter<std::string>("chessboard_frame", "chessboard_frame");
    camera_frame_ = declare_parameter<std::string>("camera_frame", "stereo_left_optical");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");

    chessboard_pose_in_map_ = declare_parameter<std::vector<double>>(
        "chessboard_pose_in_map", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    publish_map_to_chessboard_ = declare_parameter<bool>("publish_map_to_chessboard", true);
    publish_camera_to_board_ = declare_parameter<bool>("publish_camera_to_board", true);
    publish_map_to_odom_ = declare_parameter<bool>("publish_map_to_odom", true);

    keepout_margin_ = declare_parameter<double>("keepout.safety_margin", 0.035);
    keepout_rate_ = declare_parameter<double>("keepout.publish_rate", 2.0);

    // Load tag specifications
    const std::vector<int64_t> ids = declare_parameter<std::vector<int64_t>>(
        "tags.ids", std::vector<int64_t>{1, 4, 3, 6});
    const std::vector<std::string> names = declare_parameter<std::vector<std::string>>(
        "tags.names", std::vector<std::string>{"A1", "H1", "H8", "A8"});
    const std::vector<double> px = declare_parameter<std::vector<double>>(
        "tags.positions_x", std::vector<double>{0.000, 0.380, 0.380, 0.000});
    const std::vector<double> py = declare_parameter<std::vector<double>>(
        "tags.positions_y", std::vector<double>{0.000, 0.000, 0.380, 0.380});
    const std::vector<double> pz = declare_parameter<std::vector<double>>(
        "tags.positions_z", std::vector<double>{0.000, 0.000, 0.000, 0.000});
    const std::vector<double> yaws = declare_parameter<std::vector<double>>(
        "tags.yaws", std::vector<double>{0.0, 0.0, 0.0, 0.0});

    tag_configs_.clear();
    for (size_t i = 0; i < ids.size(); ++i)
    {
      TagConfig cfg;
      cfg.id = static_cast<int>(ids[i]);
      cfg.name = (i < names.size()) ? names[i] : ("Tag_" + std::to_string(cfg.id));
      const double x = (i < px.size()) ? px[i] : 0.0;
      const double y = (i < py.size()) ? py[i] : 0.0;
      const double z = (i < pz.size()) ? pz[i] : 0.0;
      cfg.center = cv::Point3d(x, y, z);
      cfg.yaw = (i < yaws.size()) ? yaws[i] : 0.0;
      cfg.corners_board = PoseSolver::compute_tag_corners(cfg.center, tag_size_, cfg.yaw);
      tag_configs_[cfg.id] = std::move(cfg);
    }
  }

  void ChessboardPoseEstimator::on_camera_info(
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg)
  {
    if (!has_camera_info_)
    {
      camera_matrix_ = (cv::Mat_<double>(3, 3) << msg->k[0], msg->k[1], msg->k[2],
                        msg->k[3], msg->k[4], msg->k[5],
                        msg->k[6], msg->k[7], msg->k[8]);
      dist_coeffs_ = cv::Mat(msg->d).clone();
      has_camera_info_ = true;
      RCLCPP_INFO(get_logger(), "Received camera_info matrix: fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f",
                  msg->k[0], msg->k[4], msg->k[2], msg->k[5]);
    }
  }

  void ChessboardPoseEstimator::on_tag_detections(
      const apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr &msg)
  {
    if (!has_camera_info_ || msg->detections.empty())
    {
      return;
    }

    cv::Mat rvec, tvec;
    int used_tags = 0;
    const bool ok = PoseSolver::estimate_board_pose(
        msg->detections, tag_configs_, camera_matrix_, dist_coeffs_,
        rvec, tvec, used_tags);

    if (!ok)
    {
      return;
    }

    // Convert OpenCV Rodrigues rvec -> tf2::Quaternion
    cv::Mat R_cam_board;
    cv::Rodrigues(rvec, R_cam_board);
    tf2::Matrix3x3 tf_rot(
        R_cam_board.at<double>(0, 0), R_cam_board.at<double>(0, 1), R_cam_board.at<double>(0, 2),
        R_cam_board.at<double>(1, 0), R_cam_board.at<double>(1, 1), R_cam_board.at<double>(1, 2),
        R_cam_board.at<double>(2, 0), R_cam_board.at<double>(2, 1), R_cam_board.at<double>(2, 2));
    tf2::Quaternion q_cam_board;
    tf_rot.getRotation(q_cam_board);

    tf2::Vector3 t_cam_board(
        tvec.at<double>(0),
        tvec.at<double>(1),
        tvec.at<double>(2));

    const tf2::Transform T_cam_board(q_cam_board, t_cam_board);
    const std::string cam_frame = msg->header.frame_id.empty() ? camera_frame_ : msg->header.frame_id;

    // 1. Publish dynamic TF: camera_optical -> chessboard_frame
    if (publish_tf_ && publish_camera_to_board_)
    {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = msg->header.stamp;
      tf_msg.header.frame_id = cam_frame;
      tf_msg.child_frame_id = chessboard_frame_;
      tf_msg.transform = tf2::toMsg(T_cam_board);
      tf_broadcaster_->sendTransform(tf_msg);
    }

    // 2. Publish Board Pose in camera frame
    auto board_pose_msg = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
    board_pose_msg->header.stamp = msg->header.stamp;
    board_pose_msg->header.frame_id = cam_frame;
    tf2::toMsg(T_cam_board, board_pose_msg->pose.pose);

    const double pos_var = (used_tags >= 2) ? 0.0001 : 0.0009;
    const double rot_var = (used_tags >= 2) ? 0.0004 : 0.0025;
    board_pose_msg->pose.covariance[0] = pos_var;
    board_pose_msg->pose.covariance[7] = pos_var;
    board_pose_msg->pose.covariance[14] = pos_var * 2.0;
    board_pose_msg->pose.covariance[21] = rot_var;
    board_pose_msg->pose.covariance[28] = rot_var;
    board_pose_msg->pose.covariance[35] = rot_var;
    board_pose_pub_->publish(std::move(board_pose_msg));

    // 3. Robot localization pose estimation: T_map^base = T_map^board * T_board^cam * T_cam^base
    try
    {
      const auto transform_cam_base = tf_buffer_->lookupTransform(
          cam_frame, base_frame_, tf2::TimePointZero);
      tf2::Transform T_cam_base;
      tf2::fromMsg(transform_cam_base.transform, T_cam_base);

      const tf2::Transform T_board_cam = T_cam_board.inverse();
      const tf2::Transform T_board_base = T_board_cam * T_cam_base;

      tf2::Quaternion q_map_board;
      if (chessboard_pose_in_map_.size() >= 6)
      {
        q_map_board.setRPY(chessboard_pose_in_map_[3], chessboard_pose_in_map_[4], chessboard_pose_in_map_[5]);
      }
      else
      {
        q_map_board.setRPY(0.0, 0.0, 0.0);
      }
      const tf2::Vector3 t_map_board(
          chessboard_pose_in_map_.size() >= 3 ? chessboard_pose_in_map_[0] : 0.0,
          chessboard_pose_in_map_.size() >= 3 ? chessboard_pose_in_map_[1] : 0.0,
          chessboard_pose_in_map_.size() >= 3 ? chessboard_pose_in_map_[2] : 0.0);
      const tf2::Transform T_map_board(q_map_board, t_map_board);

      const tf2::Transform T_map_base = T_map_board * T_board_base;

      // Cache latest detection for anchor locking
      latest_T_map_base_ = T_map_base;
      latest_detection_stamp_ = msg->header.stamp;
      has_latest_tag_detection_ = true;

      auto robot_pose_msg = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
      robot_pose_msg->header.stamp = msg->header.stamp;
      robot_pose_msg->header.frame_id = map_frame_;
      tf2::toMsg(T_map_base, robot_pose_msg->pose.pose);

      robot_pose_msg->pose.covariance.fill(0.0);
      robot_pose_msg->pose.covariance[0] = pos_var;
      robot_pose_msg->pose.covariance[7] = pos_var;
      robot_pose_msg->pose.covariance[14] = pos_var;
      robot_pose_msg->pose.covariance[21] = rot_var;
      robot_pose_msg->pose.covariance[28] = rot_var;
      robot_pose_msg->pose.covariance[35] = rot_var;
      robot_pose_pub_->publish(std::move(robot_pose_msg));

      // Continuous subtle refinement of T_map^odom if anchored and high confidence (>=2 tags)
      if (is_anchored_ && used_tags >= 2)
      {
        try
        {
          const auto transform_odom_base = tf_buffer_->lookupTransform(
              odom_frame_, base_frame_, tf2::TimePointZero);
          tf2::Transform T_odom_base;
          tf2::fromMsg(transform_odom_base.transform, T_odom_base);

          const tf2::Transform T_map_odom = T_map_base * T_odom_base.inverse();
          T_map_odom_locked_.transform = tf2::toMsg(T_map_odom);
        }
        catch (const tf2::TransformException &)
        {
        }
      }
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_DEBUG(get_logger(), "TF lookup transform failed: %s", ex.what());
    }
  }

  void ChessboardPoseEstimator::on_lock_anchor(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (!has_latest_tag_detection_)
    {
      response->success = false;
      response->message = "Cannot lock anchor: No AprilTags detected yet. Please teleop robot to face chessboard.";
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return;
    }

    try
    {
      const auto transform_odom_base = tf_buffer_->lookupTransform(
          odom_frame_, base_frame_, tf2::TimePointZero);
      tf2::Transform T_odom_base;
      tf2::fromMsg(transform_odom_base.transform, T_odom_base);

      // T_map^odom = T_map^base * (T_odom^base)^-1
      const tf2::Transform T_map_odom = latest_T_map_base_ * T_odom_base.inverse();

      T_map_odom_locked_.header.stamp = now();
      T_map_odom_locked_.header.frame_id = map_frame_;
      T_map_odom_locked_.child_frame_id = odom_frame_;
      T_map_odom_locked_.transform = tf2::toMsg(T_map_odom);

      is_anchored_ = true;

      // Broadcast immediately
      if (publish_tf_ && publish_map_to_odom_)
      {
        tf_broadcaster_->sendTransform(T_map_odom_locked_);
      }

      response->success = true;
      response->message = "Anchor locked successfully! Map -> Odom TF is now active.";
      RCLCPP_INFO(get_logger(), ">>> CHESSBOARD ANCHOR LOCKED! Map->Odom TF broadcasting active.");
    }
    catch (const tf2::TransformException &ex)
    {
      response->success = false;
      response->message = std::string("Cannot lock anchor: TF lookup failed: ") + ex.what();
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    }
  }

  void ChessboardPoseEstimator::on_reset_anchor(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    is_anchored_ = false;
    response->success = true;
    response->message = "Anchor reset. Map -> Odom TF broadcast disabled until re-locked.";
    RCLCPP_INFO(get_logger(), "Chessboard anchor reset.");
  }

  void ChessboardPoseEstimator::publish_keepout_and_static_tf()
  {
    // 1. Publish static TF: map -> chessboard_frame
    if (publish_tf_ && publish_map_to_chessboard_)
    {
      geometry_msgs::msg::TransformStamped static_tf;
      static_tf.header.stamp = now();
      static_tf.header.frame_id = map_frame_;
      static_tf.child_frame_id = chessboard_frame_;

      if (chessboard_pose_in_map_.size() >= 6)
      {
        static_tf.transform.translation.x = chessboard_pose_in_map_[0];
        static_tf.transform.translation.y = chessboard_pose_in_map_[1];
        static_tf.transform.translation.z = chessboard_pose_in_map_[2];

        tf2::Quaternion q;
        q.setRPY(chessboard_pose_in_map_[3], chessboard_pose_in_map_[4], chessboard_pose_in_map_[5]);
        static_tf.transform.rotation = tf2::toMsg(q);
      }
      else
      {
        static_tf.transform.rotation.w = 1.0;
      }
      static_tf_broadcaster_->sendTransform(static_tf);
    }

    // 2. Publish Keepout Polygon
    auto poly = PoseSolver::create_keepout_polygon(
        chessboard_frame_, now(), tag_distance_, keepout_margin_);
    costmap_polygon_pub_->publish(poly);

    // 3. If anchored, keep publishing TF map -> odom
    if (is_anchored_ && publish_tf_ && publish_map_to_odom_)
    {
      T_map_odom_locked_.header.stamp = now();
      tf_broadcaster_->sendTransform(T_map_odom_locked_);
    }
  }

} // namespace lekiwi_tag_localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lekiwi_tag_localization::ChessboardPoseEstimator)
