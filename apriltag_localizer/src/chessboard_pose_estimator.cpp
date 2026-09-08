/**
 * @file chessboard_pose_estimator.cpp
 * @brief Implementation of AprilTag chessboard pose estimation, TF broadcasting, and map anchor locking.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include "apriltag_localizer/chessboard_pose_estimator.hpp"

#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

#include <sensor_msgs/image_encodings.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace apriltag_localizer
{

  std::vector<cv::Point3d> PoseSolver::compute_tag_corners(
      const cv::Point3d &center, double size, double yaw_rad)
  {
    const double h = size / 2.0;
    // ArUco corner ordering: top-left, top-right, bottom-right, bottom-left
    const std::vector<std::pair<double, double>> local_pts = {
        {-h, h},
        {h, h},
        {h, -h},
        {-h, -h}};

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
    cv::Mat R_cam_board = R_cam_tag * R_board_tag.t();
    cv::Mat t_cam_board = tvec_tag - R_cam_board * t_board_tag;

    cv::Rodrigues(R_cam_board, rvec_board);
    tvec_board = t_cam_board.clone();
  }

  bool PoseSolver::estimate_board_pose(
      const std::vector<std::vector<cv::Point2f>> &marker_corners,
      const std::vector<int> &marker_ids,
      const std::map<int, TagConfig> &tag_configs,
      const cv::Mat &camera_mat,
      const cv::Mat &dist_coeffs,
      cv::Mat &rvec,
      cv::Mat &tvec,
      int &used_tags_cnt)
  {
    used_tags_cnt = 0;
    if (marker_corners.empty() || marker_ids.empty() || camera_mat.empty())
    {
      return false;
    }

    std::vector<cv::Point3d> object_points;
    std::vector<cv::Point2d> image_points;

    for (size_t i = 0; i < marker_ids.size(); ++i)
    {
      const int tag_id = marker_ids[i];
      auto it = tag_configs.find(tag_id);
      if (it != tag_configs.end() && it->second.corners_board.size() == 4U && marker_corners[i].size() == 4U)
      {
        const auto &cfg = it->second;
        for (size_t k = 0; k < 4U; ++k)
        {
          object_points.push_back(cfg.corners_board[k]);
          image_points.emplace_back(marker_corners[i][k].x, marker_corners[i][k].y);
        }
        used_tags_cnt++;
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
          object_points, image_points, camera_mat, dist_coeffs,
          rvec, tvec, false, cv::SOLVEPNP_SQPNP);
    }
    catch (const cv::Exception &)
    {
      success = false;
    }

    if (!success)
    {
      success = cv::solvePnP(
          object_points, image_points, camera_mat, dist_coeffs,
          rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
    }

    if (success && tvec.rows == 3 && tvec.cols == 1)
    {
      return (tvec.at<double>(2) > 0.01);
    }
    return success;
  }

  // ================= ChessboardPoseEstimator Node =================

  ChessboardPoseEstimator::ChessboardPoseEstimator(const rclcpp::NodeOptions &options)
      : rclcpp::Node("chessboard_pose_estimator", options)
  {
    load_parameters();
    init_detector();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    if (calib_)
    {
      // Calibration mode: ONLY publish detected tags to /tag_detections
      tag_detections_pub_ = create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>(
          "/tag_detections", rclcpp::SensorDataQoS());
      RCLCPP_INFO(get_logger(), "ChessboardPoseEstimator running in CALIBRATION MODE (only /tag_detections is active)");
    }
    else
    {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
      static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

      // Publishers
      robot_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/chessboard/robot_pose", rclcpp::QoS(10));

      // Services
      lock_anchor_srv_ = create_service<std_srvs::srv::Trigger>(
          "/chessboard/lock_anchor",
          std::bind(&ChessboardPoseEstimator::on_lock_anchor, this, std::placeholders::_1, std::placeholders::_2));
      reset_anchor_srv_ = create_service<std_srvs::srv::Trigger>(
          "/chessboard/reset_anchor",
          std::bind(&ChessboardPoseEstimator::on_reset_anchor, this, std::placeholders::_1, std::placeholders::_2));

      // Periodic timer for Static TF and map->odom anchor broadcaster (5 Hz)
      tf_timer_ = create_wall_timer(
          std::chrono::milliseconds(200),
          std::bind(&ChessboardPoseEstimator::publish_static_and_anchor_tf, this));

      // Publish initial static transform immediately
      publish_static_and_anchor_tf();

      RCLCPP_INFO(get_logger(), "Services ready: /chessboard/lock_anchor and /chessboard/reset_anchor");
    }

    // Subscriptions
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "~/camera_info",
        rclcpp::SensorDataQoS(),
        std::bind(&ChessboardPoseEstimator::on_camera_info, this, std::placeholders::_1));

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "~/image_raw",
        rclcpp::SensorDataQoS(),
        std::bind(&ChessboardPoseEstimator::on_image, this, std::placeholders::_1));

    camera_mode_sub_ = create_subscription<lekiwi_interfaces::msg::CameraMode>(
        "~/camera_mode",
        rclcpp::QoS(1),
        std::bind(&ChessboardPoseEstimator::on_camera_mode, this, std::placeholders::_1));

    // Diagnostics configuration
    diagnostic_updater_.setHardwareID("apriltag_localizer");
    diagnostic_updater_.add("Chessboard Pose Estimator Status", this, &ChessboardPoseEstimator::produce_diagnostics);
    last_fps_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(get_logger(), "ChessboardPoseEstimator initialized (Family: %s, Tag size: %.3fm, Rate: %.1fHz, Calib: %s)",
                tag_family_.c_str(), tag_size_, detection_rate_hz_, calib_ ? "true" : "false");
  }

  void ChessboardPoseEstimator::init_detector()
  {
    // Static lookup table without dynamic memory allocation (supporting 16h5 and 36h11)
    struct TagDictEntry
    {
      std::string_view name;
      cv::aruco::PREDEFINED_DICTIONARY_NAME dict;
    };
    static constexpr TagDictEntry kDicts[] = {
        {"16h5", cv::aruco::DICT_APRILTAG_16h5},
        {"tag16h5", cv::aruco::DICT_APRILTAG_16h5},
        {"36h11", cv::aruco::DICT_APRILTAG_36h11},
        {"tag36h11", cv::aruco::DICT_APRILTAG_36h11}};

    cv::aruco::PREDEFINED_DICTIONARY_NAME selected_dict = cv::aruco::DICT_APRILTAG_16h5;
    for (const auto &entry : kDicts)
    {
      if (tag_family_ == entry.name)
      {
        selected_dict = entry.dict;
        break;
      }
    }

    aruco_dict_ = cv::aruco::getPredefinedDictionary(selected_dict);
    aruco_params_ = cv::aruco::DetectorParameters::create();
    aruco_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
  }

  void ChessboardPoseEstimator::load_parameters()
  {
    calib_ = declare_parameter<bool>("calib", false);
    tag_family_ = declare_parameter<std::string>("tag_family", "16h5");
    tag_size_ = declare_parameter<double>("tag_size", 0.02);
    detection_rate_hz_ = declare_parameter<double>("detection_rate_hz", 2.0);

    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    chessboard_frame_ = declare_parameter<std::string>("chessboard_frame", "chessboard_frame");
    camera_frame_ = declare_parameter<std::string>("camera_frame", "stereo_left_optical");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");

    chessboard_pose_in_map_ = declare_parameter<std::vector<double>>(
        "chessboard_pose_in_map", std::vector<double>{0.0, 0.0, 0.004, 0.0, 0.0, 0.0});

    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    publish_map_to_chessboard_ = declare_parameter<bool>("publish_map_to_chessboard", true);
    publish_map_to_odom_ = declare_parameter<bool>("publish_map_to_odom", true);

    // Load tag specifications
    const std::vector<int64_t> ids = declare_parameter<std::vector<int64_t>>(
        "tags.ids", std::vector<int64_t>{0, 1, 2, 3});
    const std::vector<std::string> names = declare_parameter<std::vector<std::string>>(
        "tags.names", std::vector<std::string>{"A1", "H1", "H8", "A8"});
    const std::vector<double> px = declare_parameter<std::vector<double>>(
        "tags.positions_x", std::vector<double>{0.0000, 0.3832, 0.3772, -0.0039});
    const std::vector<double> py = declare_parameter<std::vector<double>>(
        "tags.positions_y", std::vector<double>{0.0000, 0.0037, 0.3885, 0.3850});
    const std::vector<double> pz = declare_parameter<std::vector<double>>(
        "tags.positions_z", std::vector<double>{0.0040, 0.0040, 0.0040, 0.0040});
    const std::vector<double> yaws = declare_parameter<std::vector<double>>(
        "tags.yaws", std::vector<double>{0.0000, 0.0202, 0.0082, 0.0084});

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

  void ChessboardPoseEstimator::on_camera_mode(
      const lekiwi_interfaces::msg::CameraMode::ConstSharedPtr &msg)
  {
    current_camera_mode_ = msg->value;
  }

  void ChessboardPoseEstimator::on_image(
      const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
    // Gating check: Process image only when camera is in CHESS_THINKING mode and not yet anchored (bypassed in calib mode)
    if (!calib_ && (current_camera_mode_ != lekiwi_interfaces::msg::CameraMode::CHESS_THINKING || is_anchored_))
    {
      return;
    }

    if (!has_camera_info_)
    {
      return;
    }

    // Rate limiting: enforce configured detection_rate_hz_
    const auto current_stamp = rclcpp::Time(msg->header.stamp);
    if (detection_rate_hz_ > 0.0 && last_detection_time_.nanoseconds() > 0)
    {
      const double elapsed_sec = (current_stamp - last_detection_time_).seconds();
      if (elapsed_sec < (1.0 / detection_rate_hz_))
      {
        return;
      }
    }
    last_detection_time_ = current_stamp;

    const auto proc_start = std::chrono::steady_clock::now();

    // Calculate End-to-End Latency (ms) from image header timestamp
    const auto now_time = now();
    const double e2e_lat_sec = (now_time - current_stamp).seconds();
    const double e2e_lat_ms = (e2e_lat_sec > 0.0) ? (e2e_lat_sec * 1000.0) : 0.0;
    last_e2e_latency_ms_.store(e2e_lat_ms);

    // Convert ROS Image to OpenCV grayscale cv::Mat
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      if (msg->encoding == sensor_msgs::image_encodings::MONO8)
      {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
      }
      else if (msg->encoding == sensor_msgs::image_encodings::BGR8 ||
               msg->encoding == sensor_msgs::image_encodings::RGB8)
      {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
      }
      else
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
      }
    }
    catch (const cv_bridge::Exception &e)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "cv_bridge exception: %s", e.what());
      return;
    }

    std::vector<std::vector<cv::Point2f>> marker_corners;
    std::vector<int> marker_ids;
    cv::aruco::detectMarkers(cv_ptr->image, aruco_dict_, marker_corners, marker_ids, aruco_params_);

    last_detected_tags_count_.store(static_cast<int>(marker_ids.size()));

    std::ostringstream tag_ids_ss;
    for (size_t i = 0; i < marker_ids.size(); ++i)
    {
      if (i > 0)
      {
        tag_ids_ss << ", ";
      }
      tag_ids_ss << marker_ids[i];
    }
    {
      std::lock_guard<std::mutex> lock(diag_mutex_);
      last_detected_tag_ids_str_ = tag_ids_ss.str().empty() ? "None" : tag_ids_ss.str();
    }

    // If running in calibration mode: ONLY publish detected markers to /tag_detections and return
    if (calib_)
    {
      if (tag_detections_pub_)
      {
        apriltag_msgs::msg::AprilTagDetectionArray det_array_msg;
        det_array_msg.header = msg->header;
        det_array_msg.detections.reserve(marker_ids.size());

        for (size_t i = 0; i < marker_ids.size(); ++i)
        {
          apriltag_msgs::msg::AprilTagDetection det;
          det.id = marker_ids[i];
          det.family = tag_family_;

          double cx = 0.0;
          double cy = 0.0;
          for (size_t k = 0; k < 4 && k < marker_corners[i].size(); ++k)
          {
            det.corners[k].x = static_cast<double>(marker_corners[i][k].x);
            det.corners[k].y = static_cast<double>(marker_corners[i][k].y);
            cx += det.corners[k].x;
            cy += det.corners[k].y;
          }
          det.centre.x = cx / 4.0;
          det.centre.y = cy / 4.0;

          det_array_msg.detections.push_back(det);
        }
        tag_detections_pub_->publish(det_array_msg);
      }

      const auto proc_end = std::chrono::steady_clock::now();
      last_process_time_ms_.store(std::chrono::duration<double, std::milli>(proc_end - proc_start).count());
      frame_counter_.fetch_add(1, std::memory_order_relaxed);
      return;
    }

    if (marker_ids.empty())
    {
      last_used_tags_count_.store(0);
      const auto proc_end = std::chrono::steady_clock::now();
      last_process_time_ms_.store(std::chrono::duration<double, std::milli>(proc_end - proc_start).count());
      frame_counter_.fetch_add(1, std::memory_order_relaxed);
      return;
    }

    cv::Mat rvec, tvec;
    int used_tags = 0;
    const bool ok = PoseSolver::estimate_board_pose(
        marker_corners, marker_ids, tag_configs_, camera_matrix_, dist_coeffs_,
        rvec, tvec, used_tags);

    last_used_tags_count_.store(used_tags);

    if (!ok)
    {
      const auto proc_end = std::chrono::steady_clock::now();
      last_process_time_ms_.store(std::chrono::duration<double, std::milli>(proc_end - proc_start).count());
      frame_counter_.fetch_add(1, std::memory_order_relaxed);
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

    // Robot localization pose estimation: T_map^base = T_map^board * T_board^cam * T_cam^base
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

      // Publish /chessboard/robot_pose
      auto robot_pose_msg = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
      robot_pose_msg->header.stamp = msg->header.stamp;
      robot_pose_msg->header.frame_id = map_frame_;
      tf2::toMsg(T_map_base, robot_pose_msg->pose.pose);

      const double pos_var = (used_tags >= 2) ? 0.0001 : 0.0009;
      const double rot_var = (used_tags >= 2) ? 0.0004 : 0.0025;
      robot_pose_msg->pose.covariance.fill(0.0);
      robot_pose_msg->pose.covariance[0] = pos_var;
      robot_pose_msg->pose.covariance[7] = pos_var;
      robot_pose_msg->pose.covariance[14] = pos_var;
      robot_pose_msg->pose.covariance[21] = rot_var;
      robot_pose_msg->pose.covariance[28] = rot_var;
      robot_pose_msg->pose.covariance[35] = rot_var;
      robot_pose_pub_->publish(std::move(robot_pose_msg));
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_DEBUG(get_logger(), "TF lookup transform failed: %s", ex.what());
    }

    const auto proc_end = std::chrono::steady_clock::now();
    last_process_time_ms_.store(std::chrono::duration<double, std::milli>(proc_end - proc_start).count());
    frame_counter_.fetch_add(1, std::memory_order_relaxed);
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

  void ChessboardPoseEstimator::publish_static_and_anchor_tf()
  {
    if (calib_)
    {
      return;
    }

    // 1. Publish static TF: map -> chessboard_frame
    if (publish_tf_ && publish_map_to_chessboard_ && static_tf_broadcaster_)
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

    // 2. If anchored, keep publishing TF map -> odom
    if (is_anchored_ && publish_tf_ && publish_map_to_odom_ && tf_broadcaster_)
    {
      T_map_odom_locked_.header.stamp = now();
      tf_broadcaster_->sendTransform(T_map_odom_locked_);
    }
  }

  void ChessboardPoseEstimator::produce_diagnostics(
      diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    const auto now_tp = std::chrono::steady_clock::now();
    const auto elapsed_sec = std::chrono::duration<float>(now_tp - last_fps_time_).count();
    const uint64_t current_count = frame_counter_.load(std::memory_order_relaxed);

    if (elapsed_sec >= 0.5F)
    {
      const uint64_t delta_frames = current_count - last_fps_frame_count_;
      current_fps_.store(static_cast<float>(delta_frames) / elapsed_sec);
      last_fps_time_ = now_tp;
      last_fps_frame_count_ = current_count;
    }

    const float fps = current_fps_.load();
    const double e2e_latency_ms = last_e2e_latency_ms_.load();
    const double proc_time_ms = last_process_time_ms_.load();
    const int detected_tags = last_detected_tags_count_.load();
    const int used_tags = last_used_tags_count_.load();
    std::string tag_ids_str;
    {
      std::lock_guard<std::mutex> lock(diag_mutex_);
      tag_ids_str = last_detected_tag_ids_str_;
    }

    // 1. Overall Status Summary
    if (!has_camera_info_)
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Waiting for CameraInfo");
    }
    else if (calib_)
    {
      if (detected_tags > 0)
      {
        stat.summaryf(
            diagnostic_msgs::msg::DiagnosticStatus::OK,
            "Calibration Mode (%d tags detected, streaming to /tag_detections)",
            detected_tags);
      }
      else
      {
        stat.summary(
            diagnostic_msgs::msg::DiagnosticStatus::WARN,
            "Calibration Mode (No tags detected in view)");
      }
    }
    else if (is_anchored_)
    {
      stat.summary(
          diagnostic_msgs::msg::DiagnosticStatus::OK,
          "Anchor Locked (Map->Odom active)");
    }
    else if (current_camera_mode_ != lekiwi_interfaces::msg::CameraMode::CHESS_THINKING)
    {
      stat.summary(
          diagnostic_msgs::msg::DiagnosticStatus::OK,
          "Standby (Camera mode inactive for localization)");
    }
    else if (used_tags > 0)
    {
      stat.summaryf(
          diagnostic_msgs::msg::DiagnosticStatus::OK,
          "Tracking Chessboard (%d tags used, %.1f FPS, %.1f ms E2E)",
          used_tags, fps, e2e_latency_ms);
    }
    else
    {
      stat.summary(
          diagnostic_msgs::msg::DiagnosticStatus::WARN,
          "No chessboard tags detected in view");
    }

    // 2. Telemetry & Metrics
    stat.add("Calibration Mode", calib_ ? "true" : "false");
    stat.addf("Processing FPS", "%.1f", fps);
    stat.addf("Target Detection Rate (Hz)", "%.1f", detection_rate_hz_);
    stat.addf("End-to-End Latency (ms)", "%.2f", e2e_latency_ms);
    stat.addf("Algorithm Processing Time (ms)", "%.2f", proc_time_ms);
    stat.add("Detected Tags Count", detected_tags);
    stat.add("Used Board Tags Count", used_tags);
    stat.add("Detected Tag IDs", tag_ids_str);
    stat.add("Anchor Locked", is_anchored_ ? "true" : "false");
    stat.add("Camera Info Received", has_camera_info_ ? "true" : "false");
    stat.add("Total Frames Processed", current_count);
  }

} // namespace apriltag_localizer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(apriltag_localizer::ChessboardPoseEstimator)
