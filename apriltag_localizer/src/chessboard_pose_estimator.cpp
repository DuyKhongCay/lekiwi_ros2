/**
 * @file chessboard_pose_estimator.cpp
 * @brief Implementation of AprilTag chessboard pose estimation and robot pose publisher.
 *
 * Clean Code refactor: Pure vision component with no EKF coupling or intermediate anchor services.
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
      RCLCPP_INFO(get_logger(), "ChessboardPoseEstimator running in CALIBRATION MODE (only /tag_detections active)");
    }
    else
    {
      static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

      // Vision Output Publishers
      robot_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/chessboard/robot_pose", rclcpp::QoS(10));
      tag_centers_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
          "/chess/tag_centers", rclcpp::QoS(1).transient_local().reliable());

      // Broadcast static transform map -> chessboard_frame once at startup
      publish_static_transforms();
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
        "/system/camera_mode",
        rclcpp::QoS(1).transient_local().reliable(),
        std::bind(&ChessboardPoseEstimator::on_camera_mode, this, std::placeholders::_1));

    // Diagnostics updater
    diagnostic_updater_.setHardwareID("ChessboardPoseEstimator");
    diagnostic_updater_.add("Chessboard Tracker Status", this, &ChessboardPoseEstimator::produce_diagnostics);

    last_fps_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(get_logger(), "ChessboardPoseEstimator pure vision node initialized successfully.");
  }

  void ChessboardPoseEstimator::load_parameters()
  {
    calib_ = declare_parameter<bool>("calib", false);
    tag_family_ = declare_parameter<std::string>("tag_family", "16h5");
    tag_size_ = declare_parameter<double>("tag_size", 0.029);
    detection_rate_hz_ = declare_parameter<double>("detection_rate_hz", 5.0);
    min_tags_cnt_ = declare_parameter<int>("min_tags_cnt", 2);

    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    chessboard_frame_ = declare_parameter<std::string>("chessboard_frame", "chessboard_frame");
    camera_frame_ = declare_parameter<std::string>("camera_frame", "stereo_left_optical");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");

    chessboard_pose_in_map_ = declare_parameter<std::vector<double>>(
        "chessboard_pose_in_map", {0.0, 0.0, 0.004, 0.0, 0.0, 0.0});

    publish_static_tf_ = declare_parameter<bool>("publish_tf", true);

    // Tag layout parameters
    const auto tag_ids = declare_parameter<std::vector<int64_t>>("tags.ids", {0, 1, 2, 3});
    const auto tag_names = declare_parameter<std::vector<std::string>>("tags.names", {"A1", "H1", "H8", "A8"});
    const auto positions_x = declare_parameter<std::vector<double>>("tags.positions_x", {0.0, 0.38, 0.38, 0.0});
    const auto positions_y = declare_parameter<std::vector<double>>("tags.positions_y", {0.0, 0.0, 0.38, 0.38});
    const auto positions_z = declare_parameter<std::vector<double>>("tags.positions_z", {0.004, 0.004, 0.004, 0.004});
    const auto yaws = declare_parameter<std::vector<double>>("tags.yaws", {0.0, 0.0, 0.0, 0.0});

    const size_t n = tag_ids.size();
    if (tag_names.size() != n || positions_x.size() != n ||
        positions_y.size() != n || positions_z.size() != n || yaws.size() != n)
    {
      RCLCPP_FATAL(get_logger(), "Mismatch in tags configuration array sizes!");
      throw std::runtime_error("Tags configuration array sizes mismatch");
    }

    for (size_t i = 0; i < n; ++i)
    {
      TagConfig cfg;
      cfg.id = static_cast<int>(tag_ids[i]);
      cfg.name = tag_names[i];
      cfg.center = cv::Point3d(positions_x[i], positions_y[i], positions_z[i]);
      cfg.yaw = yaws[i];
      cfg.corners_board = PoseSolver::compute_tag_corners(cfg.center, tag_size_, cfg.yaw);
      tag_configs_[cfg.id] = cfg;

      RCLCPP_INFO(
          get_logger(),
          "Configured tag ID %d ('%s') at center=[%.4f, %.4f, %.4f], yaw=%.4f rad",
          cfg.id, cfg.name.c_str(), cfg.center.x, cfg.center.y, cfg.center.z, cfg.yaw);
    }
  }

  void ChessboardPoseEstimator::init_detector()
  {
    if (tag_family_ == "16h5")
    {
      aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_16h5);
    }
    else if (tag_family_ == "36h11")
    {
      aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    }
    else
    {
      RCLCPP_WARN(get_logger(), "Unknown tag family '%s', defaulting to 16h5", tag_family_.c_str());
      aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_16h5);
    }

    aruco_params_ = cv::aruco::DetectorParameters::create();
    aruco_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    aruco_params_->cornerRefinementWinSize = 5;
    aruco_params_->cornerRefinementMaxIterations = 30;
    aruco_params_->cornerRefinementMinAccuracy = 0.05;
  }

  void ChessboardPoseEstimator::publish_static_transforms()
  {
    if (!publish_static_tf_ || !static_tf_broadcaster_)
    {
      return;
    }

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
    RCLCPP_INFO(get_logger(), "Broadcasted static transform: '%s' -> '%s'",
                map_frame_.c_str(), chessboard_frame_.c_str());
  }

  void ChessboardPoseEstimator::on_camera_info(
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg)
  {
    if (has_camera_info_)
    {
      return;
    }

    camera_matrix_ = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i)
    {
      camera_matrix_.at<double>(i / 3, i % 3) = msg->k[i];
    }

    dist_coeffs_ = cv::Mat(msg->d.size(), 1, CV_64F);
    for (size_t i = 0; i < msg->d.size(); ++i)
    {
      dist_coeffs_.at<double>(i, 0) = msg->d[i];
    }

    has_camera_info_ = true;
    RCLCPP_INFO(get_logger(), "CameraInfo received and intrinsics configured (fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f)",
                camera_matrix_.at<double>(0, 0), camera_matrix_.at<double>(1, 1),
                camera_matrix_.at<double>(0, 2), camera_matrix_.at<double>(1, 2));
  }

  void ChessboardPoseEstimator::on_camera_mode(
      const lekiwi_interfaces::msg::CameraMode::ConstSharedPtr &msg)
  {
    if (current_camera_mode_ != msg->value)
    {
      current_camera_mode_ = msg->value;
      RCLCPP_INFO(get_logger(), "Camera mode updated to: %u", current_camera_mode_);
    }
  }

  bool ChessboardPoseEstimator::should_process_image(
      const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
    const bool allow_detection = calib_ ||
                                 (current_camera_mode_ == lekiwi_interfaces::msg::CameraMode::CHESS_THINKING) ||
                                 (current_camera_mode_ == lekiwi_interfaces::msg::CameraMode::STANDBY);
    if (!allow_detection || !has_camera_info_)
    {
      return false;
    }

    // Rate limiting: enforce configured detection_rate_hz_
    const auto current_stamp = rclcpp::Time(msg->header.stamp);
    if (detection_rate_hz_ > 0.0 && last_detection_stamp_.nanoseconds() > 0)
    {
      const double elapsed_sec = (current_stamp - last_detection_stamp_).seconds();
      if (elapsed_sec < (1.0 / detection_rate_hz_))
      {
        return false;
      }
    }
    last_detection_stamp_ = current_stamp;
    return true;
  }

  cv_bridge::CvImageConstPtr ChessboardPoseEstimator::convert_to_grayscale(
      const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
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
      return nullptr;
    }
    return cv_ptr;
  }

  void ChessboardPoseEstimator::detect_tags(
      const cv::Mat &gray_img,
      std::vector<std::vector<cv::Point2f>> &marker_corners,
      std::vector<int> &marker_ids)
  {
    cv::aruco::detectMarkers(gray_img, aruco_dict_, marker_corners, marker_ids, aruco_params_);

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
  }

  void ChessboardPoseEstimator::publish_tag_detections_for_calib(
      const std_msgs::msg::Header &header,
      const std::vector<std::vector<cv::Point2f>> &marker_corners,
      const std::vector<int> &marker_ids)
  {
    if (!tag_detections_pub_)
    {
      return;
    }

    apriltag_msgs::msg::AprilTagDetectionArray det_array_msg;
    det_array_msg.header = header;
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

  void ChessboardPoseEstimator::publish_tag_centers(
      const std_msgs::msg::Header &header,
      const cv::Size &img_size,
      const std::vector<std::vector<cv::Point2f>> &marker_corners,
      const std::vector<int> &marker_ids)
  {
    if (!tag_centers_pub_ || img_size.width <= 0 || img_size.height <= 0)
    {
      return;
    }

    geometry_msgs::msg::PolygonStamped poly_msg;
    poly_msg.header = header;
    poly_msg.polygon.points.reserve(marker_ids.size());

    const double img_w = static_cast<double>(img_size.width);
    const double img_h = static_cast<double>(img_size.height);

    for (size_t i = 0; i < marker_ids.size(); ++i)
    {
      if (marker_corners[i].size() == 4U)
      {
        const double cx = (marker_corners[i][0].x + marker_corners[i][1].x +
                           marker_corners[i][2].x + marker_corners[i][3].x) /
                          4.0;
        const double cy = (marker_corners[i][0].y + marker_corners[i][1].y +
                           marker_corners[i][2].y + marker_corners[i][3].y) /
                          4.0;

        geometry_msgs::msg::Point32 pt;
        pt.x = static_cast<float>(cx / img_w);
        pt.y = static_cast<float>(cy / img_h);
        pt.z = static_cast<float>(marker_ids[i]);
        poly_msg.polygon.points.push_back(pt);
      }
    }

    tag_centers_pub_->publish(poly_msg);
  }

  void ChessboardPoseEstimator::estimate_and_publish_robot_pose(
      const std_msgs::msg::Header &header,
      const std::vector<std::vector<cv::Point2f>> &marker_corners,
      const std::vector<int> &marker_ids)
  {
    if (static_cast<int>(marker_ids.size()) < min_tags_cnt_)
    {
      last_used_tags_.store(0);
      return;
    }

    cv::Mat rvec, tvec;
    int used_tags = min_tags_cnt_;
    const bool ok = PoseSolver::estimate_board_pose(
        marker_corners, marker_ids, tag_configs_, camera_matrix_, dist_coeffs_,
        rvec, tvec, used_tags);

    last_used_tags_.store(used_tags);

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
    const std::string cam_frame = header.frame_id.empty() ? camera_frame_ : header.frame_id;

    // Robot pose estimation: T_map^base = T_map^board * T_board^cam * T_cam^base
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

      // Publish /chessboard/robot_pose with high-confidence fixed covariance (N >= 2)
      auto robot_pose_msg = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
      robot_pose_msg->header.stamp = header.stamp;
      robot_pose_msg->header.frame_id = map_frame_;
      tf2::toMsg(T_map_base, robot_pose_msg->pose.pose);

      constexpr double pos_var = 0.0001;
      constexpr double rot_var = 0.0004;
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
  }

  void ChessboardPoseEstimator::on_image(
      const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
    if (!should_process_image(msg))
    {
      return;
    }

    const auto proc_start = std::chrono::steady_clock::now();

    cv_bridge::CvImageConstPtr cv_ptr = convert_to_grayscale(msg);
    if (!cv_ptr)
    {
      return;
    }

    std::vector<std::vector<cv::Point2f>> marker_corners;
    std::vector<int> marker_ids;
    detect_tags(cv_ptr->image, marker_corners, marker_ids);

    if (calib_)
    {
      publish_tag_detections_for_calib(msg->header, marker_corners, marker_ids);
      const auto proc_end = std::chrono::steady_clock::now();
      last_proc_time_ms_.store(std::chrono::duration<double, std::milli>(proc_end - proc_start).count());
      frame_counter_.fetch_add(1, std::memory_order_relaxed);
      return;
    }

    publish_tag_centers(msg->header, cv_ptr->image.size(), marker_corners, marker_ids);
    estimate_and_publish_robot_pose(msg->header, marker_corners, marker_ids);

    const auto proc_end = std::chrono::steady_clock::now();
    last_proc_time_ms_.store(std::chrono::duration<double, std::milli>(proc_end - proc_start).count());
    frame_counter_.fetch_add(1, std::memory_order_relaxed);
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
    const double proc_time_ms = last_proc_time_ms_.load();
    const int used_tags = last_used_tags_.load();
    std::string tag_ids_str;
    {
      std::lock_guard<std::mutex> lock(diag_mutex_);
      tag_ids_str = last_detected_tag_ids_str_;
    }

    // Status Summary
    if (!has_camera_info_)
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Waiting for CameraInfo");
    }
    else if (calib_)
    {
      stat.summary(
          diagnostic_msgs::msg::DiagnosticStatus::OK,
          "Calibration Mode (streaming to /tag_detections)");
    }
    else if (current_camera_mode_ != lekiwi_interfaces::msg::CameraMode::CHESS_THINKING &&
             current_camera_mode_ != lekiwi_interfaces::msg::CameraMode::STANDBY)
    {
      stat.summary(
          diagnostic_msgs::msg::DiagnosticStatus::OK,
          "Standby (Camera mode inactive for localization)");
    }
    else if (used_tags >= min_tags_cnt_)
    {
      stat.summaryf(
          diagnostic_msgs::msg::DiagnosticStatus::OK,
          "Tracking Chessboard (%d tags used, %.1f FPS, %.1f ms proc)",
          used_tags, fps, proc_time_ms);
    }
    else
    {
      stat.summaryf(
          diagnostic_msgs::msg::DiagnosticStatus::WARN,
          "Insufficient chessboard tags in view (< %d tags)", min_tags_cnt_);
    }

    // Telemetry & Metrics
    stat.add("Calibration Mode", calib_ ? "true" : "false");
    stat.addf("Processing FPS", "%.1f", fps);
    stat.addf("Target Detection Rate (Hz)", "%.1f", detection_rate_hz_);
    stat.addf("Algorithm Processing Time (ms)", "%.2f", proc_time_ms);
    stat.add("Used Board Tags Count", used_tags);
    stat.add("Detected Tag IDs", tag_ids_str);
    stat.add("Camera Info Received", has_camera_info_ ? "true" : "false");
    stat.add("Total Frames Processed", current_count);
  }

} // namespace apriltag_localizer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(apriltag_localizer::ChessboardPoseEstimator)
