/**
 * @file chessboard_pose_estimator.cpp
 * @brief Implementation of AprilTag chessboard pose estimation as a Lifecycle Component.
 *
 * Clean Code refactor: Pure vision component with no EKF coupling or intermediate anchor services.
 * Integrates PerceptionLifecycleHelper for camera mode gating, autostart, and diagnostics.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include "chessboard_pose_estimator.hpp"

#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace lekiwi_perception
{

  ChessboardPoseEstimator::ChessboardPoseEstimator(const rclcpp::NodeOptions &options)
      : rclcpp_lifecycle::LifecycleNode("chessboard_pose_estimator", options)
  {
    declare_parameter<bool>("autostart", true);
    declare_parameter<bool>("calib", false);
    declare_parameter<std::string>("tag_family", "16h5");
    declare_parameter<double>("tag_size", 0.029);
    declare_parameter<double>("detection_rate_hz", 5.0);
    declare_parameter<int>("min_tags_cnt", 2);

    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("chessboard_frame", "chessboard_frame");
    declare_parameter<std::string>("camera_frame", "stereo_left_optical");
    declare_parameter<std::string>("base_frame", "base_footprint");

    declare_parameter<std::vector<double>>(
        "chessboard_pose_in_map", {0.0, 0.0, 0.004, 0.0, 0.0, 0.0});

    declare_parameter<bool>("publish_tf", true);

    declare_parameter<std::vector<int64_t>>("tags.ids", {0, 1, 2, 3});
    declare_parameter<std::vector<std::string>>("tags.names", {"A1", "H1", "H8", "A8"});
    declare_parameter<std::vector<double>>("tags.positions_x", {0.0, 0.38, 0.38, 0.0});
    declare_parameter<std::vector<double>>("tags.positions_y", {0.0, 0.0, 0.38, 0.38});
    declare_parameter<std::vector<double>>("tags.positions_z", {0.004, 0.004, 0.004, 0.004});
    declare_parameter<std::vector<double>>("tags.yaws", {0.0, 0.0, 0.0, 0.0});

    declare_parameter<std::string>("odom_topic", "/omni_base_controller/odom");
    declare_parameter<double>("odom_timeout_sec", 0.5);
    declare_parameter<double>("covariance.base_pos_var", 0.0001);
    declare_parameter<double>("covariance.base_rot_var", 0.0004);
    declare_parameter<double>("covariance.vel_scale_pos", 0.01);
    declare_parameter<double>("covariance.vel_scale_rot", 0.05);
    declare_parameter<double>("covariance.max_pos_var", 0.01);
    declare_parameter<double>("covariance.max_rot_var", 0.04);
    declare_parameter<bool>("covariance.scale_by_tag_count", true);

    autostart_ = get_parameter("autostart").as_bool();
    if (autostart_)
    {
      lifecycle_helper_ = std::make_unique<utils::PerceptionLifecycleHelper>(
          this, "ChessboardPoseEstimator", "Chessboard Tracker Status");
      lifecycle_helper_->setup_autostart(true);
    }
  }

  ChessboardPoseEstimator::CallbackReturn ChessboardPoseEstimator::on_configure(
      const rclcpp_lifecycle::State & /*state*/)
  {
    try
    {
      load_parameters();
      init_detector();

      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      if (calib_)
      {
        tag_detections_pub_ = create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>(
            "/tag_detections", rclcpp::SensorDataQoS());
        RCLCPP_INFO(get_logger(), "ChessboardPoseEstimator running in CALIBRATION MODE (only /tag_detections active)");
      }
      else
      {
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        robot_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/chessboard/robot_pose", rclcpp::QoS(10));
        tag_centers_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
            "/chess/tag_centers", rclcpp::QoS(1).transient_local().reliable());
      }

      camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
          "~/camera_info",
          rclcpp::SensorDataQoS(),
          std::bind(&ChessboardPoseEstimator::on_camera_info, this, std::placeholders::_1));

      image_sub_ = create_subscription<sensor_msgs::msg::Image>(
          "~/image_raw",
          rclcpp::SensorDataQoS(),
          std::bind(&ChessboardPoseEstimator::on_image, this, std::placeholders::_1));

      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          odom_topic_,
          rclcpp::SensorDataQoS(),
          std::bind(&ChessboardPoseEstimator::on_odometry, this, std::placeholders::_1));

      if (!lifecycle_helper_)
      {
        lifecycle_helper_ = std::make_unique<utils::PerceptionLifecycleHelper>(
            this, "ChessboardPoseEstimator", "Chessboard Tracker Status");
      }

      const std::vector<uint8_t> allowed_modes = {
          lekiwi_interfaces::msg::CameraMode::STANDBY,
          lekiwi_interfaces::msg::CameraMode::CHESS_THINKING};
      lifecycle_helper_->setup_camera_mode_sub(allowed_modes);

      lifecycle_helper_->diagnostics().updater().add(
          "Chessboard Tracker Status", this, &ChessboardPoseEstimator::produce_diagnostics);

      RCLCPP_INFO(get_logger(), "ChessboardPoseEstimator configured successfully.");
      return CallbackReturn::SUCCESS;
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_logger(), "Exception during on_configure: %s", e.what());
      reset_state();
      return CallbackReturn::FAILURE;
    }
  }

  ChessboardPoseEstimator::CallbackReturn ChessboardPoseEstimator::on_activate(
      const rclcpp_lifecycle::State & /*state*/)
  {
    if (tag_detections_pub_)
    {
      tag_detections_pub_->on_activate();
    }
    if (robot_pose_pub_)
    {
      robot_pose_pub_->on_activate();
    }
    if (tag_centers_pub_)
    {
      tag_centers_pub_->on_activate();
    }

    if (!calib_)
    {
      publish_static_transforms();
    }

    if (lifecycle_helper_)
    {
      lifecycle_helper_->perf_tracker().reset();
    }

    RCLCPP_INFO(get_logger(), "ChessboardPoseEstimator activated.");
    return CallbackReturn::SUCCESS;
  }

  ChessboardPoseEstimator::CallbackReturn ChessboardPoseEstimator::on_deactivate(
      const rclcpp_lifecycle::State & /*state*/)
  {
    if (tag_detections_pub_)
    {
      tag_detections_pub_->on_deactivate();
    }
    if (robot_pose_pub_)
    {
      robot_pose_pub_->on_deactivate();
    }
    if (tag_centers_pub_)
    {
      tag_centers_pub_->on_deactivate();
    }

    RCLCPP_INFO(get_logger(), "ChessboardPoseEstimator deactivated.");
    return CallbackReturn::SUCCESS;
  }

  ChessboardPoseEstimator::CallbackReturn ChessboardPoseEstimator::on_cleanup(
      const rclcpp_lifecycle::State & /*state*/)
  {
    reset_state();
    RCLCPP_INFO(get_logger(), "ChessboardPoseEstimator cleaned up.");
    return CallbackReturn::SUCCESS;
  }

  ChessboardPoseEstimator::CallbackReturn ChessboardPoseEstimator::on_shutdown(
      const rclcpp_lifecycle::State & /*state*/)
  {
    reset_state();
    RCLCPP_INFO(get_logger(), "ChessboardPoseEstimator shut down.");
    return CallbackReturn::SUCCESS;
  }

  ChessboardPoseEstimator::CallbackReturn ChessboardPoseEstimator::on_error(
      const rclcpp_lifecycle::State & /*state*/)
  {
    reset_state();
    return CallbackReturn::SUCCESS;
  }

  void ChessboardPoseEstimator::reset_state()
  {
    if (lifecycle_helper_)
    {
      lifecycle_helper_->reset();
      lifecycle_helper_.reset();
    }
    tag_detections_pub_.reset();
    robot_pose_pub_.reset();
    tag_centers_pub_.reset();
    camera_info_sub_.reset();
    image_sub_.reset();
    odom_sub_.reset();
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      latest_odom_.reset();
    }
    tf_listener_.reset();
    tf_buffer_.reset();
    static_tf_broadcaster_.reset();
    has_camera_info_ = false;
    last_used_tags_.store(0);
    odom_received_.store(false);
  }

  void ChessboardPoseEstimator::load_parameters()
  {
    calib_ = get_parameter("calib").as_bool();
    tag_family_ = get_parameter("tag_family").as_string();
    tag_size_ = get_parameter("tag_size").as_double();
    detection_rate_hz_ = get_parameter("detection_rate_hz").as_double();
    min_tags_cnt_ = get_parameter("min_tags_cnt").as_int();

    map_frame_ = get_parameter("map_frame").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    chessboard_frame_ = get_parameter("chessboard_frame").as_string();
    camera_frame_ = get_parameter("camera_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();

    chessboard_pose_in_map_ = get_parameter("chessboard_pose_in_map").as_double_array();
    publish_static_tf_ = get_parameter("publish_tf").as_bool();

    odom_topic_ = get_parameter("odom_topic").as_string();
    odom_timeout_sec_ = get_parameter("odom_timeout_sec").as_double();
    base_pos_var_ = get_parameter("covariance.base_pos_var").as_double();
    base_rot_var_ = get_parameter("covariance.base_rot_var").as_double();
    vel_scale_pos_ = get_parameter("covariance.vel_scale_pos").as_double();
    vel_scale_rot_ = get_parameter("covariance.vel_scale_rot").as_double();
    max_pos_var_ = get_parameter("covariance.max_pos_var").as_double();
    max_rot_var_ = get_parameter("covariance.max_rot_var").as_double();
    scale_by_tag_count_ = get_parameter("covariance.scale_by_tag_count").as_bool();

    const auto tag_ids = get_parameter("tags.ids").as_integer_array();
    const auto tag_names = get_parameter("tags.names").as_string_array();
    const auto positions_x = get_parameter("tags.positions_x").as_double_array();
    const auto positions_y = get_parameter("tags.positions_y").as_double_array();
    const auto positions_z = get_parameter("tags.positions_z").as_double_array();
    const auto yaws = get_parameter("tags.yaws").as_double_array();

    const size_t n = tag_ids.size();
    if (tag_names.size() != n || positions_x.size() != n ||
        positions_y.size() != n || positions_z.size() != n || yaws.size() != n)
    {
      RCLCPP_FATAL(get_logger(), "Mismatch in tags configuration array sizes!");
      throw std::runtime_error("Tags configuration array sizes mismatch");
    }

    tag_configs_.clear();
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

  void ChessboardPoseEstimator::on_odometry(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
  {
    if (!msg)
    {
      return;
    }
    std::lock_guard<std::mutex> lock(odom_mutex_);
    latest_odom_ = msg;
    odom_received_.store(true);
  }

  ChessboardPoseEstimator::CovarianceResult ChessboardPoseEstimator::compute_covariance(
      double speed, double wz, int used_tags) const
  {
    CovarianceResult res;

    // Defensive validation for inputs
    if (std::isnan(speed) || std::isinf(speed) || speed < 0.0)
    {
      speed = 0.0;
    }
    if (std::isnan(wz) || std::isinf(wz))
    {
      wz = 0.0;
    }
    else
    {
      wz = std::abs(wz);
    }

    // Velocity-dependent variance scaling
    const double pos_var_dyn = base_pos_var_ + vel_scale_pos_ * (speed * speed);
    const double rot_var_dyn = base_rot_var_ + vel_scale_rot_ * (wz * wz);

    // Tag count scale factor
    double tag_scale = 1.0;
    if (scale_by_tag_count_)
    {
      if (used_tags >= 4)
      {
        tag_scale = 1.0;
      }
      else if (used_tags == 3)
      {
        tag_scale = 2.0;
      }
      else
      {
        // 2 tags or fewer: high uncertainty
        tag_scale = 4.0;
      }
    }

    // Clamp to [base_var, max_var]
    res.pos_var = std::clamp(pos_var_dyn * tag_scale, base_pos_var_, max_pos_var_);
    res.rot_var = std::clamp(rot_var_dyn * tag_scale, base_rot_var_, max_rot_var_);

    return res;
  }

  bool ChessboardPoseEstimator::should_process_image(
      const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
    const bool is_active = (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    const bool allow_detection = calib_ || (lifecycle_helper_ && lifecycle_helper_->is_mode_allowed());

    if (!is_active || !allow_detection || !has_camera_info_)
    {
      return false;
    }

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
    if (!tag_detections_pub_ || !tag_detections_pub_->is_activated())
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
    if (!tag_centers_pub_ || !tag_centers_pub_->is_activated() || img_size.width <= 0 || img_size.height <= 0)
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
    if (!robot_pose_pub_ || !robot_pose_pub_->is_activated())
    {
      return;
    }

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

      double current_speed = 0.0;
      double current_wz = 0.0;
      {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if (latest_odom_)
        {
          bool is_fresh = true;
          if (latest_odom_->header.stamp.sec > 0 || latest_odom_->header.stamp.nanosec > 0)
          {
            const rclcpp::Time img_time(header.stamp);
            const rclcpp::Time odom_time(latest_odom_->header.stamp);
            const double age = std::abs((img_time - odom_time).seconds());
            if (age > odom_timeout_sec_)
            {
              is_fresh = false;
              RCLCPP_WARN_THROTTLE(
                  get_logger(), *get_clock(), 5000,
                  "Latest odometry is stale (age: %.3f s > timeout: %.3f s). Fallback to static covariance.",
                  age, odom_timeout_sec_);
            }
          }
          if (is_fresh)
          {
            const auto &twist = latest_odom_->twist.twist;
            current_speed = std::hypot(twist.linear.x, twist.linear.y);
            current_wz = std::abs(twist.angular.z);
          }
        }
      }

      last_linear_speed_.store(current_speed);
      last_angular_speed_.store(current_wz);

      const auto cov = compute_covariance(current_speed, current_wz, used_tags);
      last_computed_pos_var_.store(cov.pos_var);
      last_computed_rot_var_.store(cov.rot_var);

      auto robot_pose_msg = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
      robot_pose_msg->header.stamp = header.stamp;
      robot_pose_msg->header.frame_id = map_frame_;
      tf2::toMsg(T_map_base, robot_pose_msg->pose.pose);

      robot_pose_msg->pose.covariance.fill(0.0);
      robot_pose_msg->pose.covariance[0] = cov.pos_var;
      robot_pose_msg->pose.covariance[7] = cov.pos_var;
      robot_pose_msg->pose.covariance[14] = cov.pos_var;
      robot_pose_msg->pose.covariance[21] = cov.rot_var;
      robot_pose_msg->pose.covariance[28] = cov.rot_var;
      robot_pose_msg->pose.covariance[35] = cov.rot_var;
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
    }
    else
    {
      publish_tag_centers(msg->header, cv_ptr->image.size(), marker_corners, marker_ids);
      estimate_and_publish_robot_pose(msg->header, marker_corners, marker_ids);
    }

    const auto proc_end = std::chrono::steady_clock::now();
    const double proc_ms = std::chrono::duration<double, std::milli>(proc_end - proc_start).count();

    if (lifecycle_helper_)
    {
      lifecycle_helper_->perf_tracker().record_frame(proc_ms);
    }
  }

  void ChessboardPoseEstimator::produce_diagnostics(
      diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    if (!lifecycle_helper_)
    {
      return;
    }

    const int used_tags = last_used_tags_.load();
    const bool is_busy = (used_tags >= min_tags_cnt_);

    std::string tag_ids_str;
    {
      std::lock_guard<std::mutex> lock(diag_mutex_);
      tag_ids_str = last_detected_tag_ids_str_;
    }

    std::string err_msg;
    if (!has_camera_info_)
    {
      err_msg = "Waiting for CameraInfo";
    }

    lifecycle_helper_->update_diagnostics(
        stat, is_busy, err_msg, static_cast<float>(detection_rate_hz_ * 0.5),
        [&](diagnostic_updater::DiagnosticStatusWrapper &s)
        {
          s.add("Calibration Mode", calib_ ? "true" : "false");
          s.addf("Target Detection Rate (Hz)", "%.1f", detection_rate_hz_);
          s.add("Used Board Tags Count", used_tags);
          s.add("Detected Tag IDs", tag_ids_str);
          s.add("Camera Info Received", has_camera_info_ ? "true" : "false");
          s.add("Odom Connected", odom_received_.load() ? "true" : "false");
          s.addf("Current Speed (m/s)", "%.3f", last_linear_speed_.load());
          s.addf("Current Yaw Rate (deg/s)", "%.2f", last_angular_speed_.load() * (180.0 / M_PI));
          s.addf("Calculated Pos Std (cm)", "%.2f", std::sqrt(last_computed_pos_var_.load()) * 100.0);
          s.addf("Calculated Yaw Std (deg)", "%.2f", std::sqrt(last_computed_rot_var_.load()) * (180.0 / M_PI));
        });
  }

} // namespace lekiwi_perception

RCLCPP_COMPONENTS_REGISTER_NODE(lekiwi_perception::ChessboardPoseEstimator)
