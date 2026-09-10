/**
 * @file chessboard_pose_estimator.hpp
 * @brief Direct AprilTag/Aruco chessboard pose estimation, TF broadcasting, and map anchor locking.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#ifndef APRILTAG_LOCALIZER__CHESSBOARD_POSE_ESTIMATOR_HPP_
#define APRILTAG_LOCALIZER__CHESSBOARD_POSE_ESTIMATOR_HPP_

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "cv_bridge/cv_bridge.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "lekiwi_interfaces/msg/camera_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "tf2/LinearMath/Transform.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "apriltag_localizer/pose_solver.hpp"

namespace apriltag_localizer
{

  /**
   * @brief ROS 2 Node that detects AprilTags in images, publishes robot poses, TFs, and diagnostics.
   */
  class ChessboardPoseEstimator : public rclcpp::Node
  {
  public:
    /**
     * @brief Constructs a ChessboardPoseEstimator node.
     * @param[in] options Node configuration options (e.g. parameter overrides).
     */
    explicit ChessboardPoseEstimator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    /**
     * @brief Virtual default destructor.
     */
    ~ChessboardPoseEstimator() override = default;

  private:
    /**
     * @brief Declares and loads ROS parameters for tag layouts and frame IDs.
     */
    void load_parameters();

    /**
     * @brief Initializes OpenCV Aruco detector dictionary and parameters.
     */
    void init_detector();

    /**
     * @brief Callback invoked when camera calibration info arrives.
     * @param[in] msg Const shared pointer to CameraInfo message containing intrinsics and distortion.
     */
    void on_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg);

    /**
     * @brief Callback invoked when camera mode changes.
     * @param[in] msg Const shared pointer to CameraMode message.
     */
    void on_camera_mode(const lekiwi_interfaces::msg::CameraMode::ConstSharedPtr &msg);

    /**
     * @brief Callback invoked when a camera image frame arrives.
     * @param[in] msg Const shared pointer to Image message.
     */
    void on_image(const sensor_msgs::msg::Image::ConstSharedPtr &msg);

    // --- Sub-pipeline helper methods for on_image ---
    /**
     * @brief Checks mode gating and rate-limiting to decide if this image should be processed.
     */
    bool should_process_image(const sensor_msgs::msg::Image::ConstSharedPtr &msg);

    /**
     * @brief Converts ROS Image message to OpenCV MONO8 format with error handling.
     */
    cv_bridge::CvImageConstPtr convert_to_grayscale(const sensor_msgs::msg::Image::ConstSharedPtr &msg);

    /**
     * @brief Detects ArUco tags in grayscale image and updates telemetry strings.
     */
    void detect_tags(
        const cv::Mat &gray_img,
        std::vector<std::vector<cv::Point2f>> &marker_corners,
        std::vector<int> &marker_ids);

    /**
     * @brief Publishes tag detections on /tag_detections topic for calibration mode.
     */
    void publish_tag_detections_for_calib(
        const std_msgs::msg::Header &header,
        const std::vector<std::vector<cv::Point2f>> &marker_corners,
        const std::vector<int> &marker_ids);

    /**
     * @brief Publishes lightweight normalized tag centers for Hailo chess inference component.
     */
    void publish_tag_centers(
        const std_msgs::msg::Header &header,
        const cv::Size &img_size,
        const std::vector<std::vector<cv::Point2f>> &marker_corners,
        const std::vector<int> &marker_ids);

    /**
     * @brief Computes Multi-Tag PnP pose and publishes robot pose in map frame.
     */
    void estimate_and_publish_robot_pose(
        const std_msgs::msg::Header &header,
        const std::vector<std::vector<cv::Point2f>> &marker_corners,
        const std::vector<int> &marker_ids);

    /**
     * @brief Service callback to lock the map->odom anchor transform from the latest tag detection.
     */
    void on_lock_anchor(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    /**
     * @brief Service callback to reset the map->odom anchor lock.
     */
    void on_reset_anchor(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    /**
     * @brief Periodic timer callback that broadcasts static TF and map->odom transform if anchored.
     */
    void publish_static_and_anchor_tf();

    /**
     * @brief Populates diagnostics status with framerate, latency, tag health, and anchor state.
     * @param[out] stat Diagnostics status wrapper to populate.
     */
    void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

    // Node Parameters
    bool calib_{false};
    std::string tag_family_{"16h5"};
    double tag_size_{0.02};
    double detection_rate_hz_{2.0};
    std::string map_frame_{"map"};
    std::string odom_frame_{"odom"};
    std::string chessboard_frame_{"chessboard_frame"};
    std::string camera_frame_{"stereo_left_optical"};
    std::string base_frame_{"base_link"};
    std::vector<double> chessboard_pose_in_map_{0.0, 0.0, 0.004, 0.0, 0.0, 0.0};
    bool publish_tf_{true};
    bool publish_map_to_chessboard_{true};
    bool publish_map_to_odom_{true};
    double min_depth_m_{0.05};

    // Calibration & Config
    std::map<int, TagConfig> tag_configs_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    bool has_camera_info_{false};

    // OpenCV Aruco Detector
    cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
    cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;

    // ROS 2 Interfaces
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<lekiwi_interfaces::msg::CameraMode>::SharedPtr camera_mode_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr robot_pose_pub_;
    rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr tag_detections_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr tag_centers_pub_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr lock_anchor_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_anchor_srv_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::TimerBase::SharedPtr tf_timer_;

    // State gating & execution control
    uint8_t current_camera_mode_{lekiwi_interfaces::msg::CameraMode::STANDBY};
    rclcpp::Time last_detection_stamp_{0, 0, RCL_ROS_TIME};

    // Anchor lock state
    bool is_anchored_{false};
    geometry_msgs::msg::TransformStamped T_map_odom_locked_;

    // Latest valid pose (only valid when >= 2 tags detected)
    bool has_valid_pose_{false};
    tf2::Transform latest_T_map_base_;

    // Diagnostics & Telemetry
    diagnostic_updater::Updater diagnostic_updater_{this};
    std::atomic<double> last_proc_time_ms_{0.0};
    std::atomic<float> current_fps_{0.0F};
    std::atomic<uint64_t> frame_counter_{0};
    std::chrono::steady_clock::time_point last_fps_time_;
    uint64_t last_fps_frame_count_{0};
    std::atomic<int> last_used_tags_{0};
    std::mutex diag_mutex_;
    std::string last_detected_tag_ids_str_{"None"};
  };

} // namespace apriltag_localizer

#endif // APRILTAG_LOCALIZER__CHESSBOARD_POSE_ESTIMATOR_HPP_
