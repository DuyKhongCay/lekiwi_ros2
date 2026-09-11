/**
 * @file chessboard_pose_estimator.hpp
 * @brief High-precision AprilTag/ArUco visual pose estimation for LeKiwi mobile base.
 *
 * Adheres to Single Responsibility Principle (SRP):
 * Detects chessboard tags, solves Perspective-n-Point (PnP), and publishes 6-DoF robot pose
 * in the map frame and 2D tag centers for vision downstream pipelines.
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

#include "tf2/LinearMath/Transform.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "apriltag_localizer/pose_solver.hpp"

namespace apriltag_localizer
{

    /**
     * @brief Dedicated ROS 2 component for visual chessboard pose estimation.
     */
    class ChessboardPoseEstimator : public rclcpp::Node
    {
    public:
        explicit ChessboardPoseEstimator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~ChessboardPoseEstimator() override = default;

    private:
        void load_parameters();
        void init_detector();
        void publish_static_transforms();

        // Subscriptions
        void on_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg);
        void on_camera_mode(const lekiwi_interfaces::msg::CameraMode::ConstSharedPtr &msg);
        void on_image(const sensor_msgs::msg::Image::ConstSharedPtr &msg);

        // Sub-pipeline helper methods
        bool should_process_image(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
        cv_bridge::CvImageConstPtr convert_to_grayscale(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
        void detect_tags(
            const cv::Mat &gray_img,
            std::vector<std::vector<cv::Point2f>> &marker_corners,
            std::vector<int> &marker_ids);

        void publish_tag_detections_for_calib(
            const std_msgs::msg::Header &header,
            const std::vector<std::vector<cv::Point2f>> &marker_corners,
            const std::vector<int> &marker_ids);

        void publish_tag_centers(
            const std_msgs::msg::Header &header,
            const cv::Size &img_size,
            const std::vector<std::vector<cv::Point2f>> &marker_corners,
            const std::vector<int> &marker_ids);

        void estimate_and_publish_robot_pose(
            const std_msgs::msg::Header &header,
            const std::vector<std::vector<cv::Point2f>> &marker_corners,
            const std::vector<int> &marker_ids);

        void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

        // Parameters
        bool calib_{false};
        std::string tag_family_{"16h5"};
        double tag_size_{0.029};
        double detection_rate_hz_{5.0};
        int min_tags_cnt_{2};
        std::string map_frame_{"map"};
        std::string odom_frame_{"odom"};
        std::string chessboard_frame_{"chessboard_frame"};
        std::string camera_frame_{"stereo_left_optical"};
        std::string base_frame_{"base_footprint"};
        std::vector<double> chessboard_pose_in_map_{0.0, 0.0, 0.004, 0.0, 0.0, 0.0};
        bool publish_static_tf_{true};

        // Calibration & Detection structures
        std::map<int, TagConfig> tag_configs_;
        cv::Mat camera_matrix_;
        cv::Mat dist_coeffs_;
        bool has_camera_info_{false};

        cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
        cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;

        // ROS 2 Interfaces
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Subscription<lekiwi_interfaces::msg::CameraMode>::SharedPtr camera_mode_sub_;

        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr robot_pose_pub_;
        rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr tag_detections_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr tag_centers_pub_;

        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // Gating & Rate limiting
        uint8_t current_camera_mode_{lekiwi_interfaces::msg::CameraMode::STANDBY};
        rclcpp::Time last_detection_stamp_{0, 0, RCL_ROS_TIME};

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
