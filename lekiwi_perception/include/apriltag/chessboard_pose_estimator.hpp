/**
 * @file chessboard_pose_estimator.hpp
 * @brief High-precision AprilTag/ArUco visual pose estimation for LeKiwi mobile base.
 *
 * Adheres to Single Responsibility Principle (SRP) and Lifecycle architecture:
 * Detects chessboard tags, solves Perspective-n-Point (PnP), and publishes 6-DoF robot pose
 * in the map frame and 2D tag centers for vision downstream pipelines.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#ifndef LEKIWI_PERCEPTION__APRILTAG__CHESSBOARD_POSE_ESTIMATOR_HPP_
#define LEKIWI_PERCEPTION__APRILTAG__CHESSBOARD_POSE_ESTIMATOR_HPP_

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

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
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "tf2/LinearMath/Transform.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "apriltag/pose_solver.hpp"
#include "perception_utils.hpp"

namespace lekiwi_perception
{

    /**
     * @brief Dedicated ROS 2 Lifecycle Component for visual chessboard pose estimation.
     */
    class ChessboardPoseEstimator : public rclcpp_lifecycle::LifecycleNode
    {
    public:
        explicit ChessboardPoseEstimator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~ChessboardPoseEstimator() override = default;

        using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

        CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;
        CallbackReturn on_error(const rclcpp_lifecycle::State &state) override;

    private:
        void load_parameters();
        void init_detector();
        void publish_static_transforms();
        void reset_state();

        // Subscriptions
        void on_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg);
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
        bool autostart_{true};
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

        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr robot_pose_pub_;
        rclcpp_lifecycle::LifecyclePublisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr tag_detections_pub_;
        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PolygonStamped>::SharedPtr tag_centers_pub_;

        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // Rate limiting
        rclcpp::Time last_detection_stamp_{0, 0, RCL_ROS_TIME};

        // Composition helper for Lifecycle, Mode Gating, Autostart & Diagnostics
        std::unique_ptr<utils::PerceptionLifecycleHelper> lifecycle_helper_;
        std::atomic<int> last_used_tags_{0};
        std::mutex diag_mutex_;
        std::string last_detected_tag_ids_str_{"None"};
    };

} // namespace lekiwi_perception

#endif // LEKIWI_PERCEPTION__APRILTAG__CHESSBOARD_POSE_ESTIMATOR_HPP_
