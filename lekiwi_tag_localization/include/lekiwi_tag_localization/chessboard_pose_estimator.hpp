/**
 * @file chessboard_pose_estimator.hpp
 * @brief AprilTag-based chessboard pose estimation, TF broadcasting, and orientation mapping.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#ifndef LEKIWI_TAG_LOCALIZATION__CHESSBOARD_POSE_ESTIMATOR_HPP_
#define LEKIWI_TAG_LOCALIZATION__CHESSBOARD_POSE_ESTIMATOR_HPP_

#include <memory>
#include <map>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include "rclcpp/rclcpp.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Transform.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace lekiwi_tag_localization
{

  /**
   * @brief Configuration for a single AprilTag mounted on the chessboard.
   */
  struct TagConfig
  {
    int id{0};
    std::string name;
    cv::Point3d center{0.0, 0.0, 0.0};
    double yaw{0.0};
    std::vector<cv::Point3d> corners_board;
  };

  /**
   * @brief Pure algorithmic helper functions for pose estimation and geometric calculations.
   */
  class PoseSolver
  {
  public:
    /**
     * @brief Computes 4 3D corner coordinates of a square tag in chessboard coordinates.
     * @param[in] center 3D center in chessboard_frame.
     * @param[in] size Edge length in meters (e.g. 0.02).
     * @param[in] yaw_rad In-plane rotation around Z axis in radians. Default is 0.0.
     * @return 4 3D points ordered CCW: bottom-left, bottom-right, top-right, top-left.
     */
    static std::vector<cv::Point3d> compute_tag_corners(
        const cv::Point3d &center, double size, double yaw_rad = 0.0);

    /**
     * @brief Estimates chessboard pose T_cam^board using Single-Tag or Multi-Tag PnP.
     * @param[in] detections Array of detected AprilTags with 2D corner positions.
     * @param[in] tag_configs Map of configured tags with their fixed 3D layout on the board.
     * @param[in] camera_matrix 3x3 intrinsic camera matrix.
     * @param[in] dist_coeffs Camera distortion coefficients vector.
     * @param[out] rvec 3x1 Rodrigues rotation vector of the board in camera optical frame.
     * @param[out] tvec 3x1 translation vector of the board in camera optical frame.
     * @param[out] used_tags_count Number of valid board tags used in PnP estimation.
     * @return True if pose estimation succeeds and satisfies validity constraints, false otherwise.
     */
    static bool estimate_board_pose(
        const std::vector<apriltag_msgs::msg::AprilTagDetection> &detections,
        const std::map<int, TagConfig> &tag_configs,
        const cv::Mat &camera_matrix,
        const cv::Mat &dist_coeffs,
        cv::Mat &rvec,
        cv::Mat &tvec,
        int &used_tags_count);

    /**
     * @brief Computes T_cam^board from a single tag's T_cam^tag using T_cam^board = T_cam^tag * (T_board^tag)^-1.
     * @param[in] rvec_tag Rodrigues rotation vector of the single tag in camera frame.
     * @param[in] tvec_tag Translation vector of the single tag in camera frame.
     * @param[in] tag_center_board 3D center position of the tag in chessboard_frame.
     * @param[in] tag_yaw_board Yaw rotation angle (radians) of the tag in chessboard_frame.
     * @param[out] rvec_board Resulting Rodrigues rotation vector of chessboard in camera frame.
     * @param[out] tvec_board Resulting translation vector of chessboard in camera frame.
     */
    static void compute_board_from_single_tag(
        const cv::Mat &rvec_tag,
        const cv::Mat &tvec_tag,
        const cv::Point3d &tag_center_board,
        double tag_yaw_board,
        cv::Mat &rvec_board,
        cv::Mat &tvec_board);


    /**
     * @brief Generates a Keepout Polygon around the chessboard with safety margin.
     * @param[in] frame_id TF frame ID for the header (typically chessboard_frame).
     * @param[in] stamp Timestamp for the polygon header.
     * @param[in] board_size Physical side length of the chessboard in meters. Default is 0.38.
     * @param[in] margin Safety buffer distance around the board perimeter in meters. Default is 0.035.
     * @return PolygonStamped message representing the bounding keepout area for Nav2 costmap.
     */
    static geometry_msgs::msg::PolygonStamped create_keepout_polygon(
        const std::string &frame_id,
        const rclcpp::Time &stamp,
        double board_size = 0.38,
        double margin = 0.035);
  };

  /**
   * @brief ROS 2 Node that consumes AprilTag detections, publishes poses, TFs, orientation, and costmap polygon.
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
     * @brief Declares and loads ROS parameters for tag layouts, frame IDs, and keepout margins.
     */
    void load_parameters();

    /**
     * @brief Callback invoked when camera calibration info arrives.
     * @param[in] msg Const shared pointer to CameraInfo message containing intrinsics and distortion.
     */
    void on_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg);

    /**
     * @brief Callback invoked when AprilTag detections are published.
     * @param[in] msg Const shared pointer to AprilTagDetectionArray message.
     */
    void on_tag_detections(const apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr &msg);

    /**
     * @brief Timer callback that publishes the keepout costmap polygon and static TF transform.
     */
    void publish_keepout_and_static_tf();

    // Node Parameters
    std::string tag_family_{"36h11"};
    double tag_size_{0.02};
    double tag_distance_{0.38};
    std::string map_frame_{"map"};
    std::string chessboard_frame_{"chessboard_frame"};
    std::string camera_frame_{"stereo_left_optical"};
    std::string base_frame_{"base_link"};
    std::vector<double> chessboard_pose_in_map_{1.0, 0.0, 0.75, 0.0, 0.0, 0.0};
    bool publish_tf_{true};
    bool publish_map_to_chessboard_{true};
    bool publish_camera_to_board_{true};
    double keepout_margin_{0.035};
    double keepout_rate_{2.0};

    // Calibration & Config
    std::map<int, TagConfig> tag_configs_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    bool has_camera_info_{false};

    // ROS 2 Interfaces
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr tag_detections_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr board_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr robot_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr costmap_polygon_pub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::TimerBase::SharedPtr keepout_timer_;

    // Last known poses for smoothing / initial guess
    cv::Mat last_rvec_;
    cv::Mat last_tvec_;
    bool has_last_pose_{false};
  };

} // namespace lekiwi_tag_localization

#endif // LEKIWI_TAG_LOCALIZATION__CHESSBOARD_POSE_ESTIMATOR_HPP_
