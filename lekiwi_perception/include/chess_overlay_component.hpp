/**
 * @file chess_overlay_component.hpp
 * @brief Perception overlay renderer: AprilTag corners, 81-point grid, and YOLO detections.
 *
 * Subscribes to raw camera image, /chess/detections_2d, /chess/tag_centers, and /chess/grid_points.
 * Publishes /chess/overlay_image/compressed.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "perception_utils.hpp"

namespace lekiwi_perception
{

  /**
   * @brief Headless component rendering piece bounding box overlays, tag corner coordinates, and chessboard grid.
   */
  class ChessOverlayComponent : public rclcpp::Node
  {
  public:
    explicit ChessOverlayComponent(const rclcpp::NodeOptions &options);
    ~ChessOverlayComponent() override = default;

  private:
    void cameraImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void detectionsCallback(const vision_msgs::msg::Detection2DArray::ConstSharedPtr msg);
    void tagCentersCallback(const geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg);
    void gridPointsCallback(const geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg);

    void drawPieceDetections(
        cv::Mat &frame, const std::vector<vision_msgs::msg::Detection2D> &detections);

    void drawTagCenters(
        cv::Mat &frame, const std::vector<geometry_msgs::msg::Point32> &tag_pts);

    void drawChessboardGrid(
        cv::Mat &frame, const std::vector<geometry_msgs::msg::Point32> &grid_pts);

    void drawTextBadge(
        cv::Mat &frame,
        const std::string &text,
        const cv::Point &pos,
        const cv::Scalar &text_color,
        const cv::Scalar &bg_color = cv::Scalar(0, 0, 0),
        double font_scale = 0.45,
        int baseline_pad = 2);

    // ROS 2 Communications
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr tag_centers_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr grid_points_sub_;

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr overlay_pub_;

    // Parameters & State
    std::string camera_topic_{"/cameras/stereo_left/image_raw"};
    std::string detections_topic_{"/chess/detections_2d"};
    std::string tag_centers_topic_{"/chess/tag_centers"};
    std::string grid_points_topic_{"/chess/grid_points"};
    std::string overlay_topic_{"/chess/overlay_image/compressed"};
    int jpeg_quality_{80};
    bool debug_{false};
    double stale_timeout_sec_{0.5};
    std::map<int, int> tag_offsets_;

    std::vector<vision_msgs::msg::Detection2D> latest_detections_;
    std::vector<geometry_msgs::msg::Point32> latest_tag_centers_;
    std::vector<geometry_msgs::msg::Point32> latest_grid_points_;
    rclcpp::Time last_detections_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_tag_centers_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_grid_points_time_{0, 0, RCL_ROS_TIME};
    std::mutex state_mutex_;

    // FPS & Performance tracking
    utils::FramePerformanceTracker perf_tracker_;
  };

} // namespace lekiwi_perception
