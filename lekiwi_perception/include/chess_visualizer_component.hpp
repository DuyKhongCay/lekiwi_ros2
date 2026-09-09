/**
 * @file chess_visualizer_component.hpp
 * @brief Headless chess detection overlay and 2D board compressed image publisher for RViz / Host PC.
 *
 * Subscribes to raw camera image, `/chess/detections_2d`, and `/chess/fen`.
 * Publishes `/chess/overlay_image/compressed` and `/chess/board_2d/compressed`.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/string.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <opencv2/opencv.hpp>
#include <memory>
#include <string>
#include <map>
#include <vector>
#include <mutex>
#include <atomic>

namespace lekiwi_perception
{

  /**
   * @brief Headless component rendering piece bounding box overlays and 2D board panels to compressed image topics.
   */
  class ChessVisualizerComponent : public rclcpp::Node
  {
  public:
    explicit ChessVisualizerComponent(const rclcpp::NodeOptions &options);
    ~ChessVisualizerComponent() override = default;

  private:
    /**
     * Callback for raw camera image frame: renders overlay and 2D board, encodes to JPEG and publishes.
     */
    void cameraImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

    /**
     * Receive latest FEN string update.
     */
    void fenCallback(const std_msgs::msg::String::ConstSharedPtr msg);

    /**
     * Receive piece detections array.
     */
    void detectionsCallback(const vision_msgs::msg::Detection2DArray::ConstSharedPtr msg);

    /**
     * Load transparent piece PNG sprites from resources directory.
     */
    void loadPieceSprites(int cell_size, const std::string &pieces_dir);

    /**
     * Render digital 2D top-down chessboard into image panel.
     */
    void render2DBoardPanel(
        cv::Mat &panel, const std::map<std::string, std::string> &occupancy_map,
        const std::string &fen_str, int panel_width, int panel_height, float fps);

    /**
     * Draw detection bounding boxes onto the camera frame.
     */
    void drawPieceDetections(
        cv::Mat &frame, const std::vector<vision_msgs::msg::Detection2D> &detections);

    /**
     * Parse FEN string into square-to-piece character map.
     */
    std::map<std::string, std::string> parseFenToOccupancy(const std::string &fen_str);

    // ROS 2 Communications
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fen_sub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_;

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr overlay_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr board_2d_pub_;

    // Parameters & State
    std::string camera_topic_{"/cameras/stereo_left/image_raw"};
    std::string fen_topic_{"/chess/fen"};
    std::string detections_topic_{"/chess/detections_2d"};
    int jpeg_quality_{80};
    int board_panel_size_{480};

    std::string current_fen_;
    std::string last_valid_fen_;
    std::vector<vision_msgs::msg::Detection2D> latest_detections_;
    std::string pieces_dir_;
    std::map<std::string, cv::Mat> sprite_cache_;
    int cached_cell_size_{0};
    std::mutex state_mutex_;

    // FPS calculation
    double last_fps_time_{0.0};
    int frame_count_{0};
    float rolling_fps_{0.0F};
  };

} // namespace lekiwi_perception
