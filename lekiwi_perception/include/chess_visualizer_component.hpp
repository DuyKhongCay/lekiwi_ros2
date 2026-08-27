// Copyright 2026 LeKiwi Labs. All rights reserved.
#pragma once

#include <rclcpp/callback_group.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <opencv2/opencv.hpp>
#include <condition_variable>
#include <memory>
#include <string>
#include <map>
#include <mutex>
#include <thread>
#include <atomic>

namespace lekiwi_perception {

class ChessVisualizerComponent : public rclcpp::Node {
public:
  explicit ChessVisualizerComponent(const rclcpp::NodeOptions & options);
  ~ChessVisualizerComponent() override;

private:
  /**
   * Receive debug camera image and render 2D side-by-side visualization.
   */
  void debugImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  /**
   * Receive latest FEN string update.
   */
  void fenCallback(const std_msgs::msg::String::ConstSharedPtr msg);

  /**
   * Load transparent piece PNG sprites from resources directory.
   */
  void loadPieceSprites(int cell_size, const std::string & pieces_dir);

  /**
   * Render digital 2D top-down chessboard into image panel.
   */
  void render2DBoardPanel(
    cv::Mat & panel, const std::map<std::string, std::string> & occupancy_map,
    const std::string & fen_str, int panel_width, int panel_height, float fps);

  /**
   * Parse FEN string into square-to-piece character map.
   */
  std::map<std::string, std::string> parseFenToOccupancy(const std::string & fen_str);

  // Dedicated thread for OpenCV Qt GUI event loop
  void guiThreadLoop();

  // ROS 2 Communications
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr debug_image_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fen_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr visual_image_pub_;

  // Visualization state
  std::string current_fen_;
  std::string last_valid_fen_;
  std::string pieces_dir_;
  bool gui_display_ = false;
  std::string window_name_;
  std::map<std::string, cv::Mat> sprite_cache_;
  int cached_cell_size_ = 0;
  std::mutex state_mutex_;
  std::condition_variable cv_var_;

  // Dedicated GUI Thread
  std::thread gui_thread_;
  std::atomic<bool> is_running_{false};

  // Frame buffer
  cv::Mat latest_image_;
  std_msgs::msg::Header latest_header_;
  bool has_new_frame_ = false;

  // FPS calculation
  double last_fps_time_ = 0.0;
  int frame_count_ = 0;
  float rolling_fps_ = 0.0f;
};

}  // namespace lekiwi_perception
