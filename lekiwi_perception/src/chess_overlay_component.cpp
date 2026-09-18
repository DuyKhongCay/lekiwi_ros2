/**
 * @file chess_overlay_component.cpp
 * @brief Implementation of ChessOverlayComponent perception overlay renderer.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include "chess_overlay_component.hpp"
#include "hailo/chess_constants.hpp"
#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <iomanip>
#include <sstream>

namespace lekiwi_perception
{

  ChessOverlayComponent::ChessOverlayComponent(const rclcpp::NodeOptions &options)
      : Node("chess_overlay_component", options)
  {
    camera_topic_ = declare_parameter<std::string>("camera_topic", "/cameras/stereo_left/image_raw");
    detections_topic_ = declare_parameter<std::string>("detections_topic", "/chess/detections_2d");
    tag_centers_topic_ = declare_parameter<std::string>("tag_centers_topic", "/chess/tag_centers");
    grid_points_topic_ = declare_parameter<std::string>("grid_points_topic", "/chess/grid_points");
    overlay_topic_ = declare_parameter<std::string>("overlay_topic", "/chess/overlay_image/compressed");
    jpeg_quality_ = declare_parameter<int>("jpeg_quality", 80);
    debug_ = declare_parameter<bool>("debug", false);

    const auto tag_ids = declare_parameter<std::vector<int64_t>>("tags.ids", {0, 1, 2, 3});
    tag_offsets_.clear();
    for (size_t i = 0; i < tag_ids.size() && i < 4; ++i)
    {
      tag_offsets_[static_cast<int>(tag_ids[i])] = static_cast<int>(i);
    }

    RCLCPP_INFO(get_logger(),
                "Starting ChessOverlayComponent (Camera: %s, Overlay: %s, JPEG Quality: %d)",
                camera_topic_.c_str(), overlay_topic_.c_str(), jpeg_quality_);

    camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
        camera_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ChessOverlayComponent::cameraImageCallback, this, std::placeholders::_1));

    detections_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
        detections_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ChessOverlayComponent::detectionsCallback, this, std::placeholders::_1));

    tag_centers_sub_ = create_subscription<geometry_msgs::msg::PolygonStamped>(
        tag_centers_topic_, rclcpp::QoS(1).transient_local().reliable(),
        std::bind(&ChessOverlayComponent::tagCentersCallback, this, std::placeholders::_1));

    grid_points_sub_ = create_subscription<geometry_msgs::msg::PolygonStamped>(
        grid_points_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ChessOverlayComponent::gridPointsCallback, this, std::placeholders::_1));

    overlay_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
        overlay_topic_, rclcpp::SensorDataQoS());
  }

  void ChessOverlayComponent::detectionsCallback(const vision_msgs::msg::Detection2DArray::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_detections_ = msg->detections;
  }

  void ChessOverlayComponent::tagCentersCallback(const geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_tag_centers_ = msg->polygon.points;
  }

  void ChessOverlayComponent::gridPointsCallback(const geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_grid_points_ = msg->polygon.points;
  }

  void ChessOverlayComponent::drawPieceDetections(
      cv::Mat &frame, const std::vector<vision_msgs::msg::Detection2D> &detections)
  {
    for (const auto &det : detections)
    {
      if (det.results.empty())
      {
        continue;
      }

      int bw = static_cast<int>(det.bbox.size_x);
      int bh = static_cast<int>(det.bbox.size_y);
      int x1 = static_cast<int>(det.bbox.center.position.x - det.bbox.size_x / 2.0);
      int y1 = static_cast<int>(det.bbox.center.position.y - det.bbox.size_y / 2.0);

      x1 = std::clamp(x1, 0, frame.cols - 1);
      y1 = std::clamp(y1, 0, frame.rows - 1);
      bw = std::clamp(bw, 1, frame.cols - x1);
      bh = std::clamp(bh, 1, frame.rows - y1);

      const std::string &label = det.results[0].hypothesis.class_id;
      float conf = det.results[0].hypothesis.score;

      const cv::Scalar color = hailo::get_piece_color_bgr(label);

      cv::rectangle(frame, cv::Rect(x1, y1, bw, bh), color, 2);

      std::stringstream label_ss;
      label_ss << label << " " << std::fixed << std::setprecision(2) << conf;
      std::string label_str = label_ss.str();

      int baseline = 0;
      cv::Size text_size = cv::getTextSize(label_str, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseline);
      int label_y1 = std::max(y1 - text_size.height - 4, 0);
      cv::rectangle(
          frame, cv::Rect(x1, label_y1, text_size.width + 4, text_size.height + 4),
          color, -1);
      cv::putText(
          frame, label_str, cv::Point(x1 + 2, label_y1 + text_size.height + 1),
          cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
    }
  }

  void ChessOverlayComponent::drawTagCenters(
      cv::Mat &frame, const std::vector<geometry_msgs::msg::Point32> &tag_pts)
  {
    if (tag_pts.empty() || frame.cols <= 0 || frame.rows <= 0)
    {
      return;
    }

    const double img_w = static_cast<double>(frame.cols);
    const double img_h = static_cast<double>(frame.rows);

    std::map<int, cv::Point> corners_px;
    for (const auto &pt : tag_pts)
    {
      int tag_id = static_cast<int>(pt.z);
      int u = std::clamp(static_cast<int>(pt.x * img_w), 0, frame.cols - 1);
      int v = std::clamp(static_cast<int>(pt.y * img_h), 0, frame.rows - 1);
      corners_px[tag_id] = cv::Point(u, v);

      // Tag center point
      cv::circle(frame, cv::Point(u, v), 5, cv::Scalar(255, 255, 0), -1, cv::LINE_AA);
      cv::circle(frame, cv::Point(u, v), 8, cv::Scalar(0, 200, 255), 2, cv::LINE_AA);

      // Label
      std::string tag_name;
      auto offset_it = tag_offsets_.find(tag_id);
      if (offset_it != tag_offsets_.end() && offset_it->second >= 0 && offset_it->second < 4)
      {
        tag_name = hailo::kCornerNames[offset_it->second];
      }
      else
      {
        tag_name = "ID" + std::to_string(tag_id);
      }
      std::stringstream tag_text_ss;
      tag_text_ss << tag_name << "(" << u << "," << v << ")";
      std::string tag_text = tag_text_ss.str();

      int baseline = 0;
      cv::Size text_size = cv::getTextSize(tag_text, cv::FONT_HERSHEY_SIMPLEX, 0.45, 1, &baseline);
      int text_x = std::clamp(u + 10, 0, frame.cols - text_size.width - 4);
      int text_y = std::clamp(v - 8, text_size.height + 4, frame.rows - 4);

      cv::rectangle(
          frame, cv::Rect(text_x - 2, text_y - text_size.height - 2, text_size.width + 4, text_size.height + 4),
          cv::Scalar(0, 0, 0), -1);
      cv::putText(
          frame, tag_text, cv::Point(text_x, text_y),
          cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
    }

    if (corners_px.count(0) && corners_px.count(1) && corners_px.count(2) && corners_px.count(3))
    {
      std::vector<cv::Point> board_poly = {
          corners_px[0], corners_px[1], corners_px[2], corners_px[3]};
      cv::polylines(frame, board_poly, true, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    }
  }

  void ChessOverlayComponent::drawChessboardGrid(
      cv::Mat &frame, const std::vector<geometry_msgs::msg::Point32> &grid_pts)
  {
    if (grid_pts.size() != hailo::kGridPointsCount || frame.cols <= 0 || frame.rows <= 0)
    {
      return;
    }

    const float img_w = static_cast<float>(frame.cols);
    const float img_h = static_cast<float>(frame.rows);

    std::vector<cv::Point> pts_px(hailo::kGridPointsCount);
    for (size_t i = 0; i < hailo::kGridPointsCount; ++i)
    {
      int u = std::clamp(static_cast<int>(grid_pts[i].x * img_w), 0, frame.cols - 1);
      int v = std::clamp(static_cast<int>(grid_pts[i].y * img_h), 0, frame.rows - 1);
      pts_px[i] = cv::Point(u, v);
    }

    const cv::Scalar grid_line_color(0, 220, 100);
    for (int r = 0; r < 9; ++r)
    {
      for (int c = 0; c < 8; ++c)
      {
        cv::line(frame, pts_px[r * 9 + c], pts_px[r * 9 + (c + 1)], grid_line_color, 1, cv::LINE_AA);
      }
    }
    for (int c = 0; c < 9; ++c)
    {
      for (int r = 0; r < 8; ++r)
      {
        cv::line(frame, pts_px[r * 9 + c], pts_px[(r + 1) * 9 + c], grid_line_color, 1, cv::LINE_AA);
      }
    }

    for (size_t i = 0; i < hailo::kGridPointsCount; ++i)
    {
      cv::circle(frame, pts_px[i], 2, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
    }

    for (size_t i = 0; i < 4; ++i)
    {
      int grid_idx = hailo::kCornerGridIndices[i];
      const char *name = hailo::kCornerNames[i];
      const cv::Point &pt = pts_px[grid_idx];
      cv::circle(frame, pt, 5, cv::Scalar(0, 255, 255), -1, cv::LINE_AA);
      cv::circle(frame, pt, 8, cv::Scalar(0, 140, 255), 2, cv::LINE_AA);

      int baseline = 0;
      cv::Size text_size = cv::getTextSize(name, cv::FONT_HERSHEY_SIMPLEX, 0.40, 1, &baseline);
      int tx = std::clamp(pt.x + 8, 0, frame.cols - text_size.width - 4);
      int ty = std::clamp(pt.y - 6, text_size.height + 4, frame.rows - 4);

      cv::rectangle(frame, cv::Rect(tx - 2, ty - text_size.height - 2, text_size.width + 4, text_size.height + 4),
                    cv::Scalar(0, 0, 0), -1);
      cv::putText(frame, name, cv::Point(tx, ty), cv::FONT_HERSHEY_SIMPLEX, 0.40,
                  cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
    }
  }

  void ChessOverlayComponent::cameraImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    }
    catch (const cv_bridge::Exception &e)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "cv_bridge exception: %s", e.what());
      return;
    }

    if (cv_ptr->image.empty())
    {
      return;
    }

    perf_tracker_.record_frame();

    cv::Mat overlay_img = cv_ptr->image.clone();
    std::vector<vision_msgs::msg::Detection2D> dets;
    std::vector<geometry_msgs::msg::Point32> tags;
    std::vector<geometry_msgs::msg::Point32> grid_pts;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      dets = latest_detections_;
      tags = latest_tag_centers_;
      grid_pts = latest_grid_points_;
    }

    drawChessboardGrid(overlay_img, grid_pts);
    drawTagCenters(overlay_img, tags);
    drawPieceDetections(overlay_img, dets);

    if (debug_)
    {
      float fps = perf_tracker_.get_fps();
      std::stringstream fps_ss;
      fps_ss << std::fixed << std::setprecision(1) << "FPS: " << fps;
      cv::putText(overlay_img, fps_ss.str(), cv::Point(20, 30),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    }

    const std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
    std::vector<uchar> overlay_buf;
    if (cv::imencode(".jpg", overlay_img, overlay_buf, encode_params))
    {
      auto overlay_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
      overlay_msg->header = msg->header;
      overlay_msg->format = "jpeg";
      overlay_msg->data = std::move(overlay_buf);
      overlay_pub_->publish(std::move(overlay_msg));
    }
  }

} // namespace lekiwi_perception

RCLCPP_COMPONENTS_REGISTER_NODE(lekiwi_perception::ChessOverlayComponent)
