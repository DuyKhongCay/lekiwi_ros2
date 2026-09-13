/**
 * @file chess_visualizer_component.cpp
 * @brief Implementation of ChessVisualizerComponent overlay renderer.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include "chess_visualizer_component.hpp"
#include "hailo/chess_constants.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace fs = std::filesystem;

namespace lekiwi_perception
{

  ChessVisualizerComponent::ChessVisualizerComponent(const rclcpp::NodeOptions &options)
      : Node("chess_visualizer_component", options)
  {
    camera_topic_ = this->declare_parameter<std::string>("camera_topic", "/cameras/stereo_left/image_raw");
    fen_topic_ = this->declare_parameter<std::string>("fen_topic", "/chess/fen");
    detections_topic_ = this->declare_parameter<std::string>("detections_topic", "/chess/detections_2d");
    tag_centers_topic_ = this->declare_parameter<std::string>("tag_centers_topic", "/chess/tag_centers");
    grid_points_topic_ = this->declare_parameter<std::string>("grid_points_topic", "/chess/grid_points");
    jpeg_quality_ = this->declare_parameter<int>("jpeg_quality", 80);
    board_panel_size_ = this->declare_parameter<int>("board_panel_size", 480);

    const auto tag_ids = this->declare_parameter<std::vector<int64_t>>("tags.ids", {0, 1, 2, 3});
    tag_offsets_.clear();
    for (size_t i = 0; i < tag_ids.size() && i < 4; ++i)
    {
      tag_offsets_[static_cast<int>(tag_ids[i])] = static_cast<int>(i);
    }

    try
    {
      std::string share_dir = ament_index_cpp::get_package_share_directory("lekiwi_perception");
      pieces_dir_ = share_dir + "/resources/pieces";
    }
    catch (const std::exception &e)
    {
      pieces_dir_ = "resources/pieces";
    }

    RCLCPP_INFO(this->get_logger(), "Initializing Headless Chess Visualizer (JPEG Quality: %d)", jpeg_quality_);

    current_fen_ = hailo::kStandardStartingPlacement;
    last_valid_fen_ = current_fen_;

    fen_sub_ = this->create_subscription<std_msgs::msg::String>(
        fen_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ChessVisualizerComponent::fenCallback, this, std::placeholders::_1));

    detections_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
        detections_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ChessVisualizerComponent::detectionsCallback, this, std::placeholders::_1));

    tag_centers_sub_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
        tag_centers_topic_, rclcpp::QoS(1).transient_local().reliable(),
        std::bind(&ChessVisualizerComponent::tagCentersCallback, this, std::placeholders::_1));

    grid_points_sub_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
        grid_points_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ChessVisualizerComponent::gridPointsCallback, this, std::placeholders::_1));

    camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        camera_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ChessVisualizerComponent::cameraImageCallback, this, std::placeholders::_1));

    overlay_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        "/chess/overlay_image/compressed", rclcpp::SensorDataQoS());

    board_2d_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        "/chess/board_2d/compressed", rclcpp::SensorDataQoS());

    last_fps_time_ = this->now().seconds();
  }

  void ChessVisualizerComponent::fenCallback(const std_msgs::msg::String::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_fen_ = msg->data;
    if (!msg->data.empty() && msg->data.find("8/8/8/8/8/8/8/8") == std::string::npos)
    {
      last_valid_fen_ = msg->data;
    }
  }

  void ChessVisualizerComponent::detectionsCallback(const vision_msgs::msg::Detection2DArray::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_detections_ = msg->detections;
  }

  void ChessVisualizerComponent::tagCentersCallback(const geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_tag_centers_ = msg->polygon.points;
  }

  void ChessVisualizerComponent::gridPointsCallback(const geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_grid_points_ = msg->polygon.points;
  }

  void ChessVisualizerComponent::loadPieceSprites(int cell_size, const std::string &pieces_dir)
  {
    sprite_cache_.clear();
    cached_cell_size_ = cell_size;
    int icon_size = std::max(static_cast<int>(cell_size * 0.84), 8);

    for (const auto &[piece_char, filename] : hailo::kPiecePngNames)
    {
      std::string path = pieces_dir + "/" + filename;
      if (fs::exists(path))
      {
        cv::Mat img = cv::imread(path, cv::IMREAD_UNCHANGED);
        if (!img.empty() && img.channels() == 4)
        {
          cv::Mat resized;
          cv::resize(img, resized, cv::Size(icon_size, icon_size), 0, 0, cv::INTER_AREA);
          sprite_cache_[piece_char] = resized;
        }
      }
    }
  }

  std::map<std::string, std::string> ChessVisualizerComponent::parseFenToOccupancy(
      const std::string &fen_str)
  {
    std::map<std::string, std::string> occupancy_map;
    std::stringstream ss(fen_str);
    std::string board_fen;
    ss >> board_fen;

    int rank = 8;
    int file = 0;

    for (char ch : board_fen)
    {
      if (ch == '/')
      {
        rank--;
        file = 0;
      }
      else if (std::isdigit(ch))
      {
        file += (ch - '0');
      }
      else
      {
        if (file < 8 && rank >= 1)
        {
          std::string sq = std::string(1, static_cast<char>('a' + file)) + std::to_string(rank);
          occupancy_map[sq] = std::string(1, ch);
          file++;
        }
      }
    }
    return occupancy_map;
  }

  void ChessVisualizerComponent::drawPieceDetections(
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

  void ChessVisualizerComponent::drawTagCenters(
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

      // Draw tag center point (cyan circle)
      cv::circle(frame, cv::Point(u, v), 5, cv::Scalar(255, 255, 0), -1, cv::LINE_AA);
      cv::circle(frame, cv::Point(u, v), 8, cv::Scalar(0, 200, 255), 2, cv::LINE_AA);

      // Label with Tag Name and pixel coordinates
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

    // If 4 tags are available in standard order (0:A1, 1:H1, 2:H8, 3:A8), draw polygon outline
    if (corners_px.count(0) && corners_px.count(1) && corners_px.count(2) && corners_px.count(3))
    {
      std::vector<cv::Point> board_poly = {
          corners_px[0], corners_px[1], corners_px[2], corners_px[3]};
      cv::polylines(frame, board_poly, true, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    }
  }

  void ChessVisualizerComponent::drawChessboardGrid(
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

    // Draw grid lines: 9 rows (horizontal) and 9 columns (vertical)
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

    // Draw small intersection dots
    for (size_t i = 0; i < hailo::kGridPointsCount; ++i)
    {
      cv::circle(frame, pts_px[i], 2, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
    }

    // Draw 4 corners prominently:
    // 0: BL (A1, grid 72), 1: BR (H1, grid 80), 2: TR (H8, grid 8), 3: TL (A8, grid 0)
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

  void ChessVisualizerComponent::render2DBoardPanel(
      cv::Mat &panel, const std::map<std::string, std::string> &occupancy_map,
      const std::string &fen_str, int panel_width, int panel_height, float fps)
  {
    panel = cv::Mat(panel_height, panel_width, CV_8UC3, cv::Scalar(26, 30, 35));

    int header_h = std::min(static_cast<int>(panel_height * 0.08), 60);
    int footer_h = std::min(static_cast<int>(panel_height * 0.08), 60);
    int board_area_h = panel_height - header_h - footer_h;
    int board_area_w = panel_width - 40;
    int board_size = std::min(board_area_h, board_area_w);
    int cell_size = std::max(board_size / 8, 10);
    board_size = cell_size * 8;

    int start_x = (panel_width - board_size) / 2;
    int start_y = header_h + (board_area_h - board_size) / 2;

    cv::Scalar light_tile(240, 217, 181); // #F0D9B5 (BGR)
    cv::Scalar dark_tile(181, 136, 99);   // #B58863 (BGR)

    if (cached_cell_size_ != cell_size || sprite_cache_.empty())
    {
      loadPieceSprites(cell_size, pieces_dir_);
    }

    for (int r = 0; r < 8; ++r)
    {
      for (int c = 0; c < 8; ++c)
      {
        bool is_light = (r + c) % 2 == 0;
        cv::Scalar tile_color = is_light ? light_tile : dark_tile;
        int x1 = start_x + c * cell_size;
        int y1 = start_y + r * cell_size;
        cv::rectangle(panel, cv::Rect(x1, y1, cell_size, cell_size), tile_color, -1);

        std::string file_char(1, static_cast<char>('a' + c));
        std::string rank_char = std::to_string(8 - r);
        std::string sq_name = file_char + rank_char;

        auto it = occupancy_map.find(sq_name);
        if (it != occupancy_map.end() && !it->second.empty())
        {
          std::string piece = it->second;
          auto sprite_it = sprite_cache_.find(piece);
          if (sprite_it != sprite_cache_.end())
          {
            const cv::Mat &sprite = sprite_it->second;
            int sw = sprite.cols;
            int sh = sprite.rows;
            int ox = x1 + (cell_size - sw) / 2;
            int oy = y1 + (cell_size - sh) / 2;

            for (int py = 0; py < sh; ++py)
            {
              for (int px = 0; px < sw; ++px)
              {
                cv::Vec4b bgra = sprite.at<cv::Vec4b>(py, px);
                float alpha = bgra[3] / 255.0f;
                if (alpha > 0.01f)
                {
                  int ty = oy + py;
                  int tx = ox + px;
                  if (ty >= 0 && ty < panel_height && tx >= 0 && tx < panel_width)
                  {
                    cv::Vec3b &dst = panel.at<cv::Vec3b>(ty, tx);
                    dst[0] = static_cast<uchar>(alpha * bgra[0] + (1.0f - alpha) * dst[0]);
                    dst[1] = static_cast<uchar>(alpha * bgra[1] + (1.0f - alpha) * dst[1]);
                    dst[2] = static_cast<uchar>(alpha * bgra[2] + (1.0f - alpha) * dst[2]);
                  }
                }
              }
            }
          }
        }
      }
    }

    cv::rectangle(panel, cv::Rect(start_x, start_y, board_size, board_size), cv::Scalar(60, 70, 80), 2);

    double label_scale = std::max(cell_size / 90.0, 0.40);
    for (int i = 0; i < 8; ++i)
    {
      std::string file_char(1, static_cast<char>('a' + i));
      int lx = start_x + i * cell_size + cell_size / 2 - static_cast<int>(6 * label_scale);
      int ly = start_y + board_size + static_cast<int>(24 * label_scale);
      cv::putText(panel, file_char, cv::Point(lx, ly), cv::FONT_HERSHEY_SIMPLEX, label_scale, cv::Scalar(180, 180, 180), 1, cv::LINE_AA);

      std::string rank_char = std::to_string(8 - i);
      int rx = start_x - static_cast<int>(24 * label_scale);
      int ry = start_y + i * cell_size + cell_size / 2 + static_cast<int>(6 * label_scale);
      cv::putText(panel, rank_char, cv::Point(rx, ry), cv::FONT_HERSHEY_SIMPLEX, label_scale, cv::Scalar(180, 180, 180), 1, cv::LINE_AA);
    }

    std::string header_text = "2D BOARD (" + std::to_string(occupancy_map.size()) + " pieces)";
    cv::putText(panel, header_text, cv::Point(20, static_cast<int>(header_h * 0.65)),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 220, 0), 1, cv::LINE_AA);

    if (fps > 0.0f)
    {
      std::stringstream fps_ss;
      fps_ss << std::fixed << std::setprecision(1) << "FPS: " << fps;
      cv::putText(panel, fps_ss.str(), cv::Point(panel_width - 120, static_cast<int>(header_h * 0.65)),
                  cv::FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(120, 255, 0), 1, cv::LINE_AA);
    }

    std::string fen_display = fen_str.length() <= 52 ? fen_str : fen_str.substr(0, 49) + "...";
    cv::putText(panel, "FEN: " + fen_display, cv::Point(20, panel_height - static_cast<int>(footer_h * 0.35)),
                cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
  }

  void ChessVisualizerComponent::cameraImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    }
    catch (const cv_bridge::Exception &e)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "cv_bridge exception: %s", e.what());
      return;
    }

    if (cv_ptr->image.empty())
    {
      return;
    }

    // Update FPS calculation
    frame_count_++;
    double now_sec = this->now().seconds();
    double elapsed = now_sec - last_fps_time_;
    if (elapsed >= 1.0)
    {
      rolling_fps_ = static_cast<float>(frame_count_ / elapsed);
      frame_count_ = 0;
      last_fps_time_ = now_sec;
    }

    const std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};

    // 1. Overlay detections, grid points, and tag centers onto camera image & Publish /chess/overlay_image/compressed
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

    std::vector<uchar> overlay_buf;
    if (cv::imencode(".jpg", overlay_img, overlay_buf, encode_params))
    {
      auto overlay_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
      overlay_msg->header = msg->header;
      overlay_msg->format = "jpeg";
      overlay_msg->data = std::move(overlay_buf);
      overlay_pub_->publish(std::move(overlay_msg));
    }

    // 2. Render 2D top-down board panel & Publish /chess/board_2d/compressed
    std::string fen;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      fen = current_fen_;
      if (fen.empty() || fen.find(hailo::kEmptyBoardPlacement) != std::string::npos)
      {
        if (!last_valid_fen_.empty())
        {
          fen = last_valid_fen_;
        }
      }
    }
    auto occupancy_map = parseFenToOccupancy(fen);
    cv::Mat board_panel;
    render2DBoardPanel(board_panel, occupancy_map, fen, board_panel_size_, board_panel_size_, rolling_fps_);

    std::vector<uchar> board_buf;
    if (cv::imencode(".jpg", board_panel, board_buf, encode_params))
    {
      auto board_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
      board_msg->header = msg->header;
      board_msg->format = "jpeg";
      board_msg->data = std::move(board_buf);
      board_2d_pub_->publish(std::move(board_msg));
    }
  }

} // namespace lekiwi_perception

RCLCPP_COMPONENTS_REGISTER_NODE(lekiwi_perception::ChessVisualizerComponent)
