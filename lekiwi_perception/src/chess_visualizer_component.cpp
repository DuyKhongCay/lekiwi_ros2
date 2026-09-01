/**
 * @file chess_visualizer_component.cpp
 * @brief Implementation of ChessVisualizerComponent overlay renderer.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include "chess_visualizer_component.hpp"
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

  static const std::map<std::string, std::string> PIECE_PNG_NAMES = {
      {"B", "w-bishop.png"}, {"K", "w-king.png"}, {"N", "w-knight.png"}, {"P", "w-pawn.png"}, {"Q", "w-queen.png"}, {"R", "w-rook.png"}, {"b", "b-bishop.png"}, {"k", "b-king.png"}, {"n", "b-knight.png"}, {"p", "b-pawn.png"}, {"q", "b-queen.png"}, {"r", "b-rook.png"}};

  ChessVisualizerComponent::ChessVisualizerComponent(const rclcpp::NodeOptions &options)
      : Node("chess_visualizer_component", options)
  {
    gui_display_ = this->declare_parameter<bool>("gui_display", false);
    window_name_ = "Hailo Chess Vision - Live Perception & 2D Board";

    try
    {
      std::string share_dir = ament_index_cpp::get_package_share_directory("lekiwi_perception");
      pieces_dir_ = share_dir + "/resources/pieces";
    }
    catch (const std::exception &e)
    {
      pieces_dir_ = "resources/pieces";
    }

    RCLCPP_INFO(this->get_logger(), "Initializing Chess Visualizer Component (GUI Display: %s)",
                gui_display_ ? "ON" : "OFF");

    current_fen_ = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1";
    last_valid_fen_ = current_fen_;

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = callback_group_;

    fen_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/chess/fen", rclcpp::SensorDataQoS(),
        std::bind(&ChessVisualizerComponent::fenCallback, this, std::placeholders::_1), sub_opts);

    debug_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/chess/debug_image", rclcpp::SensorDataQoS(),
        std::bind(&ChessVisualizerComponent::debugImageCallback, this, std::placeholders::_1), sub_opts);

    visual_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/chess/visualization_image", rclcpp::SensorDataQoS());

    last_fps_time_ = this->now().seconds();

    if (gui_display_)
    {
      is_running_ = true;
      gui_thread_ = std::thread(&ChessVisualizerComponent::guiThreadLoop, this);
    }
  }

  ChessVisualizerComponent::~ChessVisualizerComponent()
  {
    is_running_ = false;
    cv_var_.notify_all();
    if (gui_thread_.joinable())
    {
      gui_thread_.join();
    }
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

  void ChessVisualizerComponent::loadPieceSprites(int cell_size, const std::string &pieces_dir)
  {
    sprite_cache_.clear();
    cached_cell_size_ = cell_size;
    int icon_size = std::max(static_cast<int>(cell_size * 0.84), 8);

    for (const auto &[piece_char, filename] : PIECE_PNG_NAMES)
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

    std::string header_text = "CONVERTED 2D BOARD (" + std::to_string(occupancy_map.size()) + " pieces)";
    cv::putText(panel, header_text, cv::Point(20, static_cast<int>(header_h * 0.65)),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 220, 0), 1, cv::LINE_AA);

    if (fps > 0.0f)
    {
      std::stringstream fps_ss;
      fps_ss << std::fixed << std::setprecision(1) << "FPS: " << fps;
      cv::putText(panel, fps_ss.str(), cv::Point(panel_width - 140, static_cast<int>(header_h * 0.65)),
                  cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(120, 255, 0), 1, cv::LINE_AA);
    }

    cv::line(panel, cv::Point(0, 0), cv::Point(0, panel_height), cv::Scalar(255, 200, 0), 2);

    std::string fen_display = fen_str.length() <= 52 ? fen_str : fen_str.substr(0, 49) + "...";
    cv::putText(panel, "FEN: " + fen_display, cv::Point(20, panel_height - static_cast<int>(footer_h * 0.35)),
                cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
  }

  void ChessVisualizerComponent::debugImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      // Deep copy ensures memory safety even with intra-process zero-copy comms
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
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

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_image_ = cv_ptr->image;
      latest_header_ = msg->header;
      has_new_frame_ = true;
    }
    cv_var_.notify_one();

    // Publish visualization topic if subscribed
    size_t vis_subs = visual_image_pub_->get_subscription_count() + visual_image_pub_->get_intra_process_subscription_count();
    if (vis_subs > 0)
    {
      std::string fen;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        fen = current_fen_;
        if (fen.empty() || fen.find("8/8/8/8/8/8/8/8") != std::string::npos)
        {
          if (!last_valid_fen_.empty())
          {
            fen = last_valid_fen_;
          }
        }
      }
      auto occupancy_map = parseFenToOccupancy(fen);
      int cam_h = cv_ptr->image.rows;
      int board_w = cam_h;
      cv::Mat board_panel;
      render2DBoardPanel(board_panel, occupancy_map, fen, board_w, cam_h, rolling_fps_);

      cv::Mat composite;
      cv::hconcat(cv_ptr->image, board_panel, composite);

      auto out_msg = std::make_unique<sensor_msgs::msg::Image>();
      cv_bridge::CvImage cv_out(msg->header, "bgr8", composite);
      cv_out.toImageMsg(*out_msg);
      visual_image_pub_->publish(std::move(out_msg));
    }
  }

  void ChessVisualizerComponent::guiThreadLoop()
  {
    // Initialize window exclusively on this GUI thread for Qt event loop thread affinity
    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_name_, 1360, 580);

    while (rclcpp::ok() && is_running_)
    {
      cv::Mat cam_img;
      std::string fen;
      bool has_frame = false;

      {
        std::unique_lock<std::mutex> lock(state_mutex_);
        cv_var_.wait_for(lock, std::chrono::milliseconds(30), [this]
                         { return !is_running_ || has_new_frame_; });

        if (!is_running_ || !rclcpp::ok())
          break;

        if (has_new_frame_ && !latest_image_.empty())
        {
          cam_img = latest_image_.clone();
          has_new_frame_ = false;
          has_frame = true;
        }
        fen = current_fen_;
        if (fen.empty() || fen.find("8/8/8/8/8/8/8/8") != std::string::npos)
        {
          if (!last_valid_fen_.empty())
          {
            fen = last_valid_fen_;
          }
        }
      }

      if (has_frame && !cam_img.empty())
      {
        frame_count_++;
        double now_sec = this->now().seconds();
        double elapsed = now_sec - last_fps_time_;
        if (elapsed >= 1.0)
        {
          rolling_fps_ = static_cast<float>(frame_count_ / elapsed);
          frame_count_ = 0;
          last_fps_time_ = now_sec;
        }

        auto occupancy_map = parseFenToOccupancy(fen);
        int cam_h = cam_img.rows;
        int board_w = cam_h;
        cv::Mat board_panel;
        render2DBoardPanel(board_panel, occupancy_map, fen, board_w, cam_h, rolling_fps_);

        cv::Mat composite;
        cv::hconcat(cam_img, board_panel, composite);

        cv::imshow(window_name_, composite);
      }

      cv::waitKey(1);
    }

    try
    {
      cv::destroyWindow(window_name_);
      cv::waitKey(1);
    }
    catch (...)
    {
    }
  }

} // namespace lekiwi_perception

RCLCPP_COMPONENTS_REGISTER_NODE(lekiwi_perception::ChessVisualizerComponent)
