/**
 * @file chessboard_visualizer_component.cpp
 * @brief Implementation of ChessboardVisualizerComponent 2D panel renderer.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include "lekiwi_chess_master/chessboard_visualizer_component.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace fs = std::filesystem;

namespace lekiwi_chess_master
{

  namespace
  {
    const std::map<std::string, std::string> kPiecePngNames = {
        {"P", "w-pawn.png"}, {"N", "w-knight.png"}, {"B", "w-bishop.png"}, {"R", "w-rook.png"}, {"Q", "w-queen.png"}, {"K", "w-king.png"}, {"p", "b-pawn.png"}, {"n", "b-knight.png"}, {"b", "b-bishop.png"}, {"r", "b-rook.png"}, {"q", "b-queen.png"}, {"k", "b-king.png"}};
  }

  ChessboardVisualizerComponent::ChessboardVisualizerComponent(const rclcpp::NodeOptions &options)
      : Node("chessboard_visualizer_component", options)
  {
    game_status_topic_ = declare_parameter<std::string>("game_status_topic", "/chess/game_status");
    raw_fen_topic_ = declare_parameter<std::string>("raw_fen_topic", "/chess/raw_fen");
    board_2d_topic_ = declare_parameter<std::string>("board_2d_topic", "/chess/board_2d/compressed");
    jpeg_quality_ = declare_parameter<int>("jpeg_quality", 85);
    board_panel_size_ = declare_parameter<int>("board_panel_size", 480);
    debug_ = declare_parameter<bool>("debug", false);
    render_rate_hz_ = declare_parameter<double>("render_rate_hz", 5.0);

    try
    {
      std::string share_dir = ament_index_cpp::get_package_share_directory("lekiwi_chess_master");
      pieces_dir_ = share_dir + "/resources/pieces";
    }
    catch (const std::exception &e)
    {
      pieces_dir_ = "resources/pieces";
    }

    RCLCPP_INFO(get_logger(),
                "Starting ChessboardVisualizerComponent (Status: %s, RawFEN: %s, 2D Topic: %s, Rate: %.1f Hz)",
                game_status_topic_.c_str(), raw_fen_topic_.c_str(), board_2d_topic_.c_str(), render_rate_hz_);

    board_2d_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
        board_2d_topic_, rclcpp::SensorDataQoS());

    game_status_sub_ = create_subscription<ChessGameStatus>(
        game_status_topic_, rclcpp::SystemDefaultsQoS(),
        std::bind(&ChessboardVisualizerComponent::gameStatusCallback, this, std::placeholders::_1));

    raw_fen_sub_ = create_subscription<std_msgs::msg::String>(
        raw_fen_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ChessboardVisualizerComponent::rawFenCallback, this, std::placeholders::_1));

    if (render_rate_hz_ > 0.0)
    {
      auto period = std::chrono::duration<double>(1.0 / render_rate_hz_);
      render_timer_ = create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(period),
          std::bind(&ChessboardVisualizerComponent::onRenderTimer, this));
    }
  }

  void ChessboardVisualizerComponent::gameStatusCallback(const ChessGameStatus::ConstSharedPtr msg)
  {
    if (!msg)
    {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_status_ = *msg;
      has_status_ = true;
      state_dirty_ = true;
    }
    renderAndPublish();
  }

  void ChessboardVisualizerComponent::rawFenCallback(const std_msgs::msg::String::ConstSharedPtr msg)
  {
    if (!msg)
    {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (latest_raw_fen_ != msg->data)
      {
        latest_raw_fen_ = msg->data;
        if (debug_)
        {
          state_dirty_ = true;
        }
      }
    }
    if (debug_)
    {
      renderAndPublish();
    }
  }

  void ChessboardVisualizerComponent::onRenderTimer()
  {
    bool should_render = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (state_dirty_)
      {
        should_render = true;
        state_dirty_ = false;
      }
    }
    if (should_render)
    {
      renderAndPublish();
    }
  }

  BoardLayout ChessboardVisualizerComponent::calculateLayout(int width, int height)
  {
    BoardLayout layout;
    layout.panel_w = width;
    layout.panel_h = height;
    layout.header_h = std::clamp(static_cast<int>(height * 0.08), 30, 50);
    layout.footer_h = std::clamp(static_cast<int>(height * 0.08), 30, 50);

    int board_avail_h = height - layout.header_h - layout.footer_h;
    int board_avail_w = width - 40;
    int board_box = std::min(board_avail_h, board_avail_w);
    layout.cell_size = std::max(board_box / 8, 10);
    layout.board_size = layout.cell_size * 8;

    layout.start_x = (width - layout.board_size) / 2;
    layout.start_y = layout.header_h + (board_avail_h - layout.board_size) / 2;
    return layout;
  }

  std::array<char, 64> ChessboardVisualizerComponent::parseFenToOccupancy(const std::string &fen)
  {
    std::array<char, 64> board{};
    board.fill('\0');
    std::stringstream ss(fen);
    std::string placement;
    ss >> placement;

    int r = 0;
    int c = 0;
    for (char ch : placement)
    {
      if (ch == '/')
      {
        r++;
        c = 0;
      }
      else if (std::isdigit(static_cast<unsigned char>(ch)))
      {
        c += (ch - '0');
      }
      else if (c < 8 && r < 8)
      {
        board[r * 8 + c] = ch;
        c++;
      }
    }
    return board;
  }

  bool ChessboardVisualizerComponent::parseSquareToCoord(
      const std::string &sq, const BoardLayout &layout, cv::Point &out_center)
  {
    if (sq.length() < 2)
    {
      return false;
    }
    char fc = static_cast<char>(std::tolower(static_cast<unsigned char>(sq[0])));
    char rc = sq[1];
    if (fc < 'a' || fc > 'h' || rc < '1' || rc > '8')
    {
      return false;
    }
    int c = fc - 'a';
    int r = 8 - (rc - '0');
    out_center.x = layout.start_x + c * layout.cell_size + layout.cell_size / 2;
    out_center.y = layout.start_y + r * layout.cell_size + layout.cell_size / 2;
    return true;
  }

  void ChessboardVisualizerComponent::tintSquare(
      cv::Mat &panel, const std::string &sq, const cv::Scalar &color, double alpha, const BoardLayout &layout)
  {
    cv::Point center;
    if (!parseSquareToCoord(sq, layout, center))
    {
      return;
    }
    int x = center.x - layout.cell_size / 2;
    int y = center.y - layout.cell_size / 2;
    if (x >= 0 && y >= 0 && x + layout.cell_size <= panel.cols && y + layout.cell_size <= panel.rows)
    {
      cv::Mat roi = panel(cv::Rect(x, y, layout.cell_size, layout.cell_size));
      cv::Mat color_mat(roi.size(), roi.type(), color);
      cv::addWeighted(roi, 1.0 - alpha, color_mat, alpha, 0, roi);
    }
  }

  void ChessboardVisualizerComponent::overlayAlphaSprite(
      cv::Mat &dst, const cv::Mat &sprite, int ox, int oy)
  {
    if (sprite.empty() || sprite.channels() != 4)
    {
      return;
    }
    int sw = sprite.cols;
    int sh = sprite.rows;
    if (ox < 0 || oy < 0 || ox + sw > dst.cols || oy + sh > dst.rows)
    {
      return;
    }

    cv::Mat roi = dst(cv::Rect(ox, oy, sw, sh));
    for (int y = 0; y < sh; ++y)
    {
      const cv::Vec4b *s_ptr = sprite.ptr<cv::Vec4b>(y);
      cv::Vec3b *d_ptr = roi.ptr<cv::Vec3b>(y);
      for (int x = 0; x < sw; ++x)
      {
        uchar alpha = s_ptr[x][3];
        if (alpha == 255)
        {
          d_ptr[x] = cv::Vec3b(s_ptr[x][0], s_ptr[x][1], s_ptr[x][2]);
        }
        else if (alpha > 0)
        {
          d_ptr[x][0] = (s_ptr[x][0] * alpha + d_ptr[x][0] * (255 - alpha)) / 255;
          d_ptr[x][1] = (s_ptr[x][1] * alpha + d_ptr[x][1] * (255 - alpha)) / 255;
          d_ptr[x][2] = (s_ptr[x][2] * alpha + d_ptr[x][2] * (255 - alpha)) / 255;
        }
      }
    }
  }

  void ChessboardVisualizerComponent::drawMoveArrow(
      cv::Mat &panel, const std::string &move_uci, const BoardLayout &layout, const cv::Scalar &color)
  {
    if (move_uci.length() < 4)
    {
      return;
    }
    cv::Point p_from, p_to;
    if (!parseSquareToCoord(move_uci.substr(0, 2), layout, p_from) ||
        !parseSquareToCoord(move_uci.substr(2, 2), layout, p_to))
    {
      return;
    }

    cv::Point2f dir(static_cast<float>(p_to.x - p_from.x), static_cast<float>(p_to.y - p_from.y));
    float len = std::hypot(dir.x, dir.y);
    if (len <= 1.0f)
    {
      return;
    }

    cv::Point2f norm_dir(dir.x / len, dir.y / len);
    float offset_start = static_cast<float>(layout.cell_size) * 0.15f;
    float offset_end = static_cast<float>(layout.cell_size) * 0.20f;

    cv::Point pt_start(
        p_from.x + static_cast<int>(norm_dir.x * offset_start),
        p_from.y + static_cast<int>(norm_dir.y * offset_start));
    cv::Point pt_end(
        p_to.x - static_cast<int>(norm_dir.x * offset_end),
        p_to.y - static_cast<int>(norm_dir.y * offset_end));

    int thickness = std::max(3, static_cast<int>(layout.cell_size * 0.08));

    // Shadow
    cv::arrowedLine(panel, pt_start, pt_end, cv::Scalar(15, 15, 15), thickness + 2, cv::LINE_AA, 0, 0.30);
    cv::circle(panel, pt_start, thickness + 2, cv::Scalar(15, 15, 15), -1, cv::LINE_AA);

    // Colored Arrow
    cv::arrowedLine(panel, pt_start, pt_end, color, thickness, cv::LINE_AA, 0, 0.30);
    cv::circle(panel, pt_start, thickness, color, -1, cv::LINE_AA);
  }

  void ChessboardVisualizerComponent::loadPieceSprites(int cell_size, const std::string &pieces_dir)
  {
    sprite_cache_.clear();
    cached_cell_size_ = cell_size;
    int icon_size = std::max(static_cast<int>(cell_size * 0.84), 8);

    for (const auto &[piece_char, filename] : kPiecePngNames)
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

  void ChessboardVisualizerComponent::drawBoardTiles(cv::Mat &panel, const BoardLayout &layout) const
  {
    const cv::Scalar light_tile(240, 217, 181); // #F0D9B5 (BGR)
    const cv::Scalar dark_tile(181, 136, 99);   // #B58863 (BGR)

    for (int r = 0; r < 8; ++r)
    {
      for (int c = 0; c < 8; ++c)
      {
        cv::Scalar tile_color = ((r + c) % 2 == 0) ? light_tile : dark_tile;
        int x1 = layout.start_x + c * layout.cell_size;
        int y1 = layout.start_y + r * layout.cell_size;
        cv::rectangle(panel, cv::Rect(x1, y1, layout.cell_size, layout.cell_size), tile_color, -1);
      }
    }
  }

  void ChessboardVisualizerComponent::drawMoveHighlights(
      cv::Mat &panel, const std::string &last_move, const std::string &best_move, const BoardLayout &layout) const
  {
    if (last_move.length() >= 4)
    {
      tintSquare(panel, last_move.substr(0, 2), cv::Scalar(80, 220, 80), 0.35, layout); // Soft green
      tintSquare(panel, last_move.substr(2, 2), cv::Scalar(80, 220, 80), 0.35, layout);
    }
    if (best_move.length() >= 4 && best_move != last_move)
    {
      tintSquare(panel, best_move.substr(0, 2), cv::Scalar(0, 165, 255), 0.30, layout); // Soft orange
      tintSquare(panel, best_move.substr(2, 2), cv::Scalar(0, 165, 255), 0.30, layout);
    }
  }

  int ChessboardVisualizerComponent::drawPieceSprites(
      cv::Mat &panel, const std::array<char, 64> &occupancy_board, const BoardLayout &layout)
  {
    int piece_count = 0;
    for (int r = 0; r < 8; ++r)
    {
      for (int c = 0; c < 8; ++c)
      {
        char piece = occupancy_board[r * 8 + c];
        if (piece == '\0')
        {
          continue;
        }
        piece_count++;
        std::string piece_str(1, piece);
        auto it = sprite_cache_.find(piece_str);
        if (it != sprite_cache_.end())
        {
          const cv::Mat &sprite = it->second;
          int x1 = layout.start_x + c * layout.cell_size;
          int y1 = layout.start_y + r * layout.cell_size;
          int ox = x1 + (layout.cell_size - sprite.cols) / 2;
          int oy = y1 + (layout.cell_size - sprite.rows) / 2;
          overlayAlphaSprite(panel, sprite, ox, oy);
        }
      }
    }
    return piece_count;
  }

  void ChessboardVisualizerComponent::drawMoveArrows(
      cv::Mat &panel, const std::string &last_move, const std::string &best_move, const BoardLayout &layout) const
  {
    // Green arrow for last executed move
    if (last_move.length() >= 4)
    {
      drawMoveArrow(panel, last_move, layout, cv::Scalar(40, 220, 60));
    }
    // Orange arrow for engine best move
    if (best_move.length() >= 4 && best_move != last_move)
    {
      drawMoveArrow(panel, best_move, layout, cv::Scalar(0, 180, 255));
    }
  }

  void ChessboardVisualizerComponent::drawBoardCoordinates(cv::Mat &panel, const BoardLayout &layout) const
  {
    cv::rectangle(panel, cv::Rect(layout.start_x, layout.start_y, layout.board_size, layout.board_size),
                  cv::Scalar(60, 70, 80), 2);

    double label_scale = std::max(layout.cell_size / 90.0, 0.40);
    for (int i = 0; i < 8; ++i)
    {
      std::string file_char(1, static_cast<char>('a' + i));
      int lx = layout.start_x + i * layout.cell_size + layout.cell_size / 2 - static_cast<int>(6 * label_scale);
      int ly = layout.start_y + layout.board_size + static_cast<int>(24 * label_scale);
      cv::putText(panel, file_char, cv::Point(lx, ly), cv::FONT_HERSHEY_SIMPLEX, label_scale,
                  cv::Scalar(180, 180, 180), 1, cv::LINE_AA);

      std::string rank_char = std::to_string(8 - i);
      int rx = layout.start_x - static_cast<int>(24 * label_scale);
      int ry = layout.start_y + i * layout.cell_size + layout.cell_size / 2 + static_cast<int>(6 * label_scale);
      cv::putText(panel, rank_char, cv::Point(rx, ry), cv::FONT_HERSHEY_SIMPLEX, label_scale,
                  cv::Scalar(180, 180, 180), 1, cv::LINE_AA);
    }
  }

  void ChessboardVisualizerComponent::drawHeaderAndFooter(
      cv::Mat &panel, const BoardDisplayContext &ctx, int piece_count, const BoardLayout &layout) const
  {
    // Header status
    std::string phase_str = "IDLE";
    cv::Scalar header_color(255, 220, 0);

    if (ctx.is_checkmate)
    {
      phase_str = "CHECKMATE";
      header_color = cv::Scalar(0, 0, 255);
    }
    else if (ctx.is_draw)
    {
      phase_str = "DRAW";
      header_color = cv::Scalar(180, 180, 180);
    }
    else if (ctx.is_check)
    {
      phase_str = "CHECK";
      header_color = cv::Scalar(0, 0, 255);
    }
    else
    {
      switch (ctx.game_phase)
      {
      case ChessGameStatus::PHASE_ROBOT_THINKING:
        phase_str = "THINKING...";
        header_color = cv::Scalar(0, 165, 255); // Orange
        break;
      case ChessGameStatus::PHASE_ROBOT_READY:
        phase_str = "ROBOT READY";
        header_color = cv::Scalar(80, 220, 80);
        break;
      case ChessGameStatus::PHASE_ROBOT_EXECUTING:
        phase_str = "ROBOT MOVING";
        header_color = cv::Scalar(0, 200, 255);
        break;
      case ChessGameStatus::PHASE_WAITING_PLAYER:
        phase_str = "PLAYER TURN";
        header_color = cv::Scalar(240, 240, 240);
        break;
      default:
        break;
      }
    }

    std::string header_text = (ctx.is_raw_view ? "[RAW VISION] " : "") +
                              phase_str + " (" + std::to_string(piece_count) + " pcs)";
    if (!ctx.last_move.empty())
    {
      header_text += " | Move: " + ctx.last_move;
    }
    if (!ctx.best_move.empty() && ctx.best_move != ctx.last_move)
    {
      header_text += " | Best: " + ctx.best_move;
    }

    cv::putText(panel, header_text, cv::Point(20, static_cast<int>(layout.header_h * 0.65)),
                cv::FONT_HERSHEY_SIMPLEX, 0.50, header_color, 1, cv::LINE_AA);

    // Footer FEN
    std::string fen_display = ctx.fen.length() <= 52 ? ctx.fen : ctx.fen.substr(0, 49) + "...";
    cv::putText(panel, "FEN: " + fen_display,
                cv::Point(20, layout.panel_h - static_cast<int>(layout.footer_h * 0.35)),
                cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
  }

  void ChessboardVisualizerComponent::render2DBoardPanel(
      cv::Mat &panel, const BoardDisplayContext &ctx)
  {
    BoardLayout layout = calculateLayout(board_panel_size_, board_panel_size_);
    panel = cv::Mat(layout.panel_h, layout.panel_w, CV_8UC3, cv::Scalar(26, 30, 35));

    if (cached_cell_size_ != layout.cell_size || sprite_cache_.empty())
    {
      loadPieceSprites(layout.cell_size, pieces_dir_);
    }

    drawBoardTiles(panel, layout);
    drawMoveHighlights(panel, ctx.last_move, ctx.best_move, layout);

    auto occupancy = parseFenToOccupancy(ctx.fen);
    int piece_count = drawPieceSprites(panel, occupancy, layout);

    drawMoveArrows(panel, ctx.last_move, ctx.best_move, layout);
    drawBoardCoordinates(panel, layout);
    drawHeaderAndFooter(panel, ctx, piece_count, layout);
  }

  BoardDisplayContext ChessboardVisualizerComponent::buildDisplayContext() const
  {
    auto ctx = has_status_ ? BoardDisplayContext::fromGameStatus(latest_status_)
                           : BoardDisplayContext{};

    // Override board representation with real-time camera vision when debug is active or before status is received
    if (!latest_raw_fen_.empty() && (debug_ || !has_status_))
    {
      ctx.fen = latest_raw_fen_;
      ctx.is_raw_view = true;
    }

    return ctx;
  }

  void ChessboardVisualizerComponent::renderAndPublish()
  {
    BoardDisplayContext ctx;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      ctx = buildDisplayContext();
    }

    cv::Mat panel;
    render2DBoardPanel(panel, ctx);

    std::vector<uchar> buf;
    const std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
    if (cv::imencode(".jpg", panel, buf, encode_params))
    {
      auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
      msg->header.stamp = now();
      msg->format = "jpeg";
      msg->data = std::move(buf);
      board_2d_pub_->publish(std::move(msg));
    }
  }

} // namespace lekiwi_chess_master

RCLCPP_COMPONENTS_REGISTER_NODE(lekiwi_chess_master::ChessboardVisualizerComponent)
