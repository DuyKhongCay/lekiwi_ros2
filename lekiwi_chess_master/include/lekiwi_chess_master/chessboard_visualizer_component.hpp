/**
 * @file chessboard_visualizer_component.hpp
 * @brief 2D Top-down digital chessboard panel renderer and compressed image publisher.
 *
 * Subscribes to /chess/game_status and /chess/raw_fen.
 * Publishes /chess/board_2d/compressed.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#pragma once

#include <array>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/string.hpp>
#include <lekiwi_interfaces/msg/chess_game_status.hpp>

namespace lekiwi_chess_master
{

  struct BoardLayout
  {
    int panel_w{480};
    int panel_h{480};
    int header_h{38};
    int footer_h{38};
    int board_size{384};
    int cell_size{48};
    int start_x{48};
    int start_y{48};
  };

  struct BoardDisplayContext
  {
    std::string fen;
    std::string last_move;
    std::string best_move;
    int32_t eval_cp{0};
    uint8_t game_phase{0};
    bool is_check{false};
    bool is_checkmate{false};
    bool is_draw{false};
    bool is_raw_view{false};
  };

  /**
   * @brief Headless component rendering synthetic 2D top-down chessboard into JPEG image.
   */
  class ChessboardVisualizerComponent : public rclcpp::Node
  {
  public:
    using ChessGameStatus = lekiwi_interfaces::msg::ChessGameStatus;

    explicit ChessboardVisualizerComponent(const rclcpp::NodeOptions &options);
    ~ChessboardVisualizerComponent() override = default;

  private:
    // Callbacks
    void gameStatusCallback(const ChessGameStatus::ConstSharedPtr msg);
    void rawFenCallback(const std_msgs::msg::String::ConstSharedPtr msg);
    void onRenderTimer();

    // High-level rendering orchestration
    void renderAndPublish();
    void render2DBoardPanel(cv::Mat &panel, const BoardDisplayContext &ctx);

    // Focused helper methods
    static BoardLayout calculateLayout(int width, int height);
    static std::array<char, 64> parseFenToOccupancy(const std::string &fen);
    static bool parseSquareToCoord(const std::string &sq, const BoardLayout &layout, cv::Point &out_center);
    static void tintSquare(cv::Mat &panel, const std::string &sq, const cv::Scalar &color, double alpha, const BoardLayout &layout);
    static void drawMoveArrow(cv::Mat &panel, const std::string &move_uci, const BoardLayout &layout, const cv::Scalar &color);
    static void overlayAlphaSprite(cv::Mat &dst, const cv::Mat &sprite, int ox, int oy);

    void loadPieceSprites(int cell_size, const std::string &pieces_dir);
    void drawBoardTiles(cv::Mat &panel, const BoardLayout &layout) const;
    void drawMoveHighlights(cv::Mat &panel, const std::string &last_move, const std::string &best_move, const BoardLayout &layout) const;
    int drawPieceSprites(cv::Mat &panel, const std::array<char, 64> &occupancy_board, const BoardLayout &layout);
    void drawMoveArrows(cv::Mat &panel, const std::string &last_move, const std::string &best_move, const BoardLayout &layout) const;
    void drawBoardCoordinates(cv::Mat &panel, const BoardLayout &layout) const;
    void drawHeaderAndFooter(cv::Mat &panel, const BoardDisplayContext &ctx, int piece_count, const BoardLayout &layout) const;

    // Subscriptions & Publishers
    rclcpp::Subscription<ChessGameStatus>::SharedPtr game_status_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr raw_fen_sub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr board_2d_pub_;
    rclcpp::TimerBase::SharedPtr render_timer_;

    // Parameters
    std::string game_status_topic_{"/chess/game_status"};
    std::string raw_fen_topic_{"/chess/raw_fen"};
    std::string board_2d_topic_{"/chess/board_2d/compressed"};
    int jpeg_quality_{85};
    int board_panel_size_{480};
    bool debug_{false};
    double render_rate_hz_{5.0};

    // State & Cache
    mutable std::mutex state_mutex_;
    ChessGameStatus latest_status_;
    std::string latest_raw_fen_;
    bool has_status_{false};
    bool state_dirty_{true};

    std::string pieces_dir_;
    std::map<std::string, cv::Mat> sprite_cache_;
    int cached_cell_size_{0};
  };

} // namespace lekiwi_chess_master
