/**
 * @file chessboard_2d_visualizer.hpp
 * @brief 2D Top-down digital chessboard panel renderer and compressed image publisher.
 *
 * Generates synthetic top-down 2D chessboard panels overlaid with high-resolution piece
 * sprites, move highlight tints, vector motion arrows, and game phase telemetry. Emits
 * lightweight compressed JPEG frames for web dashboards, RViz, and operator monitors.
 *
 * ROS 2 Interfaces:
 * - Subscriptions:
 *   - `/chess/game_status` [lekiwi_interfaces::msg::ChessGameStatus, SystemDefaultsQoS]
 *   - `/chess/raw_fen` [std_msgs::msg::String, SensorDataQoS]
 * - Publisher:
 *   - `/chess/board_2d/compressed` [sensor_msgs::msg::CompressedImage, SensorDataQoS]
 *
 * Parameters:
 * - `game_status_topic` (string, default: "/chess/game_status"): Status telemetry input topic.
 * - `raw_fen_topic` (string, default: "/chess/raw_fen"): Raw camera vision input topic.
 * - `board_2d_topic` (string, default: "/chess/board_2d/compressed"): Output JPEG image topic.
 * - `jpeg_quality` (int, default: 85): JPEG compression quality (1-100).
 * - `board_panel_size` (int, default: 480): Square dimension of generated image in pixels.
 * - `debug` (bool, default: false): Enable raw camera vision overlay visualization.
 * - `render_rate_hz` (double, default: 5.0): Maximum rendering loop frequency in Hertz.
 *
 * Thread Safety:
 * - Protected by `state_mutex_` across subscription callbacks, timer ticks, and render steps.
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

  /**
   * @brief Pixel geometric dimensions and layout coordinates for rendering chessboard components.
   */
  struct BoardLayout
  {
    int panel_w{480};      ///< Total image canvas width in pixels.
    int panel_h{480};      ///< Total image canvas height in pixels.
    int header_h{38};      ///< Vertical height of top status header area.
    int footer_h{38};      ///< Vertical height of bottom FEN banner area.
    int board_size{384};    ///< Total pixel span of 8x8 tile matrix.
    int cell_size{48};     ///< Pixel dimension of an individual square tile.
    int start_x{48};       ///< Top-left horizontal origin coordinate of board grid.
    int start_y{48};       ///< Top-left vertical origin coordinate of board grid.
  };

  /**
   * @brief Consolidated snapshot of chess match state consumed by rendering pipeline.
   */
  struct BoardDisplayContext
  {
    std::string fen{"rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"}; ///< Position FEN.
    std::string last_move;       ///< UCI string of last executed move (highlighted in green).
    std::string best_move;       ///< UCI string of engine recommended move (highlighted in orange).
    int32_t eval_cp{0};          ///< Engine evaluation score in centipawns.
    uint8_t game_phase{0};       ///< Current game lifecycle phase enum value.
    bool is_check{false};        ///< True if active side is in check.
    bool is_checkmate{false};    ///< True if position is terminal checkmate.
    bool is_draw{false};         ///< True if position is terminal stalemate or draw.
    bool is_raw_view{false};     ///< True if rendering direct unprocessed camera vision FEN.

    /**
     * @brief Constructs a BoardDisplayContext snapshot from a ROS 2 ChessGameStatus message.
     */
    static BoardDisplayContext fromGameStatus(const lekiwi_interfaces::msg::ChessGameStatus &status)
    {
      BoardDisplayContext ctx;
      ctx.fen = status.full_fen;
      ctx.last_move = status.last_move_details.uci;
      ctx.best_move = status.best_move_details.uci;
      ctx.eval_cp = status.eval_centipawns;
      ctx.game_phase = status.game_phase;
      ctx.is_check = status.is_check;
      ctx.is_checkmate = status.is_checkmate;
      ctx.is_draw = status.is_draw;
      return ctx;
    }
  };

  /**
   * @brief Headless ROS 2 component rendering synthetic 2D top-down chessboard into JPEG image.
   */
  class Chessboard2DVisualizer : public rclcpp::Node
  {
  public:
    using ChessGameStatus = lekiwi_interfaces::msg::ChessGameStatus;

    explicit Chessboard2DVisualizer(const rclcpp::NodeOptions &options);
    ~Chessboard2DVisualizer() override = default;

  private:
    /** @brief Callback receiving authoritative game status updates. */
    void gameStatusCallback(const ChessGameStatus::ConstSharedPtr msg);

    /** @brief Callback receiving raw unverified vision detection strings for debug overlay. */
    void rawFenCallback(const std_msgs::msg::String::ConstSharedPtr msg);

    /** @brief Wall timer callback rendering frames when state is flagged dirty. */
    void onRenderTimer();

    /** @brief Orchestrates full render pipeline and emits compressed JPEG message. */
    void renderAndPublish();

    /** @brief Compiles current status and parameters into an immutable render snapshot. */
    BoardDisplayContext buildDisplayContext() const;

    /**
     * @brief Renders all visual layers (tiles, highlights, sprites, arrows, labels) into canvas.
     * @param[out] panel Destination OpenCV 3-channel BGR matrix.
     * @param[in] ctx Game display context containing board layout and active moves.
     */
    void render2DBoardPanel(cv::Mat &panel, const BoardDisplayContext &ctx);

    /** @brief Computes centered square and board bounds from overall canvas dimensions. */
    static BoardLayout calculateLayout(int width, int height);

    /**
     * @brief Parses FEN piece placement string into a 64-element row-major piece char array.
     * @param[in] fen Standard FEN string.
     * @return 64-element array where index = row * 8 + col ('\0' represents empty square).
     */
    static std::array<char, 64> parseFenToOccupancy(const std::string &fen);

    /** @brief Converts algebraic square identifier (e.g. "e4") to canvas pixel center point. */
    static bool parseSquareToCoord(const std::string &sq, const BoardLayout &layout, cv::Point &out_center);

    /** @brief Applies translucent color tint overlay on a specific board square. */
    static void tintSquare(cv::Mat &panel, const std::string &sq, const cv::Scalar &color, double alpha, const BoardLayout &layout);

    /**
     * @brief Draws an anti-aliased directional motion vector arrow between two squares.
     * @param[in,out] panel Destination image matrix.
     * @param[in] move_uci 4-character UCI move string ("e2e4").
     * @param[in] layout Layout coordinates for square offsets.
     * @param[in] color BGR arrow color.
     */
    static void drawMoveArrow(cv::Mat &panel, const std::string &move_uci, const BoardLayout &layout, const cv::Scalar &color);

    /**
     * @brief Blends a 4-channel BGRA transparent sprite over a 3-channel BGR image canvas.
     * @param[in,out] dst Destination BGR canvas.
     * @param[in] sprite Source BGRA image containing alpha channel.
     * @param[in] ox Top-left horizontal destination offset.
     * @param[in] oy Top-left vertical destination offset.
     */
    static void overlayAlphaSprite(cv::Mat &dst, const cv::Mat &sprite, int ox, int oy);

    /** @brief Pre-loads and scales piece PNG assets from disk into internal memory cache. */
    void loadPieceSprites(int cell_size, const std::string &pieces_dir);

    /** @brief Renders the alternating 8x8 light and dark checkered board tiles. */
    void drawBoardTiles(cv::Mat &panel, const BoardLayout &layout) const;

    /** @brief Highlights previous move squares (green) and engine best move squares (orange). */
    void drawMoveHighlights(cv::Mat &panel, const std::string &last_move, const std::string &best_move, const BoardLayout &layout) const;

    /** @brief Iterates 64-square occupancy array and stamps piece sprites onto canvas. */
    int drawPieceSprites(cv::Mat &panel, const std::array<char, 64> &occupancy_board, const BoardLayout &layout);

    /** @brief Draws directional arrows for executed and recommended moves. */
    void drawMoveArrows(cv::Mat &panel, const std::string &last_move, const std::string &best_move, const BoardLayout &layout) const;

    /** @brief Renders coordinate labels [a-h] and [1-8] along the perimeter margin. */
    void drawBoardCoordinates(cv::Mat &panel, const BoardLayout &layout) const;

    /**
     * @brief Draws top header banner and bottom FEN footer with phase-specific styling.
     * @param[in,out] panel Destination image matrix.
     * @param[in] ctx Match display context.
     * @param[in] piece_count Total active pieces detected on board.
     * @param[in] layout Board layout metrics.
     */
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
