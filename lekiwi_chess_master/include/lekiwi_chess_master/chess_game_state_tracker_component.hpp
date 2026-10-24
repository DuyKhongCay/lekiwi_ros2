/**
 * @file chess_game_state_tracker_component.hpp
 * @brief ROS 2 Component tracking FIDE chess game state, vision debounce, and legal moves.
 *
 * Maintains the canonical chess::Board state machine for the LeKiwi robot.
 * Subscribes to raw perception FEN placements, applies temporal debounce filtering,
 * verifies transitions against legal FIDE move generation, and broadcasts game status.
 *
 * Implements clean Single Responsibility Principle (SRP) as a Player-Agnostic Referee.
 *
 * ROS 2 Interfaces:
 * - Subscription: `/chess/raw_fen` [std_msgs::msg::String, SensorDataQoS]
 * - Publisher: `/chess/game_status` [lekiwi_interfaces::msg::ChessGameStatus, Reliable, TransientLocal, Depth 1]
 * - Service Server: `/chess/reset_game` [std_srvs::srv::Trigger]
 * - Service Server: `/chess/set_fen` [lekiwi_interfaces::srv::SetFen]
 *
 * Parameters:
 * - `raw_fen_topic` (string, default: "/chess/raw_fen"): Vision detection topic.
 * - `game_status_topic` (string, default: "/chess/game_status"): Status broadcast topic.
 * - `set_fen_service` (string, default: "/chess/set_fen"): Set FEN service topic.
 * - `debounce_frames` (int, default: 3): Consecutive identical frames needed for stability.
 *
 * Thread Safety:
 * - Guarded by `state_mutex_` across subscriptions and service calls.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <lekiwi_interfaces/msg/chess_game_status.hpp>
#include <lekiwi_interfaces/msg/chess_move_details.hpp>
#include <lekiwi_interfaces/srv/set_fen.hpp>

#include <chess.hpp>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>

namespace lekiwi_chess_master
{

  /**
   * @brief Canonical standard FIDE starting piece placement string.
   */
  inline constexpr const char *kDefaultStartingPlacement =
      "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR";

  /**
   * @brief Tracks game status, filters camera vision, and validates FIDE moves.
   */
  class ChessGameStateTrackerComponent : public rclcpp::Node
  {
  public:
    using ChessGameStatus = lekiwi_interfaces::msg::ChessGameStatus;

    explicit ChessGameStateTrackerComponent(const rclcpp::NodeOptions &options);
    ~ChessGameStateTrackerComponent() override = default;

    /**
     * @brief Resets the internal chess board state to standard starting position.
     */
    void reset_game();

    /** @brief Accesses internal board representation (for testing). */
    [[nodiscard]] const chess::Board &get_board() const { return board_; }

    /** @brief Retrieves rich last executed move details message. */
    [[nodiscard]] const lekiwi_interfaces::msg::ChessMoveDetails &get_last_move_details() const { return last_move_details_; }

    /** @brief Checks if tracker is currently awaiting physical board sync with imported FEN. */
    [[nodiscard]] bool is_awaiting_physical_sync() const
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      return awaiting_physical_sync_;
    }

    /**
     * @brief Imports and validates a FEN string, setting tracker into awaiting physical board sync.
     *
     * @param[in] fen_str FEN string to import.
     * @param[out] canonical_fen Formatted canonical 6-field FEN.
     * @param[out] active_color Color whose turn it is ("w" or "b").
     * @param[out] message Diagnostic status message.
     * @return True if FEN was valid and imported, false otherwise.
     */
    bool set_fen(
        std::string_view fen_str,
        std::string &canonical_fen,
        std::string &active_color,
        std::string &message);

  private:
    /**
     * @brief Ingests vision detection FEN placement and runs temporal debounce filtering.
     *
     * @param[in] msg Incoming raw FEN string message from camera pipeline.
     */
    void raw_fen_callback(const std_msgs::msg::String::ConstSharedPtr msg);

    /**
     * @brief Service handler resetting game board to starting position upon external trigger.
     */
    void handle_reset_service(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    /**
     * @brief Service handler importing and configuring a FEN position.
     */
    void handle_set_fen_service(
        const std::shared_ptr<lekiwi_interfaces::srv::SetFen::Request> request,
        std::shared_ptr<lekiwi_interfaces::srv::SetFen::Response> response);

    /**
     * @brief Matches a stable vision placement against all legal FIDE candidate moves.
     *
     * @param[in] detected_placement Piece placement string token from camera vision.
     * @param[out] matched_move Populated chess::Move if a unique match is found.
     * @return True if detected placement matches exactly one legal candidate move.
     */
    bool match_legal_move(const std::string &detected_placement, chess::Move &matched_move);

    /**
     * @brief Constructs and publishes consolidated ChessGameStatus message.
     *
     * @param[in] is_legal True if board reflects a legal game state.
     * @param[in] is_stable True if vision placement has satisfied debounce threshold.
     */
    void publish_game_status(bool is_legal = true, bool is_stable = true);

    // Node Parameters
    std::string raw_fen_topic_{"/chess/raw_fen"};
    std::string game_status_topic_{"/chess/game_status"};
    std::string set_fen_service_name_{"/chess/set_fen"};
    int debounce_frames_{3};

    // ROS 2 Communications
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr raw_fen_sub_;
    rclcpp::Publisher<ChessGameStatus>::SharedPtr game_status_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
    rclcpp::Service<lekiwi_interfaces::srv::SetFen>::SharedPtr set_fen_srv_;

    // Game Engine & Debounce tracking
    mutable std::mutex state_mutex_;
    chess::Board board_;
    std::string last_accepted_placement_{kDefaultStartingPlacement};
    std::string pending_placement_;
    int consecutive_count_{0};
    bool awaiting_physical_sync_{false};

    // Match Status
    lekiwi_interfaces::msg::ChessMoveDetails last_move_details_;
  };

} // namespace lekiwi_chess_master
