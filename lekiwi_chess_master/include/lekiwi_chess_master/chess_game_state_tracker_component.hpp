/**
 * @file chess_game_state_tracker_component.hpp
 * @brief ROS 2 Component tracking FIDE chess game state, vision debounce, and legal moves.
 *
 * Maintains the canonical chess::Board state machine for the LeKiwi robot.
 * Subscribes to raw perception FEN placements, applies temporal debounce filtering,
 * verifies transitions against legal FIDE move generation, broadcasts game status,
 * and orchestrates autonomous robot move generation via Action Client.
 *
 * ROS 2 Interfaces:
 * - Subscription: `/chess/raw_fen` [std_msgs::msg::String, SensorDataQoS]
 * - Publisher: `/chess/game_status` [lekiwi_interfaces::msg::ChessGameStatus, SystemDefaultsQoS]
 * - Service Server: `/chess/reset_game` [std_srvs::srv::Trigger]
 * - Action Client: `/chess/compute_best_move` [lekiwi_interfaces::action::ComputeBestMove]
 *
 * Parameters:
 * - `raw_fen_topic` (string, default: "/chess/raw_fen"): Vision detection topic.
 * - `game_status_topic` (string, default: "/chess/game_status"): Status broadcast topic.
 * - `debounce_frames` (int, default: 3): Consecutive identical frames needed for stability.
 * - `auto_trigger_engine` (bool, default: false): Auto-dispatch calculation on robot's turn.
 * - `robot_color` (string, default: "black"): Designates robot's playing side ("white"/"black").
 * - `think_time_ms` (int, default: 1000): Engine calculation time budget.
 * - `action_name` (string, default: "/chess/compute_best_move"): Action server target.
 *
 * Thread Safety:
 * - Guarded by `state_mutex_` across subscriptions, service calls, and action callbacks.
 * - `is_engine_busy_` provides lock-free re-entrancy prevention for engine triggers.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <lekiwi_interfaces/msg/chess_game_status.hpp>
#include <lekiwi_interfaces/msg/chess_move_details.hpp>
#include <lekiwi_interfaces/action/compute_best_move.hpp>

#include <chess.hpp>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>

namespace lekiwi_chess_master
{

  /**
   * @brief Canonical standard FIDE starting piece placement string.
   */
  inline constexpr const char *kDefaultStartingPlacement =
      "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR";

  /**
   * @brief Tracks game status, filters camera vision, and coordinates chess game loops.
   */
  class ChessGameStateTrackerComponent : public rclcpp::Node
  {
  public:
    using ChessGameStatus = lekiwi_interfaces::msg::ChessGameStatus;
    using ComputeBestMove = lekiwi_interfaces::action::ComputeBestMove;
    using GoalHandleComputeBestMove = rclcpp_action::ClientGoalHandle<ComputeBestMove>;

    explicit ChessGameStateTrackerComponent(const rclcpp::NodeOptions &options);
    ~ChessGameStateTrackerComponent() override = default;

    /**
     * @brief Resets the internal chess board state to standard starting position.
     * Cancels any pending or active engine calculation goals.
     */
    void reset_game();

    /** @brief Accesses internal board representation (for testing). */
    [[nodiscard]] const chess::Board &get_board() const { return board_; }

    /** @brief Checks if autonomous engine calculation triggering is enabled. */
    [[nodiscard]] bool is_auto_trigger_enabled() const { return auto_trigger_engine_; }

    /** @brief Retrieves the configured robot player color ("white" or "black"). */
    [[nodiscard]] const std::string &get_robot_color() const { return robot_color_; }

    /** @brief Retrieves the engine think time in milliseconds. */
    [[nodiscard]] int get_think_time_ms() const { return think_time_ms_; }

    /** @brief Retrieves the current game lifecycle phase enum value. */
    [[nodiscard]] uint8_t get_game_phase() const { return current_phase_; }

    /** @brief Retrieves current best move UCI string. */
    [[nodiscard]] const std::string &get_best_move() const { return best_move_details_.uci; }

    /** @brief Retrieves rich best move details message. */
    [[nodiscard]] const lekiwi_interfaces::msg::ChessMoveDetails &get_best_move_details() const { return best_move_details_; }

    /** @brief Retrieves rich last executed move details message. */
    [[nodiscard]] const lekiwi_interfaces::msg::ChessMoveDetails &get_last_move_details() const { return last_move_details_; }

  private:
    /**
     * @brief Ingests vision detection FEN placement and runs temporal debounce filtering.
     *
     * @details Increments consecutive stability counter for matching incoming frames.
     * When stable, evaluates if the new piece placement matches the starting board or
     * a legal FIDE move. If valid, updates board state and triggers engine if needed.
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
     * @brief Matches a stable vision placement against all legal FIDE candidate moves.
     *
     * @details Generates all legal moves for current board state, simulates each move
     * on a temporary sandbox, extracts piece placement, and compares with detected placement.
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

    /**
     * @brief Evaluates whether to automatically dispatch ComputeBestMove goal to engine.
     *
     * @details Checks if auto_trigger_engine is active, if it is currently robot's turn,
     * if the game is active (not checkmate/draw), and if action server is available.
     *
     * @pre state_mutex_ must be held by caller.
     * @post Dispatches async goal handle if criteria are satisfied.
     */
    void trigger_engine_if_needed();

    /** @brief Action client callback handling server goal response acceptance/rejection. */
    void on_goal_response(const GoalHandleComputeBestMove::SharedPtr &goal_handle);

    /** @brief Action client callback receiving intermediate search progression feedback. */
    void on_feedback(
        GoalHandleComputeBestMove::SharedPtr,
        const std::shared_ptr<const ComputeBestMove::Feedback> feedback);

    /** @brief Action client callback processing final best move evaluation result. */
    void on_result(const GoalHandleComputeBestMove::WrappedResult &result);

    // Node Parameters
    std::string raw_fen_topic_{"/chess/raw_fen"};
    std::string game_status_topic_{"/chess/game_status"};
    int debounce_frames_{3};
    bool auto_trigger_engine_{false};
    std::string robot_color_{"black"};
    int think_time_ms_{1000};
    std::string action_name_{"/chess/compute_best_move"};

    // ROS 2 Communications
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr raw_fen_sub_;
    rclcpp::Publisher<ChessGameStatus>::SharedPtr game_status_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
    rclcpp_action::Client<ComputeBestMove>::SharedPtr action_client_;

    // Game Engine & Debounce tracking
    mutable std::mutex state_mutex_;
    chess::Board board_;
    std::string last_accepted_placement_{kDefaultStartingPlacement};
    std::string pending_placement_;
    int consecutive_count_{0};

    // Match & Engine Status
    lekiwi_interfaces::msg::ChessMoveDetails last_move_details_;
    lekiwi_interfaces::msg::ChessMoveDetails best_move_details_;
    int32_t current_eval_centipawns_{0};
    uint8_t current_phase_{ChessGameStatus::PHASE_WAITING_PLAYER};

    // Action Client state
    std::atomic<bool> is_engine_busy_{false};
    GoalHandleComputeBestMove::SharedPtr current_goal_handle_;
  };

} // namespace lekiwi_chess_master
