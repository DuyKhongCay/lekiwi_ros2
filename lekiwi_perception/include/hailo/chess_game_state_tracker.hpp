/**
 * @file chess_game_state_tracker.hpp
 * @brief In-process chess game state tracking, debounce filtering, and legal move validation.
 *        Integrates Disservin/chess-library in-process for FIDE chess rules compliance.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#ifndef LEKIWI_PERCEPTION__HAILO__CHESS_GAME_STATE_TRACKER_HPP_
#define LEKIWI_PERCEPTION__HAILO__CHESS_GAME_STATE_TRACKER_HPP_

#include <chess.hpp>
#include <string>

namespace lekiwi_perception::hailo
{

    /**
     * @brief Result of evaluating a vision-detected piece placement against game rules.
     */
    struct GameStateResult
    {
        /// Full standard 6-field FEN string with castling, en-passant, turn, and move counters.
        std::string full_fen{"rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"};
        /// Last detected legal move in UCI format (e.g. "e2e4"). Empty if no new move.
        std::string last_move;
        /// True if detected piece placement matches a legal move or initial setup.
        bool is_legal_move{false};
        /// True if the piece placement has remained stable across debounce window.
        bool is_board_stable{false};
    };

    /**
     * @brief In-process Game State Tracker managing board history, debounce, and legal move validation.
     */
    class ChessGameStateTracker
    {
    public:
        explicit ChessGameStateTracker(int debounce_frames = 3);

        /**
         * @brief Updates game state tracker with new vision piece placement.
         * @param[in] detected_placement 8-rank piece placement string (e.g. "rnbqkbnr/pppppppp/...").
         * @return GameStateResult containing full FEN and move validation status.
         */
        GameStateResult update(const std::string &detected_placement);

        /**
         * @brief Resets game board to standard starting position.
         */
        void reset();

        /**
         * @brief Gets current full FEN.
         */
        [[nodiscard]] std::string get_full_fen() const;

        /**
         * @brief Gets current internal board representation.
         */
        [[nodiscard]] const chess::Board &get_board() const { return board_; }

    private:
        bool match_legal_move(const std::string &detected_placement, chess::Move &matched_move);

        chess::Board board_;
        int debounce_frames_{3};
        std::string last_accepted_placement_{"rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR"};
        std::string pending_placement_;
        int consecutive_count_{0};
    };

} // namespace lekiwi_perception::hailo

#endif // LEKIWI_PERCEPTION__HAILO__CHESS_GAME_STATE_TRACKER_HPP_
