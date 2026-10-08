/**
 * @file stockfish_driver.hpp
 * @brief Non-blocking POSIX pipe driver for Stockfish Universal Chess Interface (UCI) engine.
 *
 * Spawns Stockfish as an asynchronous child process using POSIX pipes (stdin/stdout).
 * Implements non-blocking poll-driven I/O and parses standard UCI commands ("uci",
 * "isready", "position", "go", "stop", "info", "bestmove") with cancellation support.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#pragma once

#include <chrono>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <sys/types.h>

namespace lekiwi_chess_master
{

  /**
   * @brief Search evaluation and search tree progression metrics parsed from UCI "info" stream.
   */
  struct EngineInfoFeedback
  {
    uint32_t depth{0};       ///< Completed search iteration depth (plies).
    int32_t score_cp{0};     ///< Position evaluation in centipawns (from engine's perspective).
    bool is_mate{false};     ///< True if engine discovered a forced mate sequence.
    int32_t mate_in{0};      ///< Distance to checkmate in plies (positive = engine wins).
    uint64_t nps{0};         ///< Search speed in nodes per second.
    std::string pv;          ///< Principal variation (best projected sequence of UCI moves).
  };

  /**
   * @brief Best move determination and search outcome parsed from UCI "bestmove" output.
   */
  struct BestMoveResult
  {
    bool success{false};     ///< True if a legal best move was computed successfully.
    std::string best_move;   ///< Recommended move in UCI notation (e.g. "e2e4", "e7e8q").
    std::string ponder;      ///< Engine's projected opponent response for ponder caching.
    int32_t score_cp{0};     ///< Final evaluation centipawns of the chosen variation.
    bool is_mate{false};     ///< True if the chosen line leads to forced mate.
    int32_t mate_in{0};      ///< Moves until mate in the chosen line.
    std::string message;     ///< Diagnostic status or error explanation on failure.
  };

  /**
   * @brief Manages Stockfish child process lifecycle, bidirectional pipes, and UCI protocol I/O.
   *
   * @details All process management operations (spawning, termination, sending commands,
   * reading responses) are protected by an internal recursive-free std::mutex (`process_mutex_`).
   */
  class StockfishDriver
  {
  public:
    StockfishDriver();
    ~StockfishDriver();

    // Prevent copying
    StockfishDriver(const StockfishDriver &) = delete;
    StockfishDriver &operator=(const StockfishDriver &) = delete;

    /**
     * @brief Spawns the Stockfish binary child process and performs UCI handshake.
     *
     * @details Sets up unidirectional POSIX pipes for child stdin/stdout, forks child
     * process, configures non-blocking flags (O_NONBLOCK) on the read pipe, and executes
     * initial handshake sequence ("uci" -> "isready" -> "readyok").
     *
     * @param[in] executable_path Absolute filesystem path to stockfish binary.
     * @return True if child process was spawned and responded with "readyok" within timeout.
     *
     * @pre Executable binary must exist and have execute permissions.
     * @post On success, engine_pid_ > 0 and pipes are open for communication.
     * @note If the handshake times out, the child process is terminated immediately.
     */
    bool start(const std::string &executable_path = "/usr/games/stockfish");

    /**
     * @brief Gracefully terminates the child engine process via UCI "quit" and waitpid(2).
     */
    void stop();

    /**
     * @brief Inspects whether the Stockfish child process is currently alive and responsive.
     * @return True if engine PID is valid and waitpid(WNOHANG) indicates process is running.
     */
    bool is_running() const;

    /**
     * @brief Dispatches UCI "stop\n" to immediately halt search computation.
     */
    void send_uci_stop();

    /**
     * @brief Computes the optimal move for a given FEN position with streaming feedback.
     *
     * @details Issues "position fen <fen>" and "go movetime <ms> depth <d>" commands to the
     * engine. Polls stdout continuously for "info" and "bestmove" tokens while invoking
     * feedback callbacks and evaluating cancellation flags.
     *
     * @param[in] fen 6-token standard FEN string representing board state.
     * @param[in] think_time_ms Maximum calculation time allocated to engine in milliseconds.
     * @param[in] depth Maximum search depth limit in plies (0 for unconstrained depth).
     * @param[in] on_feedback Optional callback invoked on intermediate UCI "info" stream lines.
     * @param[in] is_canceled Optional predicate query checked each loop to abort search early.
     * @return BestMoveResult populated with move coordinates, ponder move, and evaluation score.
     *
     * @pre start() must have returned true and is_running() must be true.
     * @post The engine returns to an idle waiting state ready for the next position.
     * @note Thread-safe: serializes access to POSIX pipes via process_mutex_.
     */
    BestMoveResult compute_best_move(
        const std::string &fen,
        uint32_t think_time_ms = 1000,
        uint32_t depth = 0,
        std::function<void(const EngineInfoFeedback &)> on_feedback = nullptr,
        std::function<bool()> is_canceled = nullptr);

    /**
     * @brief Parses a raw UCI "info" stream line into structured EngineInfoFeedback.
     *
     * @details Extracts search metrics including iteration depth, centipawn or mate scores,
     * nodes-per-second, and principal variation (pv) move tokens.
     *
     * @param[in] line Raw text line emitted by Stockfish stdout.
     * @param[out] feedback Target struct receiving parsed search values.
     * @return True if the line was a valid UCI "info" string and parsed successfully.
     */
    static bool parse_info_line(const std::string &line, EngineInfoFeedback &feedback);

    /**
     * @brief Parses a raw UCI "bestmove" response line into a BestMoveResult struct.
     *
     * @param[in] line Raw text line starting with "bestmove ".
     * @param[out] result Target struct receiving move token and optional ponder move.
     * @return True if line began with "bestmove " and was parsed.
     */
    static bool parse_bestmove_line(const std::string &line, BestMoveResult &result);

  private:
    /**
     * @brief Writes a command string to the child stdin pipe descriptor.
     */
    bool send_command(const std::string &cmd);

    /**
     * @brief Reads a single newline-terminated line from non-blocking child stdout pipe.
     *
     * @details Accumulates incoming byte chunks into line_buffer_ using poll(2) with timeout
     * until a newline character is encountered.
     *
     * @param[in] timeout_ms Maximum duration in milliseconds to await incoming data chunks.
     * @return Extracted line string without trailing newline, or empty string on timeout.
     */
    std::string read_line(int timeout_ms = 100);

    mutable std::mutex process_mutex_; ///< Serializes process control and pipe I/O.
    pid_t engine_pid_{-1};             ///< Process ID of spawned Stockfish subprocess.
    int in_pipe_fd_{-1};               ///< Write descriptor connected to child stdin.
    int out_pipe_fd_{-1};              ///< Non-blocking read descriptor connected to child stdout.
    std::string line_buffer_;          ///< Accumulator buffer for partial pipe read chunks.
  };

} // namespace lekiwi_chess_master
