/**
 * @file stockfish_driver.cpp
 * @brief Implementation of non-blocking POSIX pipe Stockfish UCI engine driver.
 *
 * Manages child process lifecycle, IPC via redirected file descriptors, non-blocking
 * polling I/O, and line-oriented UCI stream parsing.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include "lekiwi_chess_master/stockfish_driver.hpp"

#include <csignal>
#include <fcntl.h>
#include <poll.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <thread>
#include <vector>

namespace fs = std::filesystem;

namespace lekiwi_chess_master
{

  StockfishDriver::StockfishDriver()
  {
    // Ignore SIGPIPE globally for pipe writes to avoid crashing the composable container
    struct sigaction sa;
    std::memset(&sa, 0, sizeof(sa));
    sa.sa_handler = SIG_IGN;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGPIPE, &sa, nullptr);
  }

  StockfishDriver::~StockfishDriver()
  {
    stop();
  }

  bool StockfishDriver::start(const std::string &executable_path)
  {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (engine_pid_ > 0)
    {
      return true; // Subprocess is already spawned and active
    }

    std::string exec_path = executable_path;
    if (!fs::exists(exec_path))
    {
      std::cerr << "[StockfishDriver] Stockfish binary not found at configured path: " << executable_path << std::endl;
      return false;
    }

    int in_pipe[2] = {-1, -1};
    int out_pipe[2] = {-1, -1};
    if (pipe(in_pipe) != 0)
    {
      std::cerr << "[StockfishDriver] Failed to create child stdin POSIX pipe." << std::endl;
      return false;
    }
    if (pipe(out_pipe) != 0)
    {
      std::cerr << "[StockfishDriver] Failed to create child stdout POSIX pipe." << std::endl;
      close(in_pipe[0]);
      close(in_pipe[1]);
      return false;
    }

    pid_t pid = fork();
    if (pid < 0)
    {
      std::cerr << "[StockfishDriver] Failed to fork Stockfish process." << std::endl;
      close(in_pipe[0]);
      close(in_pipe[1]);
      close(out_pipe[0]);
      close(out_pipe[1]);
      return false;
    }

    if (pid == 0)
    {
      // Child process: Redirect child stdin from in_pipe[0] and stdout to out_pipe[1]
      dup2(in_pipe[0], STDIN_FILENO);
      dup2(out_pipe[1], STDOUT_FILENO);

      // Close all inherited pipe descriptors in child
      close(in_pipe[0]);
      close(in_pipe[1]);
      close(out_pipe[0]);
      close(out_pipe[1]);

      // Replace child address space with Stockfish binary
      execl(exec_path.c_str(), exec_path.c_str(), nullptr);
      _exit(127); // Exit if execl fails
    }

    // Parent process: retain write descriptor for in_pipe and read descriptor for out_pipe
    {
      std::lock_guard<std::mutex> stdin_lock(stdin_mutex_);
      in_pipe_fd_ = in_pipe[1];
    }
    engine_pid_ = pid;
    out_pipe_fd_ = out_pipe[0];

    close(in_pipe[0]);
    close(out_pipe[1]);

    // Configure stdout pipe to non-blocking mode to prevent polling reads from hanging
    int flags = fcntl(out_pipe_fd_, F_GETFL, 0);
    if (flags >= 0)
    {
      fcntl(out_pipe_fd_, F_SETFL, flags | O_NONBLOCK);
    }

    line_buffer_.clear();

    // Perform UCI Handshake: initiate identification and readiness probe
    const char *init_cmds = "uci\nisready\n";
    if (!send_command(init_cmds))
    {
      std::cerr << "[StockfishDriver] Failed to send UCI handshake." << std::endl;
      stop_unlocked();
      return false;
    }

    // Await "readyok" token acknowledging engine initialization with 5-second deadline
    auto start_time = std::chrono::steady_clock::now();
    bool ready = false;
    while (std::chrono::duration_cast<std::chrono::seconds>(
               std::chrono::steady_clock::now() - start_time)
               .count() < 5)
    {
      std::string line = read_line(200);
      if (line == "readyok")
      {
        ready = true;
        break;
      }
    }

    if (!ready)
    {
      std::cerr << "[StockfishDriver] Handshake timed out waiting for readyok." << std::endl;
      stop_unlocked();
      return false;
    }

    return true;
  }

  void StockfishDriver::stop()
  {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    stop_unlocked();
  }

  void StockfishDriver::stop_unlocked()
  {
    if (engine_pid_ > 0)
    {
      // Gracefully signal engine shutdown over stdin pipe
      {
        std::lock_guard<std::mutex> stdin_lock(stdin_mutex_);
        if (in_pipe_fd_ >= 0)
        {
          const char *quit_cmd = "quit\n";
          (void)!write(in_pipe_fd_, quit_cmd, strlen(quit_cmd));
          close(in_pipe_fd_);
          in_pipe_fd_ = -1;
        }
      }
      if (out_pipe_fd_ >= 0)
      {
        close(out_pipe_fd_);
        out_pipe_fd_ = -1;
      }

      int status = 0;
      // Wait for child termination gracefully with timeout (up to 500ms)
      bool terminated = false;
      for (int i = 0; i < 25; ++i)
      {
        pid_t res = waitpid(engine_pid_, &status, WNOHANG);
        if (res == engine_pid_ || res < 0)
        {
          terminated = true;
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }

      // Hard kill if subprocess hangs on shutdown
      if (!terminated)
      {
        kill(engine_pid_, SIGKILL);
        waitpid(engine_pid_, &status, 0);
      }
      engine_pid_ = -1;
    }
  }

  bool StockfishDriver::is_running() const
  {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (engine_pid_ <= 0)
    {
      return false;
    }
    int status = 0;
    // Non-blocking query to inspect child termination status
    pid_t res = waitpid(engine_pid_, &status, WNOHANG);
    if (res == engine_pid_ || (res < 0 && errno == ECHILD))
    {
      engine_pid_ = -1;
      return false;
    }
    return (res == 0);
  }

  bool StockfishDriver::send_command(const std::string &cmd)
  {
    std::lock_guard<std::mutex> lock(stdin_mutex_);
    return send_command_unlocked(cmd);
  }

  bool StockfishDriver::send_command_unlocked(const std::string &cmd)
  {
    if (in_pipe_fd_ < 0)
    {
      return false;
    }
    ssize_t written = write(in_pipe_fd_, cmd.c_str(), cmd.size());
    return (written == static_cast<ssize_t>(cmd.size()));
  }

  void StockfishDriver::send_uci_stop()
  {
    std::lock_guard<std::mutex> lock(stdin_mutex_);
    send_command_unlocked("stop\n");
  }

  bool StockfishDriver::extract_buffered_line(std::string &buffer, std::string &out)
  {
    auto nl_pos = buffer.find('\n');
    if (nl_pos == std::string::npos)
    {
      return false;
    }
    out = buffer.substr(0, nl_pos);
    buffer.erase(0, nl_pos + 1);
    if (!out.empty() && out.back() == '\r')
    {
      out.pop_back();
    }
    return true;
  }

  std::string StockfishDriver::read_line(int timeout_ms)
  {
    if (out_pipe_fd_ < 0)
    {
      return "";
    }

    std::string line;
    // Inspect buffer for a newline already cached from a previous chunk read
    if (extract_buffered_line(line_buffer_, line))
    {
      return line;
    }

    // Poll child stdout descriptor for readable data until timeout expires
    struct pollfd pfd;
    pfd.fd = out_pipe_fd_;
    pfd.events = POLLIN;
    pfd.revents = 0;

    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (true)
    {
      auto now = std::chrono::steady_clock::now();
      if (now >= deadline)
      {
        break;
      }
      int remaining_ms = static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());

      int ret = poll(&pfd, 1, remaining_ms);
      if (ret > 0 && (pfd.revents & POLLIN))
      {
        char chunk[512];
        ssize_t n = read(out_pipe_fd_, chunk, sizeof(chunk) - 1);
        if (n > 0)
        {
          chunk[n] = '\0';
          line_buffer_.append(chunk, n);

          if (extract_buffered_line(line_buffer_, line))
          {
            return line;
          }
        }
        else if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK))
        {
          // EOF or pipe broken
          break;
        }
      }
      else
      {
        break;
      }
    }

    return "";
  }

  bool StockfishDriver::parse_info_line(const std::string &line, EngineInfoFeedback &feedback)
  {
    // UCI info lines always begin with the "info " token prefix
    if (line.rfind("info ", 0) != 0)
    {
      return false;
    }

    std::istringstream iss(line);
    std::string token;
    iss >> token; // Skip "info"

    while (iss >> token)
    {
      if (token == "depth")
      {
        iss >> feedback.depth;
      }
      else if (token == "nps")
      {
        iss >> feedback.nps;
      }
      else if (token == "score")
      {
        std::string score_type;
        iss >> score_type;
        if (score_type == "cp")
        {
          feedback.is_mate = false;
          iss >> feedback.score_cp;
        }
        else if (score_type == "mate")
        {
          feedback.is_mate = true;
          iss >> feedback.mate_in;
        }
      }
      else if (token == "pv")
      {
        // Principal variation occupies the remainder of the line
        std::string pv_tail;
        std::getline(iss, pv_tail);
        // Trim leading space if present
        if (!pv_tail.empty() && pv_tail.front() == ' ')
        {
          pv_tail.erase(0, 1);
        }
        feedback.pv = pv_tail;
        break; // pv is strictly the final token in UCI specification
      }
    }
    return true;
  }

  bool StockfishDriver::parse_bestmove_line(const std::string &line, BestMoveResult &result)
  {
    // UCI bestmove lines begin with "bestmove "
    if (line.rfind("bestmove ", 0) != 0)
    {
      return false;
    }

    std::istringstream iss(line);
    std::string tag, move, ponder_tag, ponder_move;
    iss >> tag >> move;

    // Detect terminal positions where engine cannot move (checkmate or stalemate)
    if (move.empty() || move == "(none)")
    {
      result.success = false;
      result.message = "No legal moves available (Checkmate or Stalemate).";
      return true;
    }

    result.success = true;
    result.best_move = move;

    // Optional opponent ponder move prediction
    if (iss >> ponder_tag && ponder_tag == "ponder" && iss >> ponder_move)
    {
      result.ponder = ponder_move;
    }
    return true;
  }

  BestMoveResult StockfishDriver::compute_best_move(
      const std::string &fen,
      uint32_t think_time_ms,
      uint32_t depth,
      std::function<void(const EngineInfoFeedback &)> on_feedback,
      std::function<bool()> is_canceled)
  {
    BestMoveResult result;
    result.success = false;

    {
      std::lock_guard<std::mutex> lock(lifecycle_mutex_);
      if (engine_pid_ <= 0 || in_pipe_fd_ < 0 || out_pipe_fd_ < 0)
      {
        result.message = "Stockfish engine is not running.";
        return result;
      }
      line_buffer_.clear();
    }

    // Instruct engine to configure internal board representation to target FEN
    if (!send_command("position fen " + fen + "\n"))
    {
      result.message = "Failed to write position command to Stockfish.";
      return result;
    }

    // Assemble search constraint command ("go movetime <ms> depth <d>")
    std::string go_cmd = "go";
    if (think_time_ms > 0)
    {
      go_cmd += " movetime " + std::to_string(think_time_ms);
    }
    if (depth > 0)
    {
      go_cmd += " depth " + std::to_string(depth);
    }
    go_cmd += "\n";

    if (!send_command(go_cmd))
    {
      result.message = "Failed to write go command to Stockfish.";
      return result;
    }

    auto start_time = std::chrono::steady_clock::now();
    // Allow engine think_time plus generous 10s buffer for process overhead before timing out
    int timeout_sec = (think_time_ms / 1000) + 10;
    bool stop_sent = false;
    EngineInfoFeedback latest_feedback;

    while (true)
    {
      // Check if external client requested goal cancellation
      if (is_canceled && is_canceled() && !stop_sent)
      {
        send_uci_stop();
        stop_sent = true;
      }

      auto elapsed_sec = std::chrono::duration_cast<std::chrono::seconds>(
                             std::chrono::steady_clock::now() - start_time)
                             .count();
      if (elapsed_sec > timeout_sec)
      {
        result.message = "Engine query timed out.";
        send_uci_stop();
        break;
      }

      std::string line = read_line(50);
      if (line.empty())
      {
        continue;
      }

      // Stream intermediate evaluation progression metrics to feedback callback
      if (line.rfind("info ", 0) == 0)
      {
        if (parse_info_line(line, latest_feedback))
        {
          if (on_feedback)
          {
            on_feedback(latest_feedback);
          }
        }
      }
      // "bestmove " denotes completion of search
      else if (line.rfind("bestmove ", 0) == 0)
      {
        parse_bestmove_line(line, result);
        result.score_cp = latest_feedback.score_cp;
        result.is_mate = latest_feedback.is_mate;
        result.mate_in = latest_feedback.mate_in;
        if (stop_sent)
        {
          result.success = false;
          result.message = "Search canceled by caller.";
        }
        break;
      }
    }

    return result;
  }

} // namespace lekiwi_chess_master
