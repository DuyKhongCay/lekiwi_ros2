/**
 * @file test_stockfish_driver.cpp
 * @brief Unit tests for StockfishDriver UCI communication and line parsers.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include <gtest/gtest.h>
#include "lekiwi_chess_master/stockfish_driver.hpp"

using namespace lekiwi_chess_master;

TEST(StockfishDriverTest, ParseInfoLineMetrics)
{
  // 1. Centipawns score & NPS
  std::string info_cp = "info depth 12 seldepth 18 multipv 1 score cp 45 nodes 152340 nps 1200000 tbhits 0 time 127 pv e2e4 e7e5 g1f3";
  EngineInfoFeedback fb_cp;
  EXPECT_TRUE(StockfishDriver::parse_info_line(info_cp, fb_cp));
  EXPECT_EQ(fb_cp.depth, 12U);
  EXPECT_FALSE(fb_cp.is_mate);
  EXPECT_EQ(fb_cp.score_cp, 45);
  EXPECT_EQ(fb_cp.nps, 1200000U);
  EXPECT_EQ(fb_cp.pv, "e2e4 e7e5 g1f3");

  // 2. Forced mate
  std::string info_mate = "info depth 5 score mate 2 pv d1h5 g7g6 h5e5";
  EngineInfoFeedback fb_mate;
  EXPECT_TRUE(StockfishDriver::parse_info_line(info_mate, fb_mate));
  EXPECT_EQ(fb_mate.depth, 5U);
  EXPECT_TRUE(fb_mate.is_mate);
  EXPECT_EQ(fb_mate.mate_in, 2);
  EXPECT_EQ(fb_mate.pv, "d1h5 g7g6 h5e5");

  // 3. Non-info line rejected
  EngineInfoFeedback fb_invalid;
  EXPECT_FALSE(StockfishDriver::parse_info_line("uciok", fb_invalid));
}

TEST(StockfishDriverTest, ParseBestMoveResponses)
{
  // 1. Bestmove with ponder
  std::string bm_ponder = "bestmove e2e4 ponder e7e5";
  BestMoveResult res_ponder;
  EXPECT_TRUE(StockfishDriver::parse_bestmove_line(bm_ponder, res_ponder));
  EXPECT_TRUE(res_ponder.success);
  EXPECT_EQ(res_ponder.best_move, "e2e4");
  EXPECT_EQ(res_ponder.ponder, "e7e5");

  // 2. Bestmove (none) terminal/no-moves
  std::string bm_none = "bestmove (none)";
  BestMoveResult res_none;
  EXPECT_TRUE(StockfishDriver::parse_bestmove_line(bm_none, res_none));
  EXPECT_FALSE(res_none.success);
}

TEST(StockfishDriverTest, SubprocessLifecycle)
{
  StockfishDriver driver;
  bool started = driver.start("/usr/games/stockfish");
  if (started)
  {
    EXPECT_TRUE(driver.is_running());
    BestMoveResult res = driver.compute_best_move(
        "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1", 200);
    EXPECT_TRUE(res.success);
    EXPECT_FALSE(res.best_move.empty());
    driver.stop();
    EXPECT_FALSE(driver.is_running());
  }
}

TEST(StockfishDriverTest, SubprocessErrorHandlingAndCancellation)
{
  StockfishDriver driver;

  // 1. Reject invalid binary
  EXPECT_FALSE(driver.start("/nonexistent/stockfish_binary"));
  EXPECT_FALSE(driver.is_running());

  // 2. Reject search on unstarted engine
  BestMoveResult res_unstarted = driver.compute_best_move(
      "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1", 100);
  EXPECT_FALSE(res_unstarted.success);

  // 3. Cancellation during search
  if (driver.start("/usr/games/stockfish"))
  {
    EXPECT_TRUE(driver.is_running());
    int cancel_poll_count = 0;
    auto is_canceled = [&cancel_poll_count]() -> bool
    {
      cancel_poll_count++;
      return cancel_poll_count >= 2;
    };

    BestMoveResult res_cancel = driver.compute_best_move(
        "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1",
        5000, 0, nullptr, is_canceled);

    EXPECT_FALSE(res_cancel.success);
    driver.stop();
    EXPECT_FALSE(driver.is_running());
  }
}
