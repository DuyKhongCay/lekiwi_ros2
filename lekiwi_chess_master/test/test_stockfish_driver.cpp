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

TEST(StockfishDriverTest, ParseInfoLineCentipawns)
{
  std::string info_str = "info depth 12 seldepth 18 multipv 1 score cp 45 nodes 152340 nps 1200000 tbhits 0 time 127 pv e2e4 e7e5 g1f3";
  EngineInfoFeedback feedback;
  EXPECT_TRUE(StockfishDriver::parse_info_line(info_str, feedback));
  EXPECT_EQ(feedback.depth, 12U);
  EXPECT_FALSE(feedback.is_mate);
  EXPECT_EQ(feedback.score_cp, 45);
  EXPECT_EQ(feedback.nps, 1200000U);
  EXPECT_EQ(feedback.pv, "e2e4 e7e5 g1f3");
}

TEST(StockfishDriverTest, ParseInfoLineMate)
{
  std::string info_str = "info depth 5 score mate 2 pv d1h5 g7g6 h5e5";
  EngineInfoFeedback feedback;
  EXPECT_TRUE(StockfishDriver::parse_info_line(info_str, feedback));
  EXPECT_EQ(feedback.depth, 5U);
  EXPECT_TRUE(feedback.is_mate);
  EXPECT_EQ(feedback.mate_in, 2);
  EXPECT_EQ(feedback.pv, "d1h5 g7g6 h5e5");
}

TEST(StockfishDriverTest, ParseBestMoveLineWithPonder)
{
  std::string bm_str = "bestmove e2e4 ponder e7e5";
  BestMoveResult res;
  EXPECT_TRUE(StockfishDriver::parse_bestmove_line(bm_str, res));
  EXPECT_TRUE(res.success);
  EXPECT_EQ(res.best_move, "e2e4");
  EXPECT_EQ(res.ponder, "e7e5");
}

TEST(StockfishDriverTest, ParseBestMoveNone)
{
  std::string bm_str = "bestmove (none)";
  BestMoveResult res;
  EXPECT_TRUE(StockfishDriver::parse_bestmove_line(bm_str, res));
  EXPECT_FALSE(res.success);
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
