/**
 * @file test_chess_vision_mapper.cpp
 * @brief Unit tests for ChessVisionMapper and ChessGameStateTracker.
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include <gtest/gtest.h>
#include <vector>

#include "hailo/chess_vision_mapper.hpp"
#include "hailo/chess_game_state_tracker.hpp"

using namespace lekiwi_perception::hailo;

namespace
{

// Generate synthetic 81 grid points in normalized coordinates [0.1, 0.9]
// TL is at index 0 (0.1, 0.1)
// TR is at index 8 (0.9, 0.1)
// BR is at index 80 (0.9, 0.9)
// BL is at index 72 (0.1, 0.9)
std::vector<cv::Point2f> create_canonical_test_grid()
{
  std::vector<cv::Point2f> grid;
  grid.reserve(81);
  for (int r = 0; r <= 8; ++r)
  {
    for (int c = 0; c <= 8; ++c)
    {
      float x = 0.1f + 0.8f * (static_cast<float>(c) / 8.0f);
      float y = 0.1f + 0.8f * (static_cast<float>(r) / 8.0f);
      grid.emplace_back(x, y);
    }
  }
  return grid;
}

} // namespace

TEST(ChessVisionMapperTest, MatchA1CornerStandardWhite)
{
  const auto grid = create_canonical_test_grid();

  // White perspective:
  // A1 is at Bottom-Left (BL, corner 3)
  // Tag 1 (A1): at BL (0.1, 0.9)
  // Tag 4 (H1): at BR (0.9, 0.9)
  // Tag 3 (H8): at TR (0.9, 0.1)
  // Tag 6 (A8): at TL (0.1, 0.1)
  std::vector<Tag2D> tags = {
      {1, cv::Point2f(0.11f, 0.89f)},
      {4, cv::Point2f(0.89f, 0.91f)},
      {3, cv::Point2f(0.91f, 0.11f)},
      {6, cv::Point2f(0.09f, 0.11f)}};

  int a1_idx = ChessVisionMapper::match_a1_corner_index(grid, tags, 0);
  EXPECT_EQ(a1_idx, 3); // BL
}

TEST(ChessVisionMapperTest, MatchA1CornerRotated180Black)
{
  const auto grid = create_canonical_test_grid();

  // Black perspective (180 deg rotation):
  // A1 is at Top-Right (TR, corner 1)
  // Tag 1 (A1): at TR (0.9, 0.1)
  // Tag 6 (A8): at BR (0.9, 0.9)
  // Tag 3 (H8): at BL (0.1, 0.9)
  // Tag 4 (H1): at TL (0.1, 0.1)
  std::vector<Tag2D> tags = {
      {1, cv::Point2f(0.89f, 0.11f)},
      {6, cv::Point2f(0.91f, 0.89f)},
      {3, cv::Point2f(0.11f, 0.91f)},
      {4, cv::Point2f(0.11f, 0.09f)}};

  int a1_idx = ChessVisionMapper::match_a1_corner_index(grid, tags, 0);
  EXPECT_EQ(a1_idx, 1); // TR
}

TEST(ChessVisionMapperTest, MatchA1CornerRotated90CCW)
{
  const auto grid = create_canonical_test_grid();

  // 90 deg CCW: A1 at Top-Left (TL, corner 0)
  // Tag 1 (A1): at TL (0.1, 0.1)
  // Tag 6 (A8): at TR (0.9, 0.1)
  // Tag 3 (H8): at BR (0.9, 0.9)
  // Tag 4 (H1): at BL (0.1, 0.9)
  std::vector<Tag2D> tags = {
      {1, cv::Point2f(0.10f, 0.10f)},
      {6, cv::Point2f(0.90f, 0.10f)},
      {3, cv::Point2f(0.90f, 0.90f)},
      {4, cv::Point2f(0.10f, 0.90f)}};

  int a1_idx = ChessVisionMapper::match_a1_corner_index(grid, tags, 0);
  EXPECT_EQ(a1_idx, 0); // TL
}

TEST(ChessVisionMapperTest, MatchA1CornerRotated90CW)
{
  const auto grid = create_canonical_test_grid();

  // 90 deg CW: A1 at Bottom-Right (BR, corner 2)
  // Tag 1 (A1): at BR (0.9, 0.9)
  // Tag 6 (A8): at BL (0.1, 0.9)
  // Tag 3 (H8): at TL (0.1, 0.1)
  // Tag 4 (H1): at TR (0.9, 0.1)
  std::vector<Tag2D> tags = {
      {1, cv::Point2f(0.90f, 0.90f)},
      {6, cv::Point2f(0.10f, 0.90f)},
      {3, cv::Point2f(0.10f, 0.10f)},
      {4, cv::Point2f(0.90f, 0.10f)}};

  int a1_idx = ChessVisionMapper::match_a1_corner_index(grid, tags, 0);
  EXPECT_EQ(a1_idx, 2); // BR
}

TEST(ChessVisionMapperTest, MatchA1CornerSingleTagInference)
{
  const auto grid = create_canonical_test_grid();

  // Only Tag 4 (H1) detected at BR (0.9, 0.9).
  // Tag 4 is H1, which is at BR in White perspective -> A1 is at BL (3).
  std::vector<Tag2D> single_tag_4 = {
      {4, cv::Point2f(0.89f, 0.91f)}};
  EXPECT_EQ(ChessVisionMapper::match_a1_corner_index(grid, single_tag_4, 0), 3);

  // Only Tag 6 (A8) detected at TL (0.1, 0.1).
  // Tag 6 is A8, which is at TL in White perspective -> A1 is at BL (3).
  std::vector<Tag2D> single_tag_6 = {
      {6, cv::Point2f(0.11f, 0.11f)}};
  EXPECT_EQ(ChessVisionMapper::match_a1_corner_index(grid, single_tag_6, 0), 3);
}

TEST(ChessVisionMapperTest, MatchA1CornerFallbackWhenEmpty)
{
  const auto grid = create_canonical_test_grid();
  std::vector<Tag2D> empty_tags;
  EXPECT_EQ(ChessVisionMapper::match_a1_corner_index(grid, empty_tags, 2), 2);
}

TEST(ChessVisionMapperTest, GeneratePiecePlacementPure)
{
  std::map<std::string, std::string> occupancy;
  // Starting position piece placement
  occupancy["a8"] = "r"; occupancy["b8"] = "n"; occupancy["c8"] = "b"; occupancy["d8"] = "q";
  occupancy["e8"] = "k"; occupancy["f8"] = "b"; occupancy["g8"] = "n"; occupancy["h8"] = "r";
  for (char f : {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h'}) {
    occupancy[std::string(1, f) + "7"] = "p";
    occupancy[std::string(1, f) + "2"] = "P";
  }
  occupancy["a1"] = "R"; occupancy["b1"] = "N"; occupancy["c1"] = "B"; occupancy["d1"] = "Q";
  occupancy["e1"] = "K"; occupancy["f1"] = "B"; occupancy["g1"] = "N"; occupancy["h1"] = "R";

  std::string fen = ChessVisionMapper::generate_fen(occupancy);
  EXPECT_EQ(fen, "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR");
}

TEST(ChessGameStateTrackerTest, ValidMoveDebounceAndFullFen)
{
  ChessGameStateTracker tracker(2); // Debounce window = 2 frames

  std::string white_e4 = "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR"; // White e4

  // Frame 1: unstable debounce
  GameStateResult res1 = tracker.update(white_e4);
  EXPECT_FALSE(res1.is_board_stable);
  EXPECT_FALSE(res1.is_legal_move);

  // Frame 2: stable debounce -> transition accepted
  GameStateResult res2 = tracker.update(white_e4);
  EXPECT_TRUE(res2.is_board_stable);
  EXPECT_TRUE(res2.is_legal_move);
  EXPECT_EQ(res2.last_move, "e2e4");
  EXPECT_EQ(res2.full_fen, "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq - 0 1");
}

TEST(ChessGameStateTrackerTest, RejectIllegalMove)
{
  ChessGameStateTracker tracker(1); // Debounce window = 1 frame

  // White king suddenly leaps to e4 (illegal opening)
  std::string illegal_king = "rnbqkbnr/pppppppp/8/8/4K3/8/PPPPPPPP/RNBQ1BNR";

  GameStateResult res = tracker.update(illegal_king);
  EXPECT_TRUE(res.is_board_stable);
  EXPECT_FALSE(res.is_legal_move);
  // Full FEN remains at starting position
  EXPECT_EQ(res.full_fen, "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1");
}
