// Copyright 2026 LeKiwi Labs
// Licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>
#include <cmath>
#include <stdexcept>

#include "lekiwi_motion/chessboard_mapper.hpp"

namespace lekiwi_motion::test
{

  TEST(ChessboardMapperTest, InitializationExplicit)
  {
    constexpr double kBoardW = 0.380;
    constexpr double kBoardH = 0.380;
    constexpr double kGraspZ = 0.025;
    ChessboardMapper mapper(kBoardW, kBoardH, kGraspZ, true);
    EXPECT_NEAR(mapper.board_width(), kBoardW, 1e-6);
    EXPECT_NEAR(mapper.board_height(), kBoardH, 1e-6);
    EXPECT_NEAR(mapper.square_size_x(), kBoardW / 8.0, 1e-6);
    EXPECT_NEAR(mapper.square_size_y(), kBoardH / 8.0, 1e-6);
    EXPECT_NEAR(mapper.grasp_z(), kGraspZ, 1e-6);
    EXPECT_TRUE(mapper.origin_at_center());
  }

  TEST(ChessboardMapperTest, InvalidDimensions)
  {
    EXPECT_THROW(ChessboardMapper(-0.1, 0.2, 0.025), std::invalid_argument);
    EXPECT_THROW(ChessboardMapper(0.2, 0.0, 0.025), std::invalid_argument);
    EXPECT_THROW(ChessboardMapper(0.0, -0.5, 0.025), std::invalid_argument);
  }

  TEST(ChessboardMapperTest, SquareSymmetryAroundCenter)
  {
    // 0.20m width => each square is 0.025m (25mm)
    constexpr double kBoardW = 0.20;
    constexpr double kBoardH = 0.20;
    constexpr double kGraspZ = 0.025;
    constexpr double kSqSize = 0.025;
    ChessboardMapper mapper(kBoardW, kBoardH, kGraspZ, true);

    // A1 (col 0, row 0): center is at (-3.5 * 0.025, -3.5 * 0.025)
    auto a1 = mapper.square_to_metric("a1");
    EXPECT_EQ(a1.square_name, "a1");
    EXPECT_EQ(a1.file_char, 'a');
    EXPECT_EQ(a1.rank_char, '1');
    EXPECT_EQ(a1.file_idx, 0);
    EXPECT_EQ(a1.rank_idx, 0);
    EXPECT_NEAR(a1.x, -3.5 * kSqSize, 1e-6);
    EXPECT_NEAR(a1.y, -3.5 * kSqSize, 1e-6);
    EXPECT_NEAR(a1.z, kGraspZ, 1e-6);

    // H8 (col 7, row 7): center is at (+3.5 * 0.025, +3.5 * 0.025)
    auto h8 = mapper.square_to_metric(std::string_view("h8"));
    EXPECT_EQ(h8.file_idx, 7);
    EXPECT_EQ(h8.rank_idx, 7);
    EXPECT_NEAR(h8.x, 3.5 * kSqSize, 1e-6);
    EXPECT_NEAR(h8.y, 3.5 * kSqSize, 1e-6);

    // D4 (col 3, row 3): center is (-0.5 * 0.025, -0.5 * 0.025)
    std::string d4_str = "d4";
    auto d4 = mapper.square_to_metric(d4_str);
    EXPECT_NEAR(d4.x, -0.5 * kSqSize, 1e-6);
    EXPECT_NEAR(d4.y, -0.5 * kSqSize, 1e-6);

    // E5 (col 4, row 4): center is (+0.5 * 0.025, +0.5 * 0.025)
    auto e5 = mapper.square_to_metric("e5");
    EXPECT_NEAR(e5.x, 0.5 * kSqSize, 1e-6);
    EXPECT_NEAR(e5.y, 0.5 * kSqSize, 1e-6);

    // Point message conversion
    auto pt = e5.to_point_msg();
    EXPECT_NEAR(pt.x, e5.x, 1e-6);
    EXPECT_NEAR(pt.y, e5.y, 1e-6);
    EXPECT_NEAR(pt.z, e5.z, 1e-6);
  }

  TEST(ChessboardMapperTest, InvalidSquareNotations)
  {
    ChessboardMapper mapper(0.380, 0.380, 0.025);
    EXPECT_THROW(mapper.square_to_metric("e"), std::invalid_argument);
    EXPECT_THROW(mapper.square_to_metric("i4"), std::invalid_argument);
    EXPECT_THROW(mapper.square_to_metric("e9"), std::invalid_argument);
    EXPECT_THROW(mapper.square_to_metric("a0"), std::invalid_argument);
    EXPECT_THROW(mapper.square_to_metric(""), std::invalid_argument);
  }

  TEST(ChessboardMapperTest, ParseUciMoveStandard)
  {
    constexpr double kBoardW = 0.20;
    constexpr double kBoardH = 0.20;
    ChessboardMapper mapper(kBoardW, kBoardH, 0.025);

    auto move = mapper.parse_uci_move("e2e4");
    EXPECT_EQ(move.uci, "e2e4");
    EXPECT_EQ(move.from_square, "e2");
    EXPECT_EQ(move.to_square, "e4");
    EXPECT_TRUE(move.promotion.empty());
    EXPECT_FALSE(move.is_capture);

    auto e2 = mapper.square_to_metric("e2");
    auto e4 = mapper.square_to_metric("e4");
    EXPECT_NEAR(move.pick_point().x, e2.x, 1e-6);
    EXPECT_NEAR(move.pick_point().y, e2.y, 1e-6);
    EXPECT_NEAR(move.place_point().x, e4.x, 1e-6);
    EXPECT_NEAR(move.place_point().y, e4.y, 1e-6);
  }

  TEST(ChessboardMapperTest, ParseUciMovePromotionAndCapture)
  {
    ChessboardMapper mapper(0.380, 0.380, 0.025);
    auto move = mapper.parse_uci_move(std::string_view("e7e8q"), true);
    EXPECT_EQ(move.uci, "e7e8q");
    EXPECT_EQ(move.from_square, "e7");
    EXPECT_EQ(move.to_square, "e8");
    EXPECT_EQ(move.promotion, "q");
    EXPECT_TRUE(move.is_capture);
  }

  TEST(ChessboardMapperTest, ParseUciMoveInvalid)
  {
    ChessboardMapper mapper(0.380, 0.380, 0.025);
    EXPECT_THROW(mapper.parse_uci_move("e2e9"), std::invalid_argument);
    EXPECT_THROW(mapper.parse_uci_move("e2"), std::invalid_argument);
    EXPECT_THROW(mapper.parse_uci_move("e2e4k"), std::invalid_argument); // 'k' is not valid promotion
    EXPECT_THROW(mapper.parse_uci_move(""), std::invalid_argument);
  }

} // namespace lekiwi_motion::test
