/**
 * @file test_chess_domain.cpp
 * @brief Unit tests for pure C++ chess domain logic (classification, FIDE rules, kinematics).
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include <gtest/gtest.h>
#include <chess.hpp>
#include <vector>
#include "lekiwi_chess_master/chess_domain.hpp"

using namespace lekiwi_chess_master::domain;

class ChessDomainTest : public ::testing::Test
{
protected:
  void SetUp() override {}
};

TEST_F(ChessDomainTest, ClassifiesAllLegalMoveCategories)
{
  struct TestCase
  {
    std::string fen;
    std::string uci;
    std::string expected_piece_type;
    std::string expected_color;
    std::string expected_promo;
    bool expected_capture;
    bool expected_ep;
    bool expected_castling;
  };

  const std::vector<TestCase> cases = {
    // 1. Quiet pawn
    {"rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1", "e2e4", "pawn", "white", "", false, false, false},
    // 2. Quiet knight
    {"rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1", "g1f3", "knight", "white", "", false, false, false},
    // 3. Standard capture (pawn captures pawn)
    {"rnbqkbnr/ppp1pppp/8/3p4/4P3/8/PPPP1PPP/RNBQKBNR w KQkq d6 0 2", "e4d5", "pawn:pawn", "white", "", true, false, false},
    // 4. En-passant capture
    {"rnbqkbnr/ppp1p1pp/8/3pPp2/8/8/PPPP1PPP/RNBQKBNR w KQkq f6 0 3", "e5f6", "pawn:pawn", "white", "", true, true, false},
    // 5. Kingside castling
    {"r1bqk2r/pppp1ppp/2n2n2/2b1p3/2B1P3/5N2/PPPP1PPP/RNBQK2R w KQkq - 4 4", "e1g1", "king", "white", "", false, false, true},
    // 6. Pawn promotion to queen
    {"8/4P3/8/8/8/8/k7/4K3 w - - 0 1", "e7e8q", "pawn", "white", "queen", false, false, false},
    // 7. Black piece quiet move
    {"rnbqkbnr/pppp1ppp/8/4p3/4P3/8/PPPP1PPP/RNBQKBNR b KQkq - 0 1", "g8f6", "knight", "black", "", false, false, false}
  };

  for (const auto &tc : cases)
  {
    chess::Board board(tc.fen);
    auto details = classify_move(board, tc.uci);
    EXPECT_EQ(details.uci, tc.uci) << "Failed on " << tc.uci;
    EXPECT_EQ(details.piece_type, tc.expected_piece_type) << "Failed on " << tc.uci;
    EXPECT_EQ(details.piece_color, tc.expected_color) << "Failed on " << tc.uci;
    EXPECT_EQ(details.promotion_piece, tc.expected_promo) << "Failed on " << tc.uci;
    EXPECT_EQ(details.is_capture, tc.expected_capture) << "Failed on " << tc.uci;
    EXPECT_EQ(details.is_en_passant, tc.expected_ep) << "Failed on " << tc.uci;
    EXPECT_EQ(details.is_castling, tc.expected_castling) << "Failed on " << tc.uci;
  }
}

TEST_F(ChessDomainTest, HandlesMalformedAndPseudoLegalMoves)
{
  chess::Board board;
  EXPECT_TRUE(classify_move(board, "(none)").uci.empty());
  EXPECT_TRUE(classify_move(board, "").uci.empty());
  EXPECT_TRUE(classify_move(board, "xxxx").uci.empty());

  auto pseudo = classify_move(board, "h7h8q");
  EXPECT_EQ(pseudo.uci, "h7h8q");
  EXPECT_EQ(pseudo.promotion_piece, "queen");
}

TEST_F(ChessDomainTest, ValidateFenStandardPositions)
{
  chess::Board board;
  std::string err;

  // 1. Full 6-field standard starting FEN
  EXPECT_TRUE(validate_and_parse_fen("rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1", board, err));
  EXPECT_TRUE(err.empty());
  EXPECT_EQ(board.getFen(), "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1");

  // 2. 1-field piece placement only
  EXPECT_TRUE(validate_and_parse_fen("rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR", board, err));
  EXPECT_TRUE(err.empty());
  EXPECT_EQ(board.sideToMove(), chess::Color::WHITE);

  // 3. Valid custom endgame position
  EXPECT_TRUE(validate_and_parse_fen("8/8/8/4k3/8/8/4K3/8 w - - 0 1", board, err));
  EXPECT_TRUE(err.empty());
}

TEST_F(ChessDomainTest, ValidateFenRejectsInvalidInput)
{
  chess::Board board;
  std::string err;

  // Empty or whitespace
  EXPECT_FALSE(validate_and_parse_fen("", board, err));
  EXPECT_FALSE(err.empty());

  EXPECT_FALSE(validate_and_parse_fen("   ", board, err));

  // Invalid rank counts (7 ranks instead of 8)
  EXPECT_FALSE(validate_and_parse_fen("rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP w - - 0 1", board, err));
  EXPECT_NE(err.find("8 ranks"), std::string::npos);

  // Invalid square count in rank (sum = 9)
  EXPECT_FALSE(validate_and_parse_fen("rnbqkbnr/ppppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w - - 0 1", board, err));
  EXPECT_NE(err.find("does not sum to 8"), std::string::npos);

  // Invalid character
  EXPECT_FALSE(validate_and_parse_fen("rnbqkbnr/pppppppp/8/8/4X3/8/PPPPPPPP/RNBQKBNR w - - 0 1", board, err));
  EXPECT_NE(err.find("Invalid character"), std::string::npos);

  // Missing kings (0 kings)
  EXPECT_FALSE(validate_and_parse_fen("8/8/8/8/8/8/8/8 w - - 0 1", board, err));
  EXPECT_NE(err.find("king"), std::string::npos);

  // Missing Black king (White king only)
  EXPECT_FALSE(validate_and_parse_fen("8/8/8/4K3/8/8/8/8 w - - 0 1", board, err));
  EXPECT_NE(err.find("Black king"), std::string::npos);

  // Two White kings
  EXPECT_FALSE(validate_and_parse_fen("8/8/8/4K2K/4k3/8/8/8 w - - 0 1", board, err));
  EXPECT_NE(err.find("White king"), std::string::npos);

  // Pawns on 8th rank
  EXPECT_FALSE(validate_and_parse_fen("4P3/8/8/4k3/8/8/4K3/8 w - - 0 1", board, err));
  EXPECT_NE(err.find("rank"), std::string::npos);

  // Inactive king under check (Black king attacked by White Queen, but it is Black's turn to move)
  EXPECT_FALSE(validate_and_parse_fen("4k3/8/8/8/8/8/4r3/4K3 b - - 0 1", board, err));
  EXPECT_NE(err.find("Opposing king"), std::string::npos);
}
