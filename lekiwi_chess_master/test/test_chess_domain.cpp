/**
 * @file test_chess_domain.cpp
 * @brief Unit tests for pure C++ chess domain logic (classification, FIDE rules, kinematics).
 *
 * @author DuyKhongCay
 * @copyright Apache-2.0
 */

#include <gtest/gtest.h>
#include <chess.hpp>
#include "lekiwi_chess_master/chess_domain.hpp"

using namespace lekiwi_chess_master::domain;

class ChessDomainTest : public ::testing::Test
{
protected:
  void SetUp() override {}
};

TEST_F(ChessDomainTest, ClassifiesQuietPawnMove)
{
  chess::Board board; // Standard starting position
  auto details = classify_move(board, "e2e4");

  EXPECT_EQ(details.uci, "e2e4");
  EXPECT_EQ(details.piece_type, "pawn");
  EXPECT_EQ(details.piece_color, "w");
  EXPECT_TRUE(details.promotion_piece.empty());
  EXPECT_FALSE(details.is_capture);
  EXPECT_FALSE(details.is_en_passant);
  EXPECT_FALSE(details.is_castling);
}

TEST_F(ChessDomainTest, ClassifiesKnightMove)
{
  chess::Board board;
  auto details = classify_move(board, "g1f3");

  EXPECT_EQ(details.uci, "g1f3");
  EXPECT_EQ(details.piece_type, "knight");
  EXPECT_EQ(details.piece_color, "w");
  EXPECT_FALSE(details.is_capture);
  EXPECT_FALSE(details.is_en_passant);
  EXPECT_FALSE(details.is_castling);
}

TEST_F(ChessDomainTest, ClassifiesStandardCapture)
{
  // Position where White pawn on e4 can capture Black pawn on d5
  chess::Board board("rnbqkbnr/ppp1pppp/8/3p4/4P3/8/PPPP1PPP/RNBQKBNR w KQkq d6 0 2");
  auto details = classify_move(board, "e4d5");

  EXPECT_EQ(details.uci, "e4d5");
  EXPECT_EQ(details.piece_type, "pawn:pawn");
  EXPECT_EQ(details.piece_color, "w");
  EXPECT_TRUE(details.is_capture);
  EXPECT_FALSE(details.is_en_passant);
  EXPECT_FALSE(details.is_castling);
}

TEST_F(ChessDomainTest, ClassifiesEnPassantCapture)
{
  // Position with en-passant target on f6
  chess::Board board("rnbqkbnr/ppp1p1pp/8/3pPp2/8/8/PPPP1PPP/RNBQKBNR w KQkq f6 0 3");
  auto details = classify_move(board, "e5f6");

  EXPECT_EQ(details.uci, "e5f6");
  EXPECT_EQ(details.piece_type, "pawn:pawn");
  EXPECT_EQ(details.piece_color, "w");
  EXPECT_TRUE(details.is_capture);
  EXPECT_TRUE(details.is_en_passant);
  EXPECT_FALSE(details.is_castling);
}

TEST_F(ChessDomainTest, ClassifiesKingsideCastling)
{
  // Position where White can castle kingside
  chess::Board board("r1bqk2r/pppp1ppp/2n2n2/2b1p3/2B1P3/5N2/PPPP1PPP/RNBQK2R w KQkq - 4 4");
  auto details = classify_move(board, "e1g1");

  EXPECT_EQ(details.uci, "e1g1");
  EXPECT_EQ(details.piece_type, "king");
  EXPECT_EQ(details.piece_color, "w");
  EXPECT_FALSE(details.is_capture);
  EXPECT_FALSE(details.is_en_passant);
  EXPECT_TRUE(details.is_castling);
}

TEST_F(ChessDomainTest, ClassifiesPawnPromotion)
{
  // White pawn on e7 promoting on e8
  chess::Board board("8/4P3/8/8/8/8/k7/4K3 w - - 0 1");
  auto details = classify_move(board, "e7e8q");

  EXPECT_EQ(details.uci, "e7e8q");
  EXPECT_EQ(details.piece_type, "pawn");
  EXPECT_EQ(details.piece_color, "w");
  EXPECT_EQ(details.promotion_piece, "queen");
  EXPECT_FALSE(details.is_capture);
  EXPECT_FALSE(details.is_en_passant);
  EXPECT_FALSE(details.is_castling);
}

TEST_F(ChessDomainTest, GuardsAgainstNoneAndGarbage)
{
  chess::Board board;
  // Guard against Stockfish (none)
  auto none_details = classify_move(board, "(none)");
  EXPECT_TRUE(none_details.uci.empty());

  // Guard against empty
  auto empty_details = classify_move(board, "");
  EXPECT_TRUE(empty_details.uci.empty());

  // Guard against invalid characters
  auto garbage_details = classify_move(board, "xxxx");
  EXPECT_TRUE(garbage_details.uci.empty());
}

TEST_F(ChessDomainTest, FallbackOnPseudoLegalMove)
{
  chess::Board board;
  auto details = classify_move(board, "h7h8q");

  EXPECT_EQ(details.uci, "h7h8q");
  EXPECT_EQ(details.promotion_piece, "queen");
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
  // White to move, but White king is already attacked by Black rook
  EXPECT_FALSE(validate_and_parse_fen("4k3/8/8/8/8/8/4r3/4K3 b - - 0 1", board, err));
  EXPECT_NE(err.find("Opposing king"), std::string::npos);
}
