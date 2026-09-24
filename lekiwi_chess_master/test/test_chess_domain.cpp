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
  EXPECT_EQ(details.san, "e4");
  EXPECT_EQ(details.from_square, "e2");
  EXPECT_EQ(details.to_square, "e4");
  EXPECT_EQ(details.piece_type, "pawn");
  EXPECT_EQ(details.piece_color, "w");
  EXPECT_TRUE(details.promotion_piece.empty());
  EXPECT_FALSE(details.is_capture);
  EXPECT_TRUE(details.captured_square.empty());
  EXPECT_FALSE(details.is_en_passant);
  EXPECT_FALSE(details.is_castling);
}

TEST_F(ChessDomainTest, ClassifiesKnightMove)
{
  chess::Board board;
  auto details = classify_move(board, "g1f3");

  EXPECT_EQ(details.uci, "g1f3");
  EXPECT_EQ(details.san, "Nf3");
  EXPECT_EQ(details.from_square, "g1");
  EXPECT_EQ(details.to_square, "f3");
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
  EXPECT_EQ(details.san, "exd5");
  EXPECT_EQ(details.from_square, "e4");
  EXPECT_EQ(details.to_square, "d5");
  EXPECT_EQ(details.piece_type, "pawn");
  EXPECT_EQ(details.piece_color, "w");
  EXPECT_TRUE(details.is_capture);
  EXPECT_EQ(details.captured_piece_type, "pawn");
  EXPECT_EQ(details.captured_square, "d5");
  EXPECT_FALSE(details.is_en_passant);
  EXPECT_FALSE(details.is_castling);
}

TEST_F(ChessDomainTest, ClassifiesEnPassantCapture)
{
  // Position with en-passant target on f6
  chess::Board board("rnbqkbnr/ppp1p1pp/8/3pPp2/8/8/PPPP1PPP/RNBQKBNR w KQkq f6 0 3");
  auto details = classify_move(board, "e5f6");

  EXPECT_EQ(details.uci, "e5f6");
  EXPECT_EQ(details.san, "exf6");
  EXPECT_EQ(details.from_square, "e5");
  EXPECT_EQ(details.to_square, "f6");
  EXPECT_EQ(details.piece_type, "pawn");
  EXPECT_EQ(details.piece_color, "w");
  EXPECT_TRUE(details.is_capture);
  EXPECT_TRUE(details.is_en_passant);
  EXPECT_EQ(details.captured_piece_type, "pawn");
  // Pawn captured en-passant is at f5, NOT f6!
  EXPECT_EQ(details.captured_square, "f5");
  EXPECT_FALSE(details.is_castling);
}

TEST_F(ChessDomainTest, ClassifiesKingsideCastling)
{
  // Position where White can castle kingside
  chess::Board board("r1bqk2r/pppp1ppp/2n2n2/2b1p3/2B1P3/5N2/PPPP1PPP/RNBQK2R w KQkq - 4 4");
  auto details = classify_move(board, "e1g1");

  EXPECT_EQ(details.uci, "e1g1");
  EXPECT_EQ(details.san, "O-O");
  EXPECT_EQ(details.from_square, "e1");
  EXPECT_EQ(details.to_square, "g1");
  EXPECT_EQ(details.piece_type, "king");
  EXPECT_EQ(details.piece_color, "w");
  EXPECT_FALSE(details.is_capture);
  EXPECT_FALSE(details.is_en_passant);
  EXPECT_TRUE(details.is_castling);
  // King-side castling: rook moves from h1 to f1
  EXPECT_EQ(details.castling_rook_from, "h1");
  EXPECT_EQ(details.castling_rook_to, "f1");
}

TEST_F(ChessDomainTest, ClassifiesPawnPromotion)
{
  // White pawn on e7 promoting on e8
  chess::Board board("8/4P3/8/8/8/8/k7/4K3 w - - 0 1");
  auto details = classify_move(board, "e7e8q");

  EXPECT_EQ(details.uci, "e7e8q");
  EXPECT_EQ(details.san, "e8=Q");
  EXPECT_EQ(details.from_square, "e7");
  EXPECT_EQ(details.to_square, "e8");
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
  EXPECT_TRUE(none_details.from_square.empty());
  EXPECT_TRUE(none_details.to_square.empty());

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
  EXPECT_EQ(details.from_square, "h7");
  EXPECT_EQ(details.to_square, "h8");
  EXPECT_EQ(details.promotion_piece, "queen");
}
