#include <catch2/catch_test_macros.hpp>

#include <igb_util/bitmagic.hpp>

TEST_CASE("bit_count_u8", "[bits]") {
  REQUIRE(igb::bit_count_u8(0b11101011) == 6);
}

TEST_CASE("bit_count_u16", "[bits]") {
  REQUIRE(igb::bit_count_u16(0b1110101100011010) == 9);
}

TEST_CASE("bit_count_u32", "[bits]") {
  REQUIRE(igb::bit_count_u32(0b11101011000110101010101011110000) == 17);
}

TEST_CASE("bit_index_u8", "[bits]") {
  REQUIRE(igb::bit_index_u8(0b00000100) == 2);
}

TEST_CASE("bit_index_u16", "[bits]") {
  REQUIRE(igb::bit_index_u16(0b0100000000000000) == 14);
}

TEST_CASE("bit_left_rotate_u16", "[bits, rotate]") {
  REQUIRE(igb::bit_left_rotate_u16(0b0010010010010010, 1) == 0b0100100100100100);
}

TEST_CASE("bit_left_rotate_with_length_u16", "[bits, rotate, length]") {
  REQUIRE(igb::bit_left_rotate_with_length_u16(0b0000000000110010, 2, 6) == 0b0000000000001011);
}

TEST_CASE("bit_right_rotate_u16", "[bits, rotate]") {
  REQUIRE(igb::bit_right_rotate_u16(0b0010010010010010, 2) == 0b1000100100100100);
}

TEST_CASE("extract_most_right1", "[bits]") {
  REQUIRE(igb::extract_most_right1((uint8_t)0b00010100) == (uint8_t)0b00000100);
}

TEST_CASE("reset_most_right1", "[bits]") {
  REQUIRE(igb::reset_most_right1((uint8_t)0b00010100) == (uint8_t)0b00010000);
}

