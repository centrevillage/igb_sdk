#include <catch2/catch_test_macros.hpp>

#include <igb_util/bitmagic.hpp>

TEST_CASE("bit_count_u8", "[bits]") {
  REQUIRE(igb::bit_count_u8(0b11101011) == 6);
}
