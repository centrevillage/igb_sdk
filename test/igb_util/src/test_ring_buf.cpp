#include <catch2/catch_test_macros.hpp>

#include <igb_util/ring_buf.hpp>

TEST_CASE("RingBuf::add", "[value]") {
  igb::RingBuf<uint8_t, 3> buf;

  buf.add(1);
  buf.add(2);

  REQUIRE(buf.size() == 2);
  REQUIRE(buf.get().value() == 1);
  REQUIRE(buf.get().value() == 2);
  REQUIRE(!buf.get().has_value());
  REQUIRE(buf.size() == 0);

  // size over
  buf.add(1);
  buf.add(2);
  buf.add(3);
  REQUIRE(buf.size() == 0);
  REQUIRE(!buf.get().has_value());

  buf.add(4);
  REQUIRE(buf.size() == 1);
  REQUIRE(buf.get().value() == 4);
}

TEST_CASE("RingBuf256::add", "[value]") {
  igb::RingBuf256<uint8_t> buf;

  buf.add(1);
  buf.add(2);
  buf.add(3);

  REQUIRE((int)buf.size() == 3);
  REQUIRE(buf.get().value() == 1);
  REQUIRE(buf.get().value() == 2);
  REQUIRE(buf.get().value() == 3);
  REQUIRE(!buf.get().has_value());
  REQUIRE(buf.size() == 0);
}

