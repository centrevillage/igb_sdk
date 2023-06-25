#include <catch2/catch_test_macros.hpp>

#include <igb_util/serialize_func.hpp>

TEST_CASE("write_buf_read_buf", "[float value]") {
  float v = 123.456f;
  uint8_t buf[4];
  igb::write_buf(buf, v);
  float v2;
  igb::read_buf(buf, v2);
  REQUIRE(v2 == v);
}

