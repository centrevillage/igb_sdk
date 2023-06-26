#include <catch2/catch_test_macros.hpp>

#include <igb_util/serialize_func.hpp>

TEST_CASE("serialize_deserialize_uint8_t") {
  uint8_t v = 123;
  uint8_t buf[4];
  igb::serialize(buf, v);
  uint8_t v2;
  igb::deserialize(buf, v2);
  REQUIRE(v2 == v);
}

TEST_CASE("serialize_deserialize_int8_t") {
  int8_t v = -123;
  uint8_t buf[4];
  igb::serialize(buf, v);
  int8_t v2;
  igb::deserialize(buf, v2);
  REQUIRE(v2 == v);
}


TEST_CASE("serialize_deserialize_uint16_t") {
  uint16_t v = 12345;
  uint8_t buf[4];
  igb::serialize(buf, v);
  uint16_t v2;
  igb::deserialize(buf, v2);
  REQUIRE(v2 == v);
}

TEST_CASE("serialize_deserialize_int16_t") {
  int16_t v = -12345;
  uint8_t buf[4];
  igb::serialize(buf, v);
  int16_t v2;
  igb::deserialize(buf, v2);
  REQUIRE(v2 == v);
}

TEST_CASE("serialize_deserialize_uint32_t") {
  uint32_t v = 123456;
  uint8_t buf[4];
  igb::serialize(buf, v);
  uint32_t v2;
  igb::deserialize(buf, v2);
  REQUIRE(v2 == v);
}

TEST_CASE("serialize_deserialize_int32_t") {
  int32_t v = -123456;
  uint8_t buf[4];
  igb::serialize(buf, v);
  int32_t v2;
  igb::deserialize(buf, v2);
  REQUIRE(v2 == v);
}

TEST_CASE("serialize_deserialize_float") {
  float v = 123.456f;
  uint8_t buf[4];
  igb::serialize(buf, v);
  float v2;
  igb::deserialize(buf, v2);
  REQUIRE(v2 == v);
}

TEST_CASE("serialize_deserialize_double") {
  double v = 123.456;
  uint8_t buf[8];
  igb::serialize(buf, v);
  double v2;
  igb::deserialize(buf, v2);
  REQUIRE(v2 == v);
}

