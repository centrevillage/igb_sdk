#include <catch2/catch_test_macros.hpp>

#include <igb_util/serializer.hpp>

struct TestStruct {
  uint8_t v1;
  uint8_t v2;

  constexpr static size_t serialized_buf_size = 2;
  size_t serialize(uint8_t* buf, size_t sz) const {
    if (sz < serialized_buf_size) { return 0; }
    buf += igb::Serializer::serialize(buf, v1);
    buf += igb::Serializer::serialize(buf, v2);
    return serialized_buf_size;
  }

  size_t deserialize(uint8_t* buf, size_t sz) {
    if (sz < serialized_buf_size) { return 0; }
    buf += igb::Serializer::deserialize(buf, v1);
    buf += igb::Serializer::deserialize(buf, v2);
    return serialized_buf_size;
  }
};

TEST_CASE("serialize_deserialize_serializable_obj") {
  TestStruct test1 {1, 2};
  uint8_t buf[4];
  igb::Serializer::serialize(buf, test1);
  TestStruct test2;
  igb::Serializer::deserialize(buf, test2);
  REQUIRE(test2.v1 == test1.v1);
  REQUIRE(test2.v2 == test1.v2);
}

TEST_CASE("serialize_deserialize_uint8_t") {
  uint8_t v = 123;
  uint8_t buf[4];
  igb::Serializer::serialize(buf, v);
  uint8_t v2;
  igb::Serializer::deserialize(buf, v2);
  REQUIRE(v2 == v);
}

TEST_CASE("serialize_deserialize_int8_t") {
  int8_t v = -123;
  uint8_t buf[4];
  igb::Serializer::serialize(buf, v);
  int8_t v2;
  igb::Serializer::deserialize(buf, v2);
  REQUIRE(v2 == v);
}


TEST_CASE("serialize_deserialize_uint16_t") {
  uint16_t v = 12345;
  uint8_t buf[4];
  igb::Serializer::serialize(buf, v);
  uint16_t v2;
  igb::Serializer::deserialize(buf, v2);
  REQUIRE(v2 == v);
}

TEST_CASE("serialize_deserialize_int16_t") {
  int16_t v = -12345;
  uint8_t buf[4];
  igb::Serializer::serialize(buf, v);
  int16_t v2;
  igb::Serializer::deserialize(buf, v2);
  REQUIRE(v2 == v);
}

TEST_CASE("serialize_deserialize_uint32_t") {
  uint32_t v = 123456;
  uint8_t buf[4];
  igb::Serializer::serialize(buf, v);
  uint32_t v2;
  igb::Serializer::deserialize(buf, v2);
  REQUIRE(v2 == v);
}

TEST_CASE("serialize_deserialize_int32_t") {
  int32_t v = -123456;
  uint8_t buf[4];
  igb::Serializer::serialize(buf, v);
  int32_t v2;
  igb::Serializer::deserialize(buf, v2);
  REQUIRE(v2 == v);
}

TEST_CASE("serialize_deserialize_float") {
  float v = 123.456f;
  uint8_t buf[4];
  igb::Serializer::serialize(buf, v);
  float v2;
  igb::Serializer::deserialize(buf, v2);
  REQUIRE(v2 == v);
}

TEST_CASE("serialize_deserialize_double") {
  double v = 123.456;
  uint8_t buf[8];
  igb::Serializer::serialize(buf, v);
  double v2;
  igb::Serializer::deserialize(buf, v2);
  REQUIRE(v2 == v);
}

