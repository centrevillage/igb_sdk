#pragma once

#include <string.h>
#include <cstddef>
#include <cstdint>
#include <array>
#include <igb_util/macro.hpp>

namespace igb {

struct Serializer {
  IGB_FAST_INLINE static size_t serialize(uint8_t* buf, const uint8_t v) {
    buf[0] = v;
    return sizeof(v);
  }

  IGB_FAST_INLINE static size_t serialize(uint8_t* buf, const int8_t v) {
    buf[0] = (uint8_t)v;
    return sizeof(v);
  }

  IGB_FAST_INLINE static size_t serialize(uint8_t* buf, const uint16_t v) {
    // maybe little-endian
    memcpy(buf, &v, sizeof(v));
    return sizeof(v);
  }

  IGB_FAST_INLINE static size_t serialize(uint8_t* buf, const int16_t v) {
    // maybe little-endian
    memcpy(buf, &v, sizeof(v));
    return sizeof(v);
  }

  IGB_FAST_INLINE static size_t serialize(uint8_t* buf, const uint32_t v) {
    // maybe little-endian
    memcpy(buf, &v, sizeof(v));
    return sizeof(v);
  }

  IGB_FAST_INLINE static size_t serialize(uint8_t* buf, const uint64_t v) {
    // maybe little-endian
    memcpy(buf, &v, sizeof(v));
    return sizeof(v);
  }

  IGB_FAST_INLINE static size_t serialize(uint8_t* buf, const int64_t v) {
    // maybe little-endian
    memcpy(buf, &v, sizeof(v));
    return sizeof(v);
  }

  IGB_FAST_INLINE static size_t serialize(uint8_t* buf, const int32_t v) {
    // maybe little-endian
    memcpy(buf, &v, sizeof(v));
    return sizeof(v);
  }

  IGB_FAST_INLINE static size_t serialize(uint8_t* buf, const float v) {
    // maybe little-endian
    memcpy(buf, &v, sizeof(v));
    return sizeof(v);
  }

  IGB_FAST_INLINE static size_t serialize(uint8_t* buf, const double v) {
    // maybe little-endian
    memcpy(buf, &v, sizeof(v));
    return sizeof(v);
  }

  IGB_FAST_INLINE static size_t deserialize(uint8_t* buf, uint8_t& v) {
    v = buf[0];
    return sizeof(v);
  }

  IGB_FAST_INLINE static size_t deserialize(uint8_t* buf, int8_t& v) {
    v = (int8_t)buf[0];
    return sizeof(v);
  }

  IGB_FAST_INLINE static size_t deserialize(uint8_t* buf, uint16_t& v) {
    memcpy(&v, buf, sizeof(v));
    return sizeof(v);
  }

  IGB_FAST_INLINE static size_t deserialize(uint8_t* buf, int16_t& v) {
    memcpy(&v, buf, sizeof(v));
    return sizeof(v);
  }

  IGB_FAST_INLINE static size_t deserialize(uint8_t* buf, uint32_t& v) {
    memcpy(&v, buf, sizeof(v));
    return sizeof(v);
  }

  IGB_FAST_INLINE static size_t deserialize(uint8_t* buf, int32_t& v) {
    memcpy(&v, buf, sizeof(v));
    return sizeof(v);
  }

  IGB_FAST_INLINE static size_t deserialize(uint8_t* buf, uint64_t& v) {
    memcpy(&v, buf, sizeof(v));
    return sizeof(v);
  }

  IGB_FAST_INLINE static size_t deserialize(uint8_t* buf, int64_t& v) {
    memcpy(&v, buf, sizeof(v));
    return sizeof(v);
  }

  IGB_FAST_INLINE static size_t deserialize(uint8_t* buf, float& v) {
    memcpy(&v, buf, sizeof(v));
    return sizeof(v);
  }

  IGB_FAST_INLINE static size_t deserialize(uint8_t* buf, double& v) {
    memcpy(&v, buf, sizeof(v));
    return sizeof(v);
  }

  constexpr static size_t size(const uint8_t v) {
    return sizeof(v);
  }

  constexpr static size_t size(const int8_t v) {
    return sizeof(v);
  }

  constexpr static size_t size(const uint16_t v) {
    return sizeof(v);
  }

  constexpr static size_t size(const int16_t v) {
    return sizeof(v);
  }

  constexpr static size_t size(const uint32_t v) {
    return sizeof(v);
  }

  constexpr static size_t size(const uint64_t v) {
    return sizeof(v);
  }

  constexpr static size_t size(const int64_t v) {
    return sizeof(v);
  }

  constexpr static size_t size(const int32_t v) {
    return sizeof(v);
  }

  constexpr static size_t size(const float v) {
    return sizeof(v);
  }

  constexpr static size_t size(const double v) {
    return sizeof(v);
  }

  template<typename T, size_t array_size>
  IGB_FAST_INLINE static size_t serialize(uint8_t* buf, const std::array<T, array_size>& values) {
    for (const auto& v : values) {
      buf += serialize(buf, v);
    }
    return size(values[0]) * array_size;
  }

  template<typename T, size_t array_size>
  IGB_FAST_INLINE static size_t deserialize(uint8_t* buf, std::array<T, array_size>& values) {
    for (auto& v : values) {
      buf += deserialize(buf, v);
    }
    return size(values[0]) * array_size;
  }

  template<typename T, size_t array_size>
  constexpr static size_t size(const std::array<T, array_size>& values) {
    return size(values[0]) * array_size;
  }

  template<typename T>
  constexpr static size_t serialize(uint8_t* buf, const T type) {
    return type.serialize(buf, type.serialized_buf_size);
  }

  template<typename T>
  constexpr static size_t deserialize(uint8_t* buf, T& type) {
    return type.deserialize(buf, type.serialized_buf_size);
  }

  template<typename T>
  constexpr static size_t size(T& type) {
    return type.serialized_buf_size;
  }
};

}
