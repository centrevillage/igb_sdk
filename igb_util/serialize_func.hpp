#ifndef IGB_UTIL_SERIALIZE_FUNC_H
#define IGB_UTIL_SERIALIZE_FUNC_H

#include <string.h>
#include <cstddef>
#include <cstdint>
#include <igb_util/macro.hpp>

namespace igb {

// DEPRECATED
IGB_FAST_INLINE uint8_t* write_uint8_t(uint8_t* buf, const uint8_t v) {
  *(buf++) = v;
  return buf;
}

// DEPRECATED
IGB_FAST_INLINE uint8_t* write_int8_t(uint8_t* buf, const int8_t v) {
  *(buf++) = (uint8_t)v;
  return buf;
}

IGB_FAST_INLINE uint8_t* read_uint8_t(uint8_t* buf, uint8_t& v) {
  v = *(buf++);
  return buf;
}

// DEPRECATED
IGB_FAST_INLINE uint8_t* read_int8_t(uint8_t* buf, int8_t& v) {
  v = (int8_t)(*(buf++));
  return buf;
}

// DEPRECATED
IGB_FAST_INLINE uint8_t* write_uint16_t(uint8_t* buf, const uint16_t v) {
  *(buf++) = (uint8_t)(v >> 8);
  *(buf++) = (uint8_t)v;
  return buf;
}

// DEPRECATED
IGB_FAST_INLINE uint8_t* read_uint16_t(uint8_t* buf, uint16_t& v) {
  const uint16_t msb = *(buf++);
  const uint16_t lsb = *(buf++);
  v = (msb << 8) | lsb;
  return buf;
}

// DEPRECATED
IGB_FAST_INLINE uint8_t* write_uint32_t(uint8_t* buf, const uint32_t v) {
  const uint8_t b1 = (uint8_t)(v >> 24);
  const uint8_t b2 = (uint8_t)(v >> 16);
  const uint8_t b3 = (uint8_t)(v >> 8);
  const uint8_t b4 = (uint8_t)v;
  *(buf++) = b1;
  *(buf++) = b2;
  *(buf++) = b3;
  *(buf++) = b4;
  return buf;
}

// DEPRECATED
IGB_FAST_INLINE uint8_t* read_uint32_t(uint8_t* buf, uint32_t& v) {
  const uint32_t b1 = *(buf++);
  const uint32_t b2 = *(buf++);
  const uint32_t b3 = *(buf++);
  const uint32_t b4 = *(buf++);
  v = (b1 << 24) | (b2 << 16) | (b3 << 8) | b4;
  return buf;
}

// DEPRECATED
IGB_FAST_INLINE uint8_t* write_float_16_16(uint8_t* buf, const float v) {
  const int32_t upper = (int32_t)v;
  const uint32_t abs_upper = upper >= 0 ? upper : -upper;
  const float abs_v = v >= 0 ? v : -v;
  const uint32_t lower = (uint32_t)((abs_v - (float)abs_upper) * 65536.0f);
  const uint8_t b1 = (uint8_t)(((abs_upper >> 8) & 0x7F) | (!!(v < 0) << 7));
  const uint8_t b2 = (uint8_t)abs_upper;
  const uint8_t b3 = (uint8_t)(lower >> 8);
  const uint8_t b4 = (uint8_t)lower;
  *(buf++) = b1;
  *(buf++) = b2;
  *(buf++) = b3;
  *(buf++) = b4;
  return buf;
}

// DEPRECATED
IGB_FAST_INLINE uint8_t* write_float_raw(uint8_t* buf, const float v) {
  memcpy(buf, &v, sizeof(v));
  return buf + (sizeof(v));
}

// DEPRECATED
IGB_FAST_INLINE uint8_t* read_float_16_16(uint8_t* buf, float& v) {
  const uint8_t b1 = *(buf++);
  const uint8_t b2 = *(buf++);
  const uint8_t b3 = *(buf++);
  const uint8_t b4 = *(buf++);
  const float upper = (float)((uint32_t)((((b1 & 0x7F) << 8)) | b2));
  const float lower = (float)((uint32_t)((b3 << 8) | b4)) / 65536.0f;
  v = upper + lower;
  if (b1 & 0x80) {
    v = -v;
  }
  return buf;
}

// DEPRECATED
IGB_FAST_INLINE uint8_t* read_float_raw(uint8_t* buf, float& v) {
  memcpy(&v, buf, sizeof(v));
  return buf + (sizeof(v));
}

// DEPRECATED
template<typename T>
IGB_FAST_INLINE uint8_t* write_buf(uint8_t* buf, const T type) {
  return buf;
}

// DEPRECATED
template<>
IGB_FAST_INLINE uint8_t* write_buf(uint8_t* buf, const uint8_t v) {
  return write_uint8_t(buf, v);
}

// DEPRECATED
template<>
IGB_FAST_INLINE uint8_t* write_buf(uint8_t* buf, const int8_t v) {
  return write_int8_t(buf, v);
}

// DEPRECATED
template<>
IGB_FAST_INLINE uint8_t* write_buf(uint8_t* buf, const uint16_t v) {
  return write_uint16_t(buf, v);
}

// DEPRECATED
template<>
IGB_FAST_INLINE uint8_t* write_buf(uint8_t* buf, const uint32_t v) {
  return write_uint32_t(buf, v);
}

// DEPRECATED
template<>
IGB_FAST_INLINE uint8_t* write_buf(uint8_t* buf, const float v) {
  return write_float_raw(buf, v);
}

// DEPRECATED
template<typename T>
IGB_FAST_INLINE uint8_t* read_buf(uint8_t* buf, T& type) {
  return buf;
}

// DEPRECATED
template<>
IGB_FAST_INLINE uint8_t* read_buf(uint8_t* buf, uint8_t& v) {
  return read_uint8_t(buf, v);
}

// DEPRECATED
template<>
IGB_FAST_INLINE uint8_t* read_buf(uint8_t* buf, int8_t& v) {
  return read_int8_t(buf, v);
}

// DEPRECATED
template<>
IGB_FAST_INLINE uint8_t* read_buf(uint8_t* buf, uint16_t& v) {
  return read_uint16_t(buf, v);
}

// DEPRECATED
template<>
IGB_FAST_INLINE uint8_t* read_buf(uint8_t* buf, uint32_t& v) {
  return read_uint32_t(buf, v);
}

// DEPRECATED
template<>
IGB_FAST_INLINE uint8_t* read_buf(uint8_t* buf, float& v) {
  return read_float_raw(buf, v);
}

template<typename T>
IGB_FAST_INLINE size_t serialize(uint8_t* buf, const T type) {
  return 0;
}

template<>
IGB_FAST_INLINE size_t serialize(uint8_t* buf, const uint8_t v) {
  buf[0] = v;
  return sizeof(v);
}

template<>
IGB_FAST_INLINE size_t serialize(uint8_t* buf, const int8_t v) {
  buf[0] = (uint8_t)v;
  return sizeof(v);
}

template<>
IGB_FAST_INLINE size_t serialize(uint8_t* buf, const uint16_t v) {
  // maybe little-endian
  memcpy(buf, &v, sizeof(v));
  return sizeof(v);
}

template<>
IGB_FAST_INLINE size_t serialize(uint8_t* buf, const int16_t v) {
  // maybe little-endian
  memcpy(buf, &v, sizeof(v));
  return sizeof(v);
}

template<>
IGB_FAST_INLINE size_t serialize(uint8_t* buf, const uint32_t v) {
  // maybe little-endian
  memcpy(buf, &v, sizeof(v));
  return sizeof(v);
}

template<>
IGB_FAST_INLINE size_t serialize(uint8_t* buf, const uint64_t v) {
  // maybe little-endian
  memcpy(buf, &v, sizeof(v));
  return sizeof(v);
}

template<>
IGB_FAST_INLINE size_t serialize(uint8_t* buf, const int64_t v) {
  // maybe little-endian
  memcpy(buf, &v, sizeof(v));
  return sizeof(v);
}

template<>
IGB_FAST_INLINE size_t serialize(uint8_t* buf, const int32_t v) {
  // maybe little-endian
  memcpy(buf, &v, sizeof(v));
  return sizeof(v);
}

template<>
IGB_FAST_INLINE size_t serialize(uint8_t* buf, const float v) {
  // maybe little-endian
  memcpy(buf, &v, sizeof(v));
  return sizeof(v);
}

template<>
IGB_FAST_INLINE size_t serialize(uint8_t* buf, const double v) {
  // maybe little-endian
  memcpy(buf, &v, sizeof(v));
  return sizeof(v);
}

template<typename T>
IGB_FAST_INLINE size_t deserialize(uint8_t* buf, T& type) {
  return 0;
}

template<>
IGB_FAST_INLINE size_t deserialize(uint8_t* buf, uint8_t& v) {
  v = buf[0];
  return sizeof(v);
}

template<>
IGB_FAST_INLINE size_t deserialize(uint8_t* buf, int8_t& v) {
  v = (int8_t)buf[0];
  return sizeof(v);
}

template<>
IGB_FAST_INLINE size_t deserialize(uint8_t* buf, uint16_t& v) {
  memcpy(&v, buf, sizeof(v));
  return sizeof(v);
}

template<>
IGB_FAST_INLINE size_t deserialize(uint8_t* buf, int16_t& v) {
  memcpy(&v, buf, sizeof(v));
  return sizeof(v);
}

template<>
IGB_FAST_INLINE size_t deserialize(uint8_t* buf, uint32_t& v) {
  memcpy(&v, buf, sizeof(v));
  return sizeof(v);
}

template<>
IGB_FAST_INLINE size_t deserialize(uint8_t* buf, int32_t& v) {
  memcpy(&v, buf, sizeof(v));
  return sizeof(v);
}

template<>
IGB_FAST_INLINE size_t deserialize(uint8_t* buf, uint64_t& v) {
  memcpy(&v, buf, sizeof(v));
  return sizeof(v);
}

template<>
IGB_FAST_INLINE size_t deserialize(uint8_t* buf, int64_t& v) {
  memcpy(&v, buf, sizeof(v));
  return sizeof(v);
}

template<>
IGB_FAST_INLINE size_t deserialize(uint8_t* buf, float& v) {
  memcpy(&v, buf, sizeof(v));
  return sizeof(v);
}

template<>
IGB_FAST_INLINE size_t deserialize(uint8_t* buf, double& v) {
  memcpy(&v, buf, sizeof(v));
  return sizeof(v);
}

}

#endif /* IGB_UTIL_SERIALIZE_FUNC_H */
