#ifndef IGB_UTIL_RING_BUF_H
#define IGB_UTIL_RING_BUF_H

#include <optional>
#include <igb_util/macro.hpp>

namespace igb {

template<typename DataType, uint32_t data_size>
struct RingBuf {
  DataType _data[data_size]; 
  uint32_t _read_idx = 0;
  uint32_t _end_idx = 0;

  IGB_FAST_INLINE void add(DataType value) {
    _data[_end_idx] = value;
    _end_idx = (_end_idx + 1) % data_size;
  }

  IGB_FAST_INLINE const DataType& peekLast(uint32_t idx_from_last) const {
    uint32_t peek_idx = (_end_idx + data_size - (idx_from_last + 1)) % data_size;
    return _data[peek_idx];
  }

  IGB_FAST_INLINE const DataType& peekNext() const {
    return _data[_read_idx];
  }

  IGB_FAST_INLINE std::optional<DataType> get() {
    std::optional<DataType> v = std::nullopt;
    if (_read_idx != _end_idx) {
      v = _data[_read_idx];
      _read_idx = (_read_idx + 1) % data_size;
    }
    return v;
  }

  IGB_FAST_INLINE void clear() {
    _read_idx = 0;
    _end_idx = 0;
  }

  IGB_FAST_INLINE uint32_t size() const {
    int s = _end_idx - _read_idx;
    if (s < 0) {
      return data_size - _read_idx + _end_idx;
    }
    return s;
  }
};

// fixed size(256) buffer
template<typename DataType>
struct RingBuf256 {
  DataType _data[256];
  uint8_t _read_idx = 0;
  uint8_t _end_idx = 0;

  IGB_FAST_INLINE void add(DataType value) {
    _data[_end_idx++] = value;
  }

  IGB_FAST_INLINE std::optional<DataType> get() {
    std::optional<DataType> v = std::nullopt;
    if (_read_idx != _end_idx) {
      v = _data[_read_idx++];
    }
    return v;
  }

  IGB_FAST_INLINE void clear() {
    _read_idx = 0;
    _end_idx = 0;
  }

  IGB_FAST_INLINE uint8_t size() const {
    return _end_idx - _read_idx;
  }
};

}

#endif /* IGB_UTIL_RING_BUF_H */
