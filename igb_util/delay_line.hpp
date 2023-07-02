#pragma once

#include <cstdint>
#include <cstddef>
#include <igb_util/macro.hpp>

namespace igb {

struct DelayLine {
  size_t write_pos = 0;
  float dummy_buf = 0.0f;
  float* buf = nullptr;
  size_t buf_size = 0;

  DelayLine() {
    buf = &dummy_buf;
    buf_size = 1;
  }

  void init(float* target_buf, size_t size) {
    buf = target_buf;
    buf_size = size;
  }

  IGB_FAST_INLINE void write(float value) {
    *(buf + write_pos) = value;
    write_pos = (write_pos + 1) % buf_size;
  }

  IGB_FAST_INLINE float read(size_t pos) {
    size_t read_pos = (write_pos - pos + buf_size) % buf_size;
    return *(buf + read_pos);
  }

  template<typename T> // T = float or double
  IGB_FAST_INLINE float readF(T pos) {
    size_t pos_i = (size_t)pos;
    float t = pos - (T)pos_i;
    size_t read_pos_i = (write_pos - pos_i + buf_size) % buf_size;
    size_t read_pos_i_prev = (read_pos_i - 1 + buf_size) % buf_size;
    float v = *(buf + read_pos_i);
    float v_prev = *(buf + read_pos_i_prev);
    return (1.0f - t) * v + t * v_prev;
  }
};

}

