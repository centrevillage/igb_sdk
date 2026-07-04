#pragma once

#include <cstdint>
#include <cstddef>
#include <igb_util/macro.hpp>

namespace igb::dsp {

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

struct DelayLineStereo {
  size_t write_pos = 0;
  std::pair<float, float> _dummy_buf = {0.0f, 0.0f};
  std::pair<float, float>* buf = nullptr;
  size_t buf_size = 0;

  DelayLineStereo() {
    buf = &_dummy_buf;
    buf_size = 1;
  }

  void init(std::pair<float, float>* target_buf, size_t size) {
    buf = target_buf;
    buf_size = size;
  }

  // 録音: ネイティブ速度で1サンプル書き込み、write_pos +1
  IGB_FAST_INLINE void write(std::pair<float, float> value) {
    *(buf + write_pos) = value;
    write_pos = (write_pos + 1) % buf_size;
  }

  // 絶対位置への書き込み (オーバーダブ用)
  IGB_FAST_INLINE void writeAt(size_t pos, std::pair<float, float> value) {
    *(buf + (pos % buf_size)) = value;
  }

  // 再生: write_posからの逆方向オフセットで読み取り (整数位置)
  IGB_FAST_INLINE std::pair<float, float> read(size_t pos) {
    size_t read_pos = (write_pos - pos + buf_size) % buf_size;
    return *(buf + read_pos);
  }

  // 絶対位置からの読み取り (整数位置)
  IGB_FAST_INLINE std::pair<float, float> readAbs(size_t pos) {
    return *(buf + (pos % buf_size));
  }

  // 絶対位置からの補間付き読み取り (小数位置、可変速度再生用)
  template<typename T>
  IGB_FAST_INLINE std::pair<float, float> readAbsF(T pos) {
    size_t pos_i = (size_t)pos;
    float t = (float)(pos - (T)pos_i);
    size_t idx0 = pos_i % buf_size;
    size_t idx1 = (pos_i + 1) % buf_size;
    auto v0 = *(buf + idx0);
    auto v1 = *(buf + idx1);
    return {
      (1.0f - t) * v0.first  + t * v1.first,
      (1.0f - t) * v0.second + t * v1.second
    };
  }

  // write_posからの逆方向オフセットで補間付き読み取り
  template<typename T>
  IGB_FAST_INLINE std::pair<float, float> readF(T pos) {
    size_t pos_i = (size_t)pos;
    float t = (float)(pos - (T)pos_i);
    size_t read_pos_i = (write_pos - pos_i + buf_size) % buf_size;
    size_t read_pos_i_prev = (read_pos_i - 1 + buf_size) % buf_size;
    auto v = *(buf + read_pos_i);
    auto v_prev = *(buf + read_pos_i_prev);
    return {
      (1.0f - t) * v.first  + t * v_prev.first,
      (1.0f - t) * v.second + t * v_prev.second
    };
  }

  // オーバーダブ開始時にwrite_posをread_posの実アドレスに同期
  void syncWritePos(size_t abs_pos) {
    write_pos = abs_pos % buf_size;
  }
};

}

