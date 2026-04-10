#pragma once

#include <cstdint>
#include <cstddef>
#include <utility>
#include <igb_util/macro.hpp>
#include <igb_util/dsp/deinterpolator.hpp>

namespace igb::dsp {

template<typename Deinterp = DeinterpNo>
struct LoopBufferStereo {
  std::pair<float, float> _dummy_buf = {0.0f, 0.0f};
  std::pair<float, float>* buf = nullptr;
  size_t buf_size = 1;

  size_t write_pos = 0;      // native-speed recording
  size_t loop_start = 0;     // absolute start of recorded region
  size_t loop_length = 0;    // length of recorded region
  double pos = 0.0;          // position within loop (0 to loop_length)
  double tape_speed = 1.0;

  Deinterp _deinterp;

  LoopBufferStereo() {
    buf = &_dummy_buf;
    buf_size = 1;
  }

  void init(std::pair<float, float>* target_buf, size_t size) {
    buf = target_buf;
    buf_size = size;
  }

  IGB_FAST_INLINE void changeSpeed(double speed) {
    tape_speed = speed;
  }

  // loop-relative position → absolute buffer index
  IGB_FAST_INLINE size_t _toAbsIdx(uint32_t loop_rel) {
    return (loop_start + loop_rel) % buf_size;
  }

  // --- position management ---

  IGB_FAST_INLINE void movePos() {
    pos += tape_speed;
    double len = (double)loop_length;
    if (pos >= len) {
      pos -= len;
    } else if (pos < 0.0) {
      pos += len;
    }
  }

  // --- native-speed recording ---

  IGB_FAST_INLINE void write(std::pair<float, float> value) {
    *(buf + write_pos) = value;
    write_pos = (write_pos + 1) % buf_size;
  }

  // --- variable-speed overdub with de-interpolation ---

  IGB_FAST_INLINE void overdub(std::pair<float, float> value, float feedback) {
    uint32_t pos_u32 = (uint32_t)pos;
    double pos_frac = pos - (double)pos_u32;

    if (tape_speed < 1.0) {
      _overdubSlow(value, feedback, pos_u32, pos_frac);
    } else {
      _overdubFast(value, feedback, pos_u32, pos_frac);
    }

    _deinterp.update(value);
    movePos();
  }

  IGB_FAST_INLINE void _overdubSlow(
      std::pair<float, float> value, float feedback,
      uint32_t pos_u32, double pos_frac) {
    if (pos_frac + tape_speed > 1.0) {
      // crosses integer boundary
      size_t abs0 = _toAbsIdx(pos_u32);
      size_t abs1 = _toAbsIdx(pos_u32 + 1);
      double rate = (1.0 - pos_frac) / tape_speed;
      float gain = (float)tape_speed;
      // leading portion: += (feedback applied by previous trailing edge)
      auto& b0 = *(buf + abs0);
      b0.first  += value.first  * gain * rate;
      b0.second += value.second * gain * rate;
      // trailing portion: feedback + add (first touch of new position)
      float rem = gain * (1.0f - (float)rate);
      auto& b1 = *(buf + abs1);
      b1.first  = b1.first  * feedback + value.first  * rem;
      b1.second = b1.second * feedback + value.second * rem;
    } else if (pos_frac == 0.0) {
      // integer boundary: feedback + add (first touch)
      size_t abs0 = _toAbsIdx(pos_u32);
      float gain = (float)tape_speed;
      auto& b0 = *(buf + abs0);
      b0.first  = b0.first  * feedback + value.first  * gain;
      b0.second = b0.second * feedback + value.second * gain;
    } else {
      // mid-sample: += (feedback already applied)
      size_t abs0 = _toAbsIdx(pos_u32);
      float gain = (float)tape_speed;
      auto& b0 = *(buf + abs0);
      b0.first  += value.first  * gain;
      b0.second += value.second * gain;
    }
  }

  IGB_FAST_INLINE void _overdubFast(
      std::pair<float, float> value, float feedback,
      uint32_t pos_u32, double pos_frac) {
    size_t abs0 = _toAbsIdx(pos_u32);
    // leading edge
    if (pos_frac == 0.0) {
      // integer boundary: feedback + add
      auto& b0 = *(buf + abs0);
      b0.first  = b0.first  * feedback + value.first;
      b0.second = b0.second * feedback + value.second;
    } else {
      // fractional: += (feedback applied by previous trailing edge)
      auto& b0 = *(buf + abs0);
      b0.first  += value.first  * (1.0f - (float)pos_frac);
      b0.second += value.second * (1.0f - (float)pos_frac);
    }

    // intermediate positions: feedback + deinterp
    double next_pos = pos + tape_speed;
    uint32_t next_pos_u32 = (uint32_t)next_pos;
    float next_pos_frac = (float)(next_pos - (double)next_pos_u32);

    float inv_span = 1.0f / (float)(next_pos_u32 - pos_u32);
    for (uint32_t k = pos_u32 + 1; k < next_pos_u32; ++k) {
      float t = (float)(k - pos_u32) * inv_span;
      auto interp = _deinterp(value, t);
      size_t abs_k = _toAbsIdx(k);
      auto& bk = *(buf + abs_k);
      bk.first  = bk.first  * feedback + interp.first;
      bk.second = bk.second * feedback + interp.second;
    }

    // trailing edge: feedback + add
    size_t abs_next = _toAbsIdx(next_pos_u32);
    auto& bn = *(buf + abs_next);
    bn.first  = bn.first  * feedback + value.first  * next_pos_frac;
    bn.second = bn.second * feedback + value.second * next_pos_frac;
  }

  // --- read ---

  IGB_FAST_INLINE std::pair<float, float> readAbs(size_t abs_pos) {
    return *(buf + (abs_pos % buf_size));
  }

  template<typename T>
  IGB_FAST_INLINE std::pair<float, float> readAbsF(T abs_pos) {
    size_t pos_i = (size_t)abs_pos;
    float t = (float)(abs_pos - (T)pos_i);
    size_t idx0 = pos_i % buf_size;
    size_t idx1 = (pos_i + 1) % buf_size;
    auto v0 = *(buf + idx0);
    auto v1 = *(buf + idx1);
    return {
      (1.0f - t) * v0.first  + t * v1.first,
      (1.0f - t) * v0.second + t * v1.second
    };
  }

  // read within loop at current pos (interpolated)
  IGB_FAST_INLINE std::pair<float, float> readLoop() {
    uint32_t pos_i = (uint32_t)pos;
    float t = (float)(pos - (double)pos_i);
    size_t idx0 = _toAbsIdx(pos_i);
    size_t idx1 = _toAbsIdx(pos_i + 1);
    auto v0 = *(buf + idx0);
    auto v1 = *(buf + idx1);
    return {
      (1.0f - t) * v0.first  + t * v1.first,
      (1.0f - t) * v0.second + t * v1.second
    };
  }
};

}
