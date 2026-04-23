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
  volatile float pos_snapshot = 0.0f;  // atomic 32-bit view of pos for non-audio readers

  Deinterp _deinterp;
  uint32_t _last_fb_pos = UINT32_MAX;  // feedback tracking

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

  void resetFeedbackPos() {
    _last_fb_pos = UINT32_MAX;
  }

  // loop-relative position → absolute buffer index
  IGB_FAST_INLINE size_t _toAbsIdx(uint32_t loop_rel) {
    return (loop_start + loop_rel) % buf_size;
  }

  // feedback-aware write: apply feedback on first touch, += on revisit
  IGB_FAST_INLINE void _writeFb(
      size_t abs_idx, uint32_t loop_pos,
      float val_l, float val_r, float feedback, bool update_last) {
    auto& b = *(buf + abs_idx);
    if (loop_pos != _last_fb_pos) {
      b.first  = b.first  * feedback + val_l;
      b.second = b.second * feedback + val_r;
    } else {
      b.first  += val_l;
      b.second += val_r;
    }
    if (update_last) {
      _last_fb_pos = loop_pos;
    }
  }

  // --- position management ---

  // Advance pos by tape_speed, wrapping at loop boundaries. Returns true if a
  // wrap occurred (forward past loop_length, or reverse past 0). Callers that
  // don't need the wrap signal can ignore the return value.
  IGB_FAST_INLINE bool movePos() {
    pos += tape_speed;
    double len = (double)loop_length;
    bool wrapped = false;
    if (pos >= len) {
      pos -= len;
      wrapped = true;
    } else if (pos < 0.0) {
      pos += len;
      wrapped = true;
    }
    pos_snapshot = (float)pos;
    return wrapped;
  }

  IGB_FAST_INLINE void resetPos(double new_pos = 0.0) {
    pos = new_pos;
    pos_snapshot = (float)new_pos;
  }

  // --- native-speed recording ---

  IGB_FAST_INLINE void write(std::pair<float, float> value) {
    *(buf + write_pos) = value;
    write_pos = (write_pos + 1) % buf_size;
  }

  // Issue #57 ループ境界スムージング用。loop-relative 位置に feedback 付き
  // の加算 (scatter 書き込み)。`_last_fb_pos` は更新しない (OD 側の scatter
  // 追跡と干渉させないため)。
  IGB_FAST_INLINE void fadeInAddAt(
      uint32_t loop_pos, float val_l, float val_r, float feedback) {
    _writeFb(_toAbsIdx(loop_pos), loop_pos, val_l, val_r, feedback, false);
  }

  // Issue #57 ループ境界スムージング用。loop-relative 位置の既存値に gain を
  // in-place 乗算 (録音 tail の fade_out retroactive 用)。
  IGB_FAST_INLINE void gainAt(uint32_t loop_pos, float gain) {
    auto& b = *(buf + _toAbsIdx(loop_pos));
    b.first  *= gain;
    b.second *= gain;
  }

  // --- variable-speed overdub with de-interpolation ---

  // Scatter-write `value` at current pos with feedback, advance pos, and
  // return true if the pos advance crossed the loop boundary. The wrap
  // signal lets callers detect OD one-lap-completed without an extra
  // prev_pos/new_pos fetch pair (LilaCRepeater issue #64).
  IGB_FAST_INLINE bool overdub(std::pair<float, float> value, float feedback) {
    uint32_t pos_u32 = (uint32_t)pos;
    double pos_frac = pos - (double)pos_u32;

    if (tape_speed >= 0.0) {
      if (tape_speed < 1.0) {
        _overdubSlow(value, feedback, pos_u32, pos_frac);
      } else {
        _overdubFast(value, feedback, pos_u32, pos_frac);
      }
    } else {
      double abs_speed = -tape_speed;
      if (abs_speed < 1.0) {
        _overdubSlowReverse(value, feedback, pos_u32, pos_frac, abs_speed);
      } else {
        _overdubFastReverse(value, feedback, pos_u32, pos_frac);
      }
    }

    _deinterp.update(value);
    return movePos();
  }

  // --- forward scatter ---

  IGB_FAST_INLINE void _overdubSlow(
      std::pair<float, float> value, float feedback,
      uint32_t pos_u32, double pos_frac) {
    float gain = (float)tape_speed;
    if (pos_frac + tape_speed > 1.0) {
      // crosses integer boundary
      double rate = (1.0 - pos_frac) / tape_speed;
      // leading portion
      _writeFb(_toAbsIdx(pos_u32), pos_u32,
               value.first * gain * (float)rate,
               value.second * gain * (float)rate,
               feedback, false);
      // trailing portion (update _last_fb_pos). Issue #67: wrap by loop_length
      // so the contribution lands on loop_pos 0 when we cross the boundary,
      // instead of writing past the loop into unused buffer space.
      uint32_t next = (pos_u32 + 1) % loop_length;
      float rem = gain * (1.0f - (float)rate);
      _writeFb(_toAbsIdx(next), next,
               value.first * rem, value.second * rem,
               feedback, true);
    } else if (pos_frac == 0.0) {
      // integer boundary
      _writeFb(_toAbsIdx(pos_u32), pos_u32,
               value.first * gain, value.second * gain,
               feedback, true);
    } else {
      // mid-sample
      _writeFb(_toAbsIdx(pos_u32), pos_u32,
               value.first * gain, value.second * gain,
               feedback, false);
    }
  }

  IGB_FAST_INLINE void _overdubFast(
      std::pair<float, float> value, float feedback,
      uint32_t pos_u32, double pos_frac) {
    // leading edge
    float lead = (pos_frac == 0.0) ? 1.0f : (1.0f - (float)pos_frac);
    _writeFb(_toAbsIdx(pos_u32), pos_u32,
             value.first * lead, value.second * lead,
             feedback, false);

    // intermediate positions. Issue #67: span is computed on raw (unwrapped)
    // indices so `t` stays a clean fraction, but every `_writeFb` loop_pos is
    // wrapped by loop_length so contributions that straddle the boundary land
    // on loop positions 0..N-1 instead of past-the-end slots.
    double next_pos = pos + tape_speed;
    uint32_t next_pos_raw = (uint32_t)next_pos;
    float next_pos_frac = (float)(next_pos - (double)next_pos_raw);

    float inv_span = 1.0f / (float)(next_pos_raw - pos_u32);
    for (uint32_t k = pos_u32 + 1; k < next_pos_raw; ++k) {
      uint32_t k_wrapped = k % loop_length;
      float t = (float)(k - pos_u32) * inv_span;
      auto interp = _deinterp(value, t);
      _writeFb(_toAbsIdx(k_wrapped), k_wrapped,
               interp.first, interp.second,
               feedback, false);
    }

    // trailing edge (update _last_fb_pos with the wrapped loop pos so the
    // next call's lead at loop_pos 0 correctly hits the REVISIT branch).
    uint32_t next_pos_u32 = next_pos_raw % loop_length;
    _writeFb(_toAbsIdx(next_pos_u32), next_pos_u32,
             value.first * next_pos_frac, value.second * next_pos_frac,
             feedback, true);
  }

  // --- reverse scatter ---

  IGB_FAST_INLINE void _overdubSlowReverse(
      std::pair<float, float> value, float feedback,
      uint32_t pos_u32, double pos_frac, double abs_speed) {
    float gain = (float)abs_speed;
    if (pos_frac > 0.0 && pos_frac < abs_speed) {
      // crosses integer boundary (backward)
      float rate = (float)(pos_frac / abs_speed);
      // leading portion (current position)
      _writeFb(_toAbsIdx(pos_u32), pos_u32,
               value.first * gain * rate,
               value.second * gain * rate,
               feedback, false);
      // trailing portion (previous position, update _last_fb_pos)
      uint32_t prev = (pos_u32 + loop_length - 1) % loop_length;
      float rem = gain * (1.0f - rate);
      _writeFb(_toAbsIdx(prev), prev,
               value.first * rem, value.second * rem,
               feedback, true);
    } else if (pos_frac == 0.0) {
      // integer boundary: entering previous position
      uint32_t prev = (pos_u32 + loop_length - 1) % loop_length;
      _writeFb(_toAbsIdx(prev), prev,
               value.first * gain, value.second * gain,
               feedback, true);
    } else {
      // mid-sample
      _writeFb(_toAbsIdx(pos_u32), pos_u32,
               value.first * gain, value.second * gain,
               feedback, false);
    }
  }

  IGB_FAST_INLINE void _overdubFastReverse(
      std::pair<float, float> value, float feedback,
      uint32_t pos_u32, double pos_frac) {
    // leading edge (at current pos)
    float lead = (pos_frac == 0.0) ? 1.0f : (float)pos_frac;
    _writeFb(_toAbsIdx(pos_u32), pos_u32,
             value.first * lead, value.second * lead,
             feedback, false);

    // backward destination (wrapped to positive)
    double dest = pos + tape_speed;
    if (dest < 0.0) dest += (double)loop_length;
    uint32_t dest_u32 = (uint32_t)dest;
    float dest_frac = (float)(dest - (double)dest_u32);

    // intermediate positions (backward)
    uint32_t span = (pos_u32 + loop_length - dest_u32) % loop_length;
    if (span > 1) {
      float inv_span = 1.0f / (float)span;
      for (uint32_t i = 1; i < span; ++i) {
        uint32_t k = (pos_u32 + loop_length - i) % loop_length;
        float t = (float)i * inv_span;
        auto interp = _deinterp(value, t);
        _writeFb(_toAbsIdx(k), k,
                 interp.first, interp.second,
                 feedback, false);
      }
    }

    // trailing edge (update _last_fb_pos)
    float trailing = 1.0f - dest_frac;
    _writeFb(_toAbsIdx(dest_u32), dest_u32,
             value.first * trailing, value.second * trailing,
             feedback, true);
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
    size_t idx1 = _toAbsIdx((pos_i + 1) % loop_length);
    auto v0 = *(buf + idx0);
    auto v1 = *(buf + idx1);
    return {
      (1.0f - t) * v0.first  + t * v1.first,
      (1.0f - t) * v0.second + t * v1.second
    };
  }
};

}
