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
  size_t loop_start = 0;     // absolute start of recorded region (LilaC: always 0)
  size_t loop_length = 0;    // length of recorded region (= recorded content ring)
  double pos = 0.0;          // position within the WINDOW (0 to winLen())
  double tape_speed = 1.0;
  volatile float pos_snapshot = 0.0f;  // atomic 32-bit view of pos for non-audio readers

  // LilaCRepeater issue #121: non-destructive playback WINDOW (loop range).
  // The recorded content stays at absolute [0, loop_length) (loop_start≡0 is a
  // hard invariant: the save/load body base and the OD→alt undo copy both
  // address absolute content, so the window must NOT be expressed via loop_start
  // — see docs/121). Instead the window is a sub-range of the content ring:
  //   win_len == 0  → full window (start 0, length loop_length) — the default.
  //   win_len  > 0  → sub-window: pos wraps at win_len, and a window-relative
  //                   index w maps to absolute content (win_start + w) % loop_length
  //                   (the % loop_length lets the window WRAP past the recording
  //                   end, e.g. start=1,length=full). win_start/win_len are derived
  //                   from the per-track play_start_step/play_length_step by the
  //                   domain layer and kept ≤ loop_length.
  // win_len is a DOUBLE (fractional samples): the domain sets it to the tick-exact
  // length_step*6*recorded_length/recorded_ticks so a window wraps in exactly
  // length_step*6 ticks and stays clock-grid-locked (the #121 generalization of
  // the #89 full-loop exactness). win_start stays an integer content offset
  // (a sub-sample start offset has no timing effect).
  size_t win_start = 0;
  double win_len = 0.0;      // 0 = full window (sentinel); else fractional samples

  // Effective window (clamped so an invalid/stale window degrades to full).
  IGB_FAST_INLINE double winLen() const {
    return (win_len == 0.0 || win_len > (double)loop_length) ? (double)loop_length : win_len;
  }
  IGB_FAST_INLINE size_t winStart() const {
    return (win_len == 0.0 || win_len > (double)loop_length || win_start >= loop_length)
             ? 0 : win_start;
  }
  IGB_FAST_INLINE bool hasWindow() const {
    return win_len != 0.0 && win_len <= (double)loop_length && win_start < loop_length
        && !(win_start == 0 && win_len == (double)loop_length);
  }
  // Integer window length for the overdub scatter modulus: ceil(win_len) so the
  // last (fractional) window sample is still a valid scatter slot. == winLen()
  // for an integer / full window.
  IGB_FAST_INLINE uint32_t _winLenInt() const {
    double wl = winLen();
    uint32_t i = (uint32_t)wl;
    return ((double)i < wl) ? (i + 1u) : i;
  }

  // Issue #68: readLoop() reads at pos + read_head_offset (forward, fixed)
  // so scatter writes at pos never contaminate the current sample's read.
  // The offset is direction-independent: forward OD reads the "not yet
  // scattered" region, reverse OD reads the "already fully scattered and
  // stable" region. Keeping the sign fixed avoids a ~32-sample jump at
  // is_reverse toggles. 16 samples = 0.33 ms @ 48 kHz, well below the
  // perceptual threshold; covers scatter spans up to 14 samples plus interp.
  constexpr static double read_head_offset = 16.0;

  // Issue #84: read-time loop-boundary crossfade. When boundary_xfade is set
  // (overdubbed loops), readLoop() spreads the buf[L-1]→buf[0] discontinuity
  // over boundary_xfade_len samples on each side of the wrap. Non-destructive
  // (the stored buffer is never modified for smoothing) so it cannot
  // accumulate across repeated OD start/stop. Length-preserving (the wrap
  // stays at loop_length).
  constexpr static uint32_t boundary_xfade_len = 64;
  bool boundary_xfade = false;
  // Frozen bridge endpoints (issue #84): during active OD the scatter overdubs
  // buf[0]/buf[L-1] while the read is inside the crossfade region, which would
  // step the bridge. readLoop() refreshes these from the (stable) edges while
  // OUTSIDE the region and uses the frozen values inside it.
  std::pair<float, float> _xfade_e0{};   // buf[0]
  std::pair<float, float> _xfade_eL{};   // buf[L-1]

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

  // Issue #84: enable the read-time boundary crossfade and seed the frozen
  // bridge endpoints from the current loop edges so the first boundary crossing
  // after enabling uses valid values.
  void enableBoundaryXfade() {
    boundary_xfade = true;
    // Issue #121: bridge the WINDOW seam (window start ↔ window end), not the
    // content end. Reduces to buf[0]/buf[L-1] when no sub-window is active. The
    // end edge is the last integer window sample (_winLenInt()-1).
    _xfade_e0 = *(buf + _winToAbsIdx(0));
    _xfade_eL = (winLen() > 0.0) ? *(buf + _winToAbsIdx(_winLenInt() - 1u))
                                 : std::pair<float, float>{0.0f, 0.0f};
  }

  // loop-relative position → absolute buffer index. Absolute content addressing
  // (loop_start≡0): used by the OD→alt copy, rec-tail smoothing, save/load and
  // the waveform thumbnail, which all operate on the full recording.
  IGB_FAST_INLINE size_t _toAbsIdx(uint32_t loop_rel) {
    return (loop_start + loop_rel) % buf_size;
  }

  // Issue #121: WINDOW-relative position [0,winLen()) → absolute buffer index,
  // wrapping within the content ring (% loop_length) so a window can straddle
  // the recording end. Used by the playback read and the overdub scatter. When
  // no sub-window is active (winStart()==0, winLen()==loop_length) this reduces
  // to _toAbsIdx(win_rel) exactly.
  IGB_FAST_INLINE size_t _winToAbsIdx(uint32_t win_rel) {
    size_t L = loop_length ? loop_length : 1;
    return _toAbsIdx((uint32_t)((winStart() + (size_t)win_rel) % L));
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
    // Issue #121: pos lives in the WINDOW ring; wrap at winLen() so the loop
    // (and the OD-lap wrap signal) lands on the window boundary. winLen()==
    // loop_length when no sub-window is active.
    double len = (double)winLen();
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
    // Issue #121: pos_u32 is WINDOW-relative; wrap by winLen() and map to
    // absolute content via _winToAbsIdx so the scatter stays inside the window.
    const uint32_t wl = _winLenInt();  // Issue #121: ceil(win_len) (fractional window)
    float gain = (float)tape_speed;
    if (pos_frac + tape_speed > 1.0) {
      // crosses integer boundary
      double rate = (1.0 - pos_frac) / tape_speed;
      // leading portion
      _writeFb(_winToAbsIdx(pos_u32), pos_u32,
               value.first * gain * (float)rate,
               value.second * gain * (float)rate,
               feedback, false);
      // trailing portion (update _last_fb_pos). Issue #67: wrap by winLen()
      // so the contribution lands on window pos 0 when we cross the boundary,
      // instead of writing past the window into other content.
      uint32_t next = (pos_u32 + 1) % wl;
      float rem = gain * (1.0f - (float)rate);
      _writeFb(_winToAbsIdx(next), next,
               value.first * rem, value.second * rem,
               feedback, true);
    } else if (pos_frac == 0.0) {
      // integer boundary
      _writeFb(_winToAbsIdx(pos_u32), pos_u32,
               value.first * gain, value.second * gain,
               feedback, true);
    } else {
      // mid-sample
      _writeFb(_winToAbsIdx(pos_u32), pos_u32,
               value.first * gain, value.second * gain,
               feedback, false);
    }
  }

  IGB_FAST_INLINE void _overdubFast(
      std::pair<float, float> value, float feedback,
      uint32_t pos_u32, double pos_frac) {
    // Issue #121: window-relative scatter (see _overdubSlow).
    const uint32_t wl = _winLenInt();  // Issue #121: ceil(win_len) (fractional window)
    // leading edge
    float lead = (pos_frac == 0.0) ? 1.0f : (1.0f - (float)pos_frac);
    _writeFb(_winToAbsIdx(pos_u32), pos_u32,
             value.first * lead, value.second * lead,
             feedback, false);

    // intermediate positions. Issue #67: span is computed on raw (unwrapped)
    // indices so `t` stays a clean fraction, but every `_writeFb` loop_pos is
    // wrapped by winLen() so contributions that straddle the boundary land
    // on window positions 0..N-1 instead of past-the-window slots.
    double next_pos = pos + tape_speed;
    uint32_t next_pos_raw = (uint32_t)next_pos;
    float next_pos_frac = (float)(next_pos - (double)next_pos_raw);

    float inv_span = 1.0f / (float)(next_pos_raw - pos_u32);
    for (uint32_t k = pos_u32 + 1; k < next_pos_raw; ++k) {
      uint32_t k_wrapped = k % wl;
      float t = (float)(k - pos_u32) * inv_span;
      auto interp = _deinterp(value, t);
      _writeFb(_winToAbsIdx(k_wrapped), k_wrapped,
               interp.first, interp.second,
               feedback, false);
    }

    // trailing edge (update _last_fb_pos with the wrapped window pos so the
    // next call's lead at window pos 0 correctly hits the REVISIT branch).
    uint32_t next_pos_u32 = next_pos_raw % wl;
    _writeFb(_winToAbsIdx(next_pos_u32), next_pos_u32,
             value.first * next_pos_frac, value.second * next_pos_frac,
             feedback, true);
  }

  // --- reverse scatter ---

  IGB_FAST_INLINE void _overdubSlowReverse(
      std::pair<float, float> value, float feedback,
      uint32_t pos_u32, double pos_frac, double abs_speed) {
    // Issue #121: window-relative backward scatter (see _overdubSlow).
    const uint32_t wl = _winLenInt();  // Issue #121: ceil(win_len) (fractional window)
    float gain = (float)abs_speed;
    if (pos_frac > 0.0 && pos_frac < abs_speed) {
      // crosses integer boundary (backward)
      float rate = (float)(pos_frac / abs_speed);
      // leading portion (current position)
      _writeFb(_winToAbsIdx(pos_u32), pos_u32,
               value.first * gain * rate,
               value.second * gain * rate,
               feedback, false);
      // trailing portion (previous position, update _last_fb_pos)
      uint32_t prev = (pos_u32 + wl - 1) % wl;
      float rem = gain * (1.0f - rate);
      _writeFb(_winToAbsIdx(prev), prev,
               value.first * rem, value.second * rem,
               feedback, true);
    } else if (pos_frac == 0.0) {
      // integer boundary: entering previous position
      uint32_t prev = (pos_u32 + wl - 1) % wl;
      _writeFb(_winToAbsIdx(prev), prev,
               value.first * gain, value.second * gain,
               feedback, true);
    } else {
      // mid-sample
      _writeFb(_winToAbsIdx(pos_u32), pos_u32,
               value.first * gain, value.second * gain,
               feedback, false);
    }
  }

  IGB_FAST_INLINE void _overdubFastReverse(
      std::pair<float, float> value, float feedback,
      uint32_t pos_u32, double pos_frac) {
    // Issue #121: window-relative backward scatter (see _overdubSlow).
    const uint32_t wl = _winLenInt();  // Issue #121: ceil(win_len) (fractional window)
    // leading edge (at current pos)
    float lead = (pos_frac == 0.0) ? 1.0f : (float)pos_frac;
    _writeFb(_winToAbsIdx(pos_u32), pos_u32,
             value.first * lead, value.second * lead,
             feedback, false);

    // backward destination (wrapped to positive)
    double dest = pos + tape_speed;
    if (dest < 0.0) dest += (double)wl;
    uint32_t dest_u32 = (uint32_t)dest;
    float dest_frac = (float)(dest - (double)dest_u32);

    // intermediate positions (backward)
    uint32_t span = (pos_u32 + wl - dest_u32) % wl;
    if (span > 1) {
      float inv_span = 1.0f / (float)span;
      for (uint32_t i = 1; i < span; ++i) {
        uint32_t k = (pos_u32 + wl - i) % wl;
        float t = (float)i * inv_span;
        auto interp = _deinterp(value, t);
        _writeFb(_winToAbsIdx(k), k,
                 interp.first, interp.second,
                 feedback, false);
      }
    }

    // trailing edge (update _last_fb_pos)
    float trailing = 1.0f - dest_frac;
    _writeFb(_winToAbsIdx(dest_u32), dest_u32,
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

  // read within loop at current pos (interpolated).
  // Issue #68: reads at pos + read_head_offset rather than pos directly, so
  // the auditory now-position is decoupled from the scatter write head. See
  // `read_head_offset` comment above for the direction and safety reasoning.
  IGB_FAST_INLINE std::pair<float, float> readLoop() {
    return readLoopAhead(0.0, true);
  }

  // Issue #111: io-rate (96 kHz) oversampled read. Reads at pos + `ahead`
  // (+ read_head_offset) WITHOUT advancing pos, so a loop frame can emit
  // `audio_oversample` sub-frame samples (ahead = k·speed/N) while pos still
  // advances by a single movePos() — keeping the io path bit-equivalent to the
  // 48 k path at loop-frame boundaries. `is_canonical` (true only for the
  // ahead==0 read) gates the boundary_xfade frozen-edge refresh: a sub-frame
  // read must not refresh the snapshots from a position the loop-frame
  // bookkeeping has not yet reached.
  IGB_FAST_INLINE std::pair<float, float> readLoopAhead(double ahead, bool is_canonical) {
    // Issue #121: read within the WINDOW ring. read_pos wraps at winLen() and
    // window-relative indices map to absolute content via _winToAbsIdx (which
    // wraps at loop_length, so a window straddling the recording end reads the
    // correct content). winLen()==loop_length when no sub-window is active.
    const double wl = winLen();
    double read_pos = pos + ahead + read_head_offset;
    if (read_pos >= wl) read_pos -= wl;
    else if (read_pos < 0.0) read_pos += wl;
    uint32_t pos_i = (uint32_t)read_pos;
    float t = (float)(read_pos - (double)pos_i);
    // Issue #121: the interp partner wraps at the (possibly FRACTIONAL) window
    // length — next == 0 once it would reach/exceed wl, so the seam interpolates
    // window-end → window-start without an integer-modulus off-by-one. For an
    // integer / full window this matches the prior (pos_i+1) % wl behavior.
    uint32_t next_i = pos_i + 1u;
    if ((double)next_i >= wl) next_i = 0u;
    size_t idx0 = _winToAbsIdx(pos_i);
    size_t idx1 = _winToAbsIdx(next_i);
    auto v0 = *(buf + idx0);
    auto v1 = *(buf + idx1);
    float out_l = (1.0f - t) * v0.first  + t * v1.first;
    float out_r = (1.0f - t) * v0.second + t * v1.second;

    // Issue #84: read-time loop-boundary crossfade (overdubbed loops). Within
    // boundary_xfade_len of the wrap on each side, blend the content toward a
    // linear bridge between buf[L-1] and buf[0] (weight w peaks at the wrap),
    // spreading the steep 1-sample jump over the window. At the wrap both
    // sides converge to 0.5*(buf[L-1]+buf[0]) → continuous; at ±N w=0 → normal.
    if (boundary_xfade && winLen() > 2u * boundary_xfade_len) {
      const float N = (float)boundary_xfade_len;
      double dist = -1.0;
      bool pre_wrap = false;
      if (read_pos >= wl - (double)N) {
        dist = wl - read_pos;                     // 0..N (approaching wrap)
        pre_wrap = true;
      } else if (read_pos < (double)N) {
        dist = read_pos;                          // 0..N (leaving wrap)
      }
      if (dist >= 0.0) {
        // Inside the region: bridge using the FROZEN edges (the live buf[0]/
        // buf[L-1] are overdubbed by the OD scatter while we are here, which
        // would step the bridge — issue #84 problem 2).
        float d = (float)dist / N;                // 0 at wrap, 1 at ±N
        // Smoothstep weight: 1 at the wrap, 0 at ±N, with ZERO slope at both
        // ends AND the peak. A linear (triangle) weight has a slope kink at its
        // peak — i.e. a corner in the waveform right at the loop boundary —
        // which is audible as a soft click proportional to the boundary
        // discontinuity (issue #84 problem 2). Smoothstep removes the kink.
        float w = 1.0f - d * d * (3.0f - 2.0f * d);  // 1 - smoothstep(d)
        float phi = pre_wrap ? (0.5f * (1.0f - d))   // 0 at L-N → 0.5 at wrap
                             : (0.5f + 0.5f * d);    // 0.5 at wrap → 1 at N
        float bridge_l = (1.0f - phi) * _xfade_eL.first  + phi * _xfade_e0.first;
        float bridge_r = (1.0f - phi) * _xfade_eL.second + phi * _xfade_e0.second;
        out_l = out_l * (1.0f - w) + bridge_l * w;
        out_r = out_r * (1.0f - w) + bridge_r * w;
      } else if (is_canonical) {
        // Outside the region the edges are not being overdubbed (the write head
        // touches them only at pos 0 / L-1, which map into the region), so they
        // are stable — refresh the snapshots for the next boundary crossing.
        // Issue #111: only the canonical (ahead==0) read refreshes; the io
        // sub-frame read reuses the snapshots so it cannot step the bridge.
        // Issue #121: window edges (reduces to buf[0]/buf[L-1] for a full
        // window); end edge is the last integer window sample (_winLenInt()-1).
        _xfade_e0 = *(buf + _winToAbsIdx(0));
        _xfade_eL = *(buf + _winToAbsIdx(_winLenInt() - 1u));
      }
    }
    return { out_l, out_r };
  }
};

}
