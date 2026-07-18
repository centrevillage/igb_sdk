#pragma once

#include <cstdint>
#include <cstddef>
#include <utility>
#include <igb_util/macro.hpp>
#include <igb_util/dsp/deinterpolator.hpp>
#include <igb_util/dsp/q32_pos.hpp>

namespace igb::dsp {

template<typename Deinterp = DeinterpNo>
struct LoopBufferStereo {
  std::pair<float, float> _dummy_buf = {0.0f, 0.0f};
  std::pair<float, float>* buf = nullptr;
  size_t buf_size = 1;

  size_t write_pos = 0;      // native-speed recording
  size_t loop_start = 0;     // absolute start of recorded region (LilaC: always 0)
  size_t loop_length = 0;    // length of recorded region (= recorded content ring)
  // Issue #198: window-relative playhead in Q32.32 (value = pos_q / 2^32,
  // range [0, winLen()), docs/198). Written by the audio IRQ (movePos /
  // overdub) with plain stores; MAIN-LOOP readers/writers must go through
  // q32_atomic_load/store or resetPosQ (docs/198 §3.2 — an int64 access may
  // compile to two 32-bit halves and the IRQ can land between them; the
  // bracket is the guarantee, alignas(8) only makes single LDRD/STRD likely).
  alignas(8) q32_t pos_q = 0;
  // Issue #198: playback rate, Q32.32 for position math + float mirror for
  // the scatter sample gains (samples are float — f64 gain math was never
  // load-bearing). Both derived in changeSpeed() (main-loop context).
  alignas(8) q32_t tape_speed_q = q32_one;
  float tape_speed_f = 1.0f;
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
  // These LIVE accessors are safe from ANY context (main loop UI/tick readers,
  // load workers, tests) — they read only the raw fields. The audio hot path
  // uses the _syncWin() cache below instead (see its comment for why the two
  // are kept separate).
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

  // Issue #171 (docs/169 F3): AUDIO-CONTEXT resolved-window cache. The live
  // accessors above re-derive the effective window (f64 compares + u32→f64
  // converts) on every call — pre-#171 that ran 4-6 times per interpolated
  // read on the audio IRQ hot path. The derived values change only when the
  // raw fields change (window commit / stutter / record stop / load / clear),
  // so the hot path calls _syncWin() once per operation (a 3-compare key
  // check; refresh only on mismatch) and then reads these fields directly.
  //
  // Single-writer rule: _syncWin() is called ONLY from the audio-IRQ-context
  // functions (readLoopAhead / movePos / overdub / LoopTrack::_advanceOdCopy).
  // Main-loop code must keep using the live accessors — that keeps this cache
  // written from exactly one context (the audio IRQ preempts the main loop and
  // is never preempted itself), so no cross-context torn/interleaved refresh
  // can exist by construction. The key check also makes a stale cache
  // impossible regardless of WHERE the raw fields were written — there is
  // deliberately no "must call refresh after mutating" contract (the raw
  // fields stay public and domain code / tests assign them freely; a missed
  // refresh call was judged the likeliest future accident).
  size_t _ck_wstart = SIZE_MAX;  // cache keys: last-seen raw fields
  double _ck_wlen = -1.0;        // (-1 unreachable → first _syncWin refreshes)
  size_t _ck_llen = SIZE_MAX;
  double _wl_d = 0.0;            // == winLen()
  q32_t _wl_q = 0;               // == q32_from_double(winLen())  (issue #198)
  uint32_t _wl_int = 0;          // == _winLenInt()
  size_t _wstart = 0;            // == winStart()
  bool _xfade_wide = false;      // == (winLen() > 2*boundary_xfade_len)

  IGB_FAST_INLINE void _syncWin() {
    if (win_start == _ck_wstart && win_len == _ck_wlen && loop_length == _ck_llen) {
      return;
    }
    _ck_wstart = win_start;
    _ck_wlen = win_len;
    _ck_llen = loop_length;
    _wl_d = winLen();
    // Issue #198: q32_from_double is hand-split (no __aeabi libcall) — this
    // refresh runs in the audio IRQ, where QSPI-resident newlib is forbidden
    // (docs/198 §4-7).
    _wl_q = q32_from_double(_wl_d);
    _wstart = winStart();
    _wl_int = _winLenInt();
    _xfade_wide = (_wl_d > (double)(2u * boundary_xfade_len));
  }

  // Issue #68: readLoop() reads at pos + read_head_offset (forward, fixed)
  // so scatter writes at pos never contaminate the current sample's read.
  // The offset is direction-independent: forward OD reads the "not yet
  // scattered" region, reverse OD reads the "already fully scattered and
  // stable" region. Keeping the sign fixed avoids a ~32-sample jump at
  // is_reverse toggles. 16 samples = 0.33 ms @ 48 kHz, well below the
  // perceptual threshold; covers scatter spans up to 14 samples plus interp.
  constexpr static q32_t read_head_offset_q = q32_t(16) << 32;  // == 16.0 (issue #198)

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

  // Issue #198: derive both rate representations. Called from the MAIN LOOP
  // every iteration (LooperModel::process → updateTapeSpeed) while the audio
  // IRQ reads tape_speed_q — the value-change guard drops the conversion to
  // effective tick rate (the PLL mod only changes per tick) and the atomic
  // store keeps the 64-bit publish untorn (docs/198 §3.2). The f/q pair can
  // be observed one frame apart by the IRQ (gain vs rate skew for a single
  // sample during a speed change) — same order-of-magnitude transient the
  // PLL ramp already produces.
  double _last_speed_in = 1.0;
  IGB_FAST_INLINE void changeSpeed(double speed) {
    if (speed == _last_speed_in) return;
    _last_speed_in = speed;
    tape_speed_f = (float)speed;
    q32_atomic_store(tape_speed_q, q32_from_double(speed));
  }

  void resetFeedbackPos() {
    _last_fb_pos = UINT32_MAX;
  }

  // Issue #84: enable the read-time boundary crossfade and seed the frozen
  // bridge endpoints from the current loop edges so the first boundary crossing
  // after enabling uses valid values. Main-loop context → live accessors.
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
  // Issue #171 (docs/169 F2): the % buf_size became a bounded-subtract loop —
  // the exact same modulus for every input (0 iterations for the common
  // in-content index; buf_size >= 1 always, so it terminates), minus the
  // udiv+mls pair on the audio hot path.
  IGB_FAST_INLINE size_t _toAbsIdx(uint32_t loop_rel) {
    size_t idx = loop_start + loop_rel;
    while (idx >= buf_size) idx -= buf_size;
    return idx;
  }

  // Issue #121: WINDOW-relative position [0,winLen()) → absolute buffer index,
  // wrapping within the content ring (% loop_length) so a window can straddle
  // the recording end. When no sub-window is active (winStart()==0, winLen()==
  // loop_length) this reduces to _toAbsIdx(win_rel) exactly. This public form
  // computes via the LIVE accessors (any-context safe; cold callers: boundary
  // xfade seeding, undo-window capture). The audio hot path uses _winIdx below.
  IGB_FAST_INLINE size_t _winToAbsIdx(uint32_t win_rel) {
    size_t L = loop_length ? loop_length : 1;
    return _toAbsIdx((uint32_t)((winStart() + (size_t)win_rel) % L));
  }

  // Post-_syncWin() hot-path body of _winToAbsIdx (audio context only: the
  // caller syncs once per operation, then indexes repeatedly). Issue #171
  // (docs/169 F2): both moduli are bounded-subtract loops — identical result
  // to ((winStart()+rel) % L) % buf_size for EVERY input (the #121 spec lets
  // pos run transiently past the window after a window change; the loops just
  // iterate more in that case, still exact), without the two udiv+mls pairs
  // per index (4 per interpolated read) on the audio IRQ.
  IGB_FAST_INLINE size_t _winIdx(uint32_t win_rel) const {
    const size_t L = loop_length ? loop_length : 1;
    size_t w = _wstart + (size_t)win_rel;
    while (w >= L) w -= L;
    size_t idx = loop_start + w;
    while (idx >= buf_size) idx -= buf_size;
    return idx;
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
  // don't need the wrap signal can ignore the return value. Audio context.
  IGB_FAST_INLINE bool movePos() {
    // Issue #121: pos lives in the WINDOW ring; wrap at winLen() so the loop
    // (and the OD-lap wrap signal) lands on the window boundary. winLen()==
    // loop_length when no sub-window is active.
    // Issue #198: Q32.32 advance — add + int64 compare, no f64 math. Audio
    // context: plain (non-bracketed) access to pos_q / tape_speed_q is
    // correct here, the audio IRQ is never preempted (docs/198 §3.2).
    _syncWin();
    bool wrapped = q32_advance_wrap(pos_q, tape_speed_q, _wl_q);
    pos_snapshot = q32_to_float(pos_q);
    return wrapped;
  }

  // Issue #198 §3.2: main-loop entry — atomic publish so the audio IRQ never
  // observes a torn 64-bit position. The reset-vs-advance ORDER race (the IRQ
  // may advance right before/after the reset) keeps the exact semantics the
  // double code had; only torn values are new-ly impossible.
  IGB_FAST_INLINE void resetPosQ(q32_t new_pos_q) {
    q32_atomic_store(pos_q, new_pos_q);
    pos_snapshot = q32_to_float(new_pos_q);
  }
  IGB_FAST_INLINE void resetPos(double new_pos = 0.0) {
    resetPosQ(q32_from_double(new_pos));
  }

  // --- native-speed recording ---

  IGB_FAST_INLINE void write(std::pair<float, float> value) {
    *(buf + write_pos) = value;
    // Issue #171: write_pos < buf_size is an invariant, so the old % buf_size
    // was a single conditional wrap paying a udiv per recorded sample.
    if (++write_pos >= buf_size) write_pos = 0;
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

  // Issue #210: positioned scatter write — the overdub() body with the write
  // anchor and speed made EXPLICIT and the pos advance left to the caller.
  // Deferred class-2 record writes capture {pos_q, tape_speed_q/f} at render
  // time (movePos still advances per frame) and replay here at the block
  // tail. Passing the CAPTURED speed preserves the #198 §4-2 invariant (the
  // scatter span always matches the advance the PAIRED movePos made — a
  // mid-block changeSpeed must not skew the replay). Replays must keep frame
  // order: the deinterp state and the _last_fb_pos first-touch bookkeeping
  // are sequential.
  IGB_FAST_INLINE void overdubAt(q32_t at_pos_q, q32_t speed_q, float speed_f,
                                 std::pair<float, float> value, float feedback) {
    _syncWin();   // Issue #171: one key check; the scatter uses _wl_int/_winIdx
    // Issue #198: all scatter POSITION math is integer Q32.32; only the
    // sample-gain arithmetic stays float (docs/198 §3). The extraction base
    // saturates a (degenerate-state) negative pos to 0, mirroring the old
    // `(uint32_t)pos_double` hardware saturation — see readLoopAhead. The
    // speed-class branches compare exact integers — same branch the f64
    // compares took for on-grid values, now branch-deterministic.
    const q32_t pos_qc = (at_pos_q < 0) ? 0 : at_pos_q;
    uint32_t pos_u32 = q32_idx(pos_qc);
    uint32_t pos_frac = (uint32_t)pos_qc;   // fractional word

    if (speed_q >= 0) {
      if (speed_q < q32_one) {
        _overdubSlow(value, feedback, pos_u32, pos_frac, speed_q, speed_f);
      } else {
        _overdubFast(value, feedback, pos_qc, pos_u32, pos_frac, speed_q);
      }
    } else {
      q32_t abs_speed_q = -speed_q;
      if (abs_speed_q < q32_one) {
        _overdubSlowReverse(value, feedback, pos_u32, pos_frac, abs_speed_q,
                            speed_f);
      } else {
        _overdubFastReverse(value, feedback, pos_qc, pos_u32, pos_frac, speed_q);
      }
    }

    _deinterp.update(value);
  }

  // Scatter-write `value` at current pos with feedback, advance pos, and
  // return true if the pos advance crossed the loop boundary. The wrap
  // signal lets callers detect OD one-lap-completed without an extra
  // prev_pos/new_pos fetch pair (LilaCRepeater issue #64). Audio context.
  // Issue #210: == overdubAt at the live pos/speed + the paired movePos.
  IGB_FAST_INLINE bool overdub(std::pair<float, float> value, float feedback) {
    overdubAt(pos_q, tape_speed_q, tape_speed_f, value, feedback);
    return movePos();
  }

  // Issue #171: scatter modulus (post-_syncWin). ceil(win_len) like
  // _winLenInt(), floored to 1 so the wrap loops below terminate even in the
  // (guarded-out by callers) empty state — the old `% 0` was UB there anyway.
  IGB_FAST_INLINE uint32_t _scatterLen() const {
    return _wl_int ? _wl_int : 1u;
  }

  // --- forward scatter ---

  IGB_FAST_INLINE void _overdubSlow(
      std::pair<float, float> value, float feedback,
      uint32_t pos_u32, uint32_t pos_frac, q32_t speed_q, float speed_f) {
    // Issue #121: pos_u32 is WINDOW-relative; wrap by winLen() and map to
    // absolute content via _winIdx so the scatter stays inside the window.
    // Issue #198: pos_frac is the Q32.32 fractional word; the boundary test
    // is exact integer math (was `pos_frac + tape_speed > 1.0` in f64).
    // Issue #210: speed is a parameter (deferred replays pass the captured
    // value; the live overdub() passes the members).
    const uint32_t wl = _scatterLen();
    float gain = speed_f;
    if ((q32_t)pos_frac + speed_q > q32_one) {
      // crosses integer boundary. pos_frac > 0 here (frac 0 + slow speed
      // can't cross), so q32_one - pos_frac < 2^32 fits the frac word.
      float rate = q32_frac_to_float(q32_one - (q32_t)pos_frac) / speed_f;
      // leading portion
      _writeFb(_winIdx(pos_u32), pos_u32,
               value.first * gain * rate,
               value.second * gain * rate,
               feedback, false);
      // trailing portion (update _last_fb_pos). Issue #67: wrap by winLen()
      // so the contribution lands on window pos 0 when we cross the boundary,
      // instead of writing past the window into other content.
      uint32_t next = pos_u32 + 1;
      while (next >= wl) next -= wl;
      float rem = gain * (1.0f - rate);
      _writeFb(_winIdx(next), next,
               value.first * rem, value.second * rem,
               feedback, true);
    } else if (pos_frac == 0u) {
      // integer boundary
      _writeFb(_winIdx(pos_u32), pos_u32,
               value.first * gain, value.second * gain,
               feedback, true);
    } else {
      // mid-sample
      _writeFb(_winIdx(pos_u32), pos_u32,
               value.first * gain, value.second * gain,
               feedback, false);
    }
  }

  IGB_FAST_INLINE void _overdubFast(
      std::pair<float, float> value, float feedback,
      q32_t pos_qc, uint32_t pos_u32, uint32_t pos_frac, q32_t speed_q) {
    // Issue #121: window-relative scatter (see _overdubSlow).
    const uint32_t wl = _scatterLen();
    // leading edge
    float lead = (pos_frac == 0u)
        ? 1.0f : (1.0f - q32_frac_to_float((q32_t)pos_frac));
    _writeFb(_winIdx(pos_u32), pos_u32,
             value.first * lead, value.second * lead,
             feedback, false);

    // intermediate positions. Issue #67: span is computed on raw (unwrapped)
    // indices so `t` stays a clean fraction, but every `_writeFb` loop_pos is
    // wrapped by winLen() so contributions that straddle the boundary land
    // on window positions 0..N-1 instead of past-the-window slots.
    // Issue #171: the wrap is carried incrementally (k advances by 1 per
    // iteration) instead of paying a `% wl` per position.
    // Issue #198: next_pos_q = pos + speed_q is the IDENTICAL increment
    // the PAIRED movePos() applies — the scatter destination and the
    // advanced pos can never diverge (docs/198 §4-2; #210: deferred replays
    // pass the captured speed for exactly this reason). Based on the
    // SATURATED pos_qc so a degenerate negative pos can't shift a huge index
    // into the span math (matches the old per-cast hardware saturation).
    q32_t next_pos_q = pos_qc + speed_q;        // unwrapped, >= 0 (forward)
    uint32_t next_pos_raw = q32_idx(next_pos_q);
    float next_pos_frac = q32_frac_to_float(next_pos_q);

    float inv_span = 1.0f / (float)(next_pos_raw - pos_u32);
    uint32_t kw = pos_u32 + 1;
    while (kw >= wl) kw -= wl;
    for (uint32_t k = pos_u32 + 1; k < next_pos_raw; ++k) {
      float t = (float)(k - pos_u32) * inv_span;
      auto interp = _deinterp(value, t);
      _writeFb(_winIdx(kw), kw,
               interp.first, interp.second,
               feedback, false);
      if (++kw >= wl) kw -= wl;
    }

    // trailing edge (update _last_fb_pos with the wrapped window pos so the
    // next call's lead at window pos 0 correctly hits the REVISIT branch).
    uint32_t next_pos_u32 = next_pos_raw;
    while (next_pos_u32 >= wl) next_pos_u32 -= wl;
    _writeFb(_winIdx(next_pos_u32), next_pos_u32,
             value.first * next_pos_frac, value.second * next_pos_frac,
             feedback, true);
  }

  // --- reverse scatter ---

  IGB_FAST_INLINE void _overdubSlowReverse(
      std::pair<float, float> value, float feedback,
      uint32_t pos_u32, uint32_t pos_frac, q32_t abs_speed_q, float speed_f) {
    // Issue #121: window-relative backward scatter (see _overdubSlow).
    // Issue #198: boundary test in exact integer Q32.32; gains in float
    // (abs_speed_f == the old (float)abs_speed exactly — float negate).
    const uint32_t wl = _scatterLen();
    const float abs_speed_f = -speed_f;
    float gain = abs_speed_f;
    if (pos_frac != 0u && (q32_t)pos_frac < abs_speed_q) {
      // crosses integer boundary (backward)
      float rate = q32_frac_to_float((q32_t)pos_frac) / abs_speed_f;
      // leading portion (current position)
      _writeFb(_winIdx(pos_u32), pos_u32,
               value.first * gain * rate,
               value.second * gain * rate,
               feedback, false);
      // trailing portion (previous position, update _last_fb_pos)
      uint32_t prev = pos_u32 ? (pos_u32 - 1) : (wl - 1);
      while (prev >= wl) prev -= wl;
      float rem = gain * (1.0f - rate);
      _writeFb(_winIdx(prev), prev,
               value.first * rem, value.second * rem,
               feedback, true);
    } else if (pos_frac == 0u) {
      // integer boundary: entering previous position
      uint32_t prev = pos_u32 ? (pos_u32 - 1) : (wl - 1);
      while (prev >= wl) prev -= wl;
      _writeFb(_winIdx(prev), prev,
               value.first * gain, value.second * gain,
               feedback, true);
    } else {
      // mid-sample
      _writeFb(_winIdx(pos_u32), pos_u32,
               value.first * gain, value.second * gain,
               feedback, false);
    }
  }

  IGB_FAST_INLINE void _overdubFastReverse(
      std::pair<float, float> value, float feedback,
      q32_t pos_qc, uint32_t pos_u32, uint32_t pos_frac, q32_t speed_q) {
    // Issue #121: window-relative backward scatter (see _overdubSlow).
    const uint32_t wl = _scatterLen();
    // leading edge (at current pos)
    float lead = (pos_frac == 0u) ? 1.0f : q32_frac_to_float((q32_t)pos_frac);
    _writeFb(_winIdx(pos_u32), pos_u32,
             value.first * lead, value.second * lead,
             feedback, false);

    // backward destination (wrapped to positive). Issue #198 (docs/198 §8):
    // the backward dest wraps by the INTEGER scatter modulus wl
    // (== _scatterLen()), NOT the fractional _wl_q — preserved exactly from
    // the double code. The defensive clamp guards the degenerate tiny-window
    // case (wl < |speed|, dest still negative after one wrap): the double
    // path hit UB there ((uint32_t) of a negative double); a negative dest_q
    // would arithmetic-shift to a huge index and stall _winIdx's bounded-
    // subtract loop inside the audio IRQ, so clamp to 0 instead.
    q32_t dest_q = pos_qc + speed_q;
    if (dest_q < 0) dest_q += (q32_t)wl << 32;
    if (dest_q < 0) dest_q = 0;
    uint32_t dest_u32 = q32_idx(dest_q);
    float dest_frac = q32_frac_to_float(dest_q);

    // intermediate positions (backward). Issue #171: the wrap is carried by a
    // decrementing counter instead of a `% wl` per position.
    uint32_t span = pos_u32 + wl - dest_u32;
    while (span >= wl) span -= wl;
    if (span > 1) {
      float inv_span = 1.0f / (float)span;
      uint32_t kd = pos_u32;
      while (kd >= wl) kd -= wl;
      for (uint32_t i = 1; i < span; ++i) {
        kd = kd ? (kd - 1) : (wl - 1);
        float t = (float)i * inv_span;
        auto interp = _deinterp(value, t);
        _writeFb(_winIdx(kd), kd,
                 interp.first, interp.second,
                 feedback, false);
      }
    }

    // trailing edge (update _last_fb_pos)
    float trailing = 1.0f - dest_frac;
    _writeFb(_winIdx(dest_u32), dest_u32,
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

  // --- issue #198 Layer 2 (docs/198 §9.2): Q32.32 interp-read core ---------
  // Post-_syncWin index math only (no fetch): the tap pair for a wrapped
  // window-relative Q32.32 read position. Thin wrapper over the Layer 1 pure
  // function (proven against the frozen double reference in Phase 0).
  IGB_FAST_INLINE InterpTapsQ _interpTapsQ(q32_t read_pos_q) const {
    return q32_interp_taps(read_pos_q, _wl_q);
  }

  // Post-_syncWin interp read at an arbitrary wrapped window-relative
  // position: taps → _winIdx ×2 → 2-tap lerp. No boundary xfade, no pos
  // dependence, no state change — the grain read core (Phase 2 granular
  // engine calls this per tap after one _syncWin per frame, the same
  // convention _advanceOdCopy uses).
  IGB_FAST_INLINE std::pair<float, float> _readInterpQ(q32_t read_pos_q) const {
    InterpTapsQ taps = q32_interp_taps(read_pos_q, _wl_q);
    auto v0 = *(buf + _winIdx(taps.i0));
    auto v1 = *(buf + _winIdx(taps.i1));
    return {
      (1.0f - taps.t) * v0.first  + taps.t * v1.first,
      (1.0f - taps.t) * v0.second + taps.t * v1.second
    };
  }

  // One-shot any-position read (audio context): sync + wrap + core. Grain
  // entry / test convenience. Negative-saturating like readLoopAhead.
  IGB_FAST_INLINE std::pair<float, float> readAtQ(q32_t pos_q_any) {
    _syncWin();
    q32_t p = q32_wrap_once(pos_q_any, _wl_q);
    if (p < 0) p = 0;
    return _readInterpQ(p);
  }

  // Post-_syncWin effective window length in Q32.32 (grain wrap modulus).
  IGB_FAST_INLINE q32_t winLenQ() const { return _wl_q; }

  // read within loop at current pos (interpolated).
  // Issue #68: reads at pos + read_head_offset rather than pos directly, so
  // the auditory now-position is decoupled from the scatter write head. See
  // `read_head_offset_q` comment above for the direction and safety reasoning.
  IGB_FAST_INLINE std::pair<float, float> readLoop() {
    return readLoopAhead(0, true);
  }

  // Issue #111: io-rate (96 kHz) oversampled read. Reads at pos + `ahead`
  // (+ read_head_offset) WITHOUT advancing pos, so a loop frame can emit
  // `audio_oversample` sub-frame samples (ahead = k·speed/N) while pos still
  // advances by a single movePos() — keeping the io path bit-equivalent to the
  // 48 k path at loop-frame boundaries. `is_canonical` (true only for the
  // ahead==0 read) gates the boundary_xfade frozen-edge refresh: a sub-frame
  // read must not refresh the snapshots from a position the loop-frame
  // bookkeeping has not yet reached. Audio context.
  IGB_FAST_INLINE std::pair<float, float> readLoopAhead(q32_t ahead_q, bool is_canonical) {
    // Issue #121: read within the WINDOW ring. read_pos wraps at winLen() and
    // window-relative indices map to absolute content via _winIdx (which
    // wraps at loop_length, so a window straddling the recording end reads the
    // correct content). winLen()==loop_length when no sub-window is active.
    // Issue #171: one _syncWin() up front; the body reads the cached fields.
    // Issue #198: pos + ahead + offset → wrap → _readInterpQ core → xfade
    // blend. The interp-partner fractional-window seam (#121) lives inside
    // q32_interp_taps; xfade / frozen edges / canonical refresh stay OUT of
    // the core (playhead-seam-specific, docs/198 §9.2).
    _syncWin();
    const q32_t wl_q = _wl_q;
    q32_t read_pos_q = q32_wrap_once(pos_q + ahead_q + read_head_offset_q, wl_q);
    // Issue #198: saturate a (degenerate-state) negative read position to 0,
    // mirroring what the old `(uint32_t)read_pos_double` did in hardware
    // (vcvt/FCVTZU saturate negatives to 0 — UB in C++ but the de-facto
    // behavior this code shipped with). Reachable only when pos_q has gone
    // negative in the loop_length==0-while-playing degenerate state; without
    // the clamp the huge idx would spin _winIdx's bounded-subtract loop
    // INSIDE THE AUDIO IRQ (watchdog reset on device, test hang on host).
    if (read_pos_q < 0) read_pos_q = 0;
    auto out = _readInterpQ(read_pos_q);
    float out_l = out.first;
    float out_r = out.second;

    // Issue #84: read-time loop-boundary crossfade (overdubbed loops). Within
    // boundary_xfade_len of the wrap on each side, blend the content toward a
    // linear bridge between buf[L-1] and buf[0] (weight w peaks at the wrap),
    // spreading the steep 1-sample jump over the window. At the wrap both
    // sides converge to 0.5*(buf[L-1]+buf[0]) → continuous; at ±N w=0 → normal.
    if (boundary_xfade && _xfade_wide) {
      constexpr q32_t N_q = (q32_t)boundary_xfade_len << 32;
      const float N = (float)boundary_xfade_len;
      q32_t dist_q = -1;
      bool pre_wrap = false;
      if (read_pos_q >= wl_q - N_q) {
        dist_q = wl_q - read_pos_q;               // 0..N (approaching wrap)
        pre_wrap = true;
      } else if (read_pos_q < N_q) {
        dist_q = read_pos_q;                      // 0..N (leaving wrap)
      }
      if (dist_q >= 0) {
        // Inside the region: bridge using the FROZEN edges (the live buf[0]/
        // buf[L-1] are overdubbed by the OD scatter while we are here, which
        // would step the bridge — issue #84 problem 2).
        // Issue #198: dist_q <= 64·2^32 exceeds the u32 frac word, so the
        // float conversion goes through the hi/lo-split q32_to_float (§8).
        float d = q32_to_float(dist_q) / N;       // 0 at wrap, 1 at ±N
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
        _xfade_e0 = *(buf + _winIdx(0));
        _xfade_eL = *(buf + _winIdx(_wl_int - 1u));
      }
    }
    return { out_l, out_r };
  }
};

}
