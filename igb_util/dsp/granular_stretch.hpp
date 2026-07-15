#pragma once

#include <cstdint>
#include <utility>
#include <igb_util/macro.hpp>
#include <igb_util/dsp/q32_pos.hpp>

namespace igb::dsp {

// LilaC issue #200 (Phase B): granular time-stretch renderer.
//
// Synthesis-hop granular with a constant TWO overlapping grains at 50%
// overlap under a Hann window — which is mathematically an equal-sum
// crossfade between the outgoing and incoming grain, so it is implemented as
// exactly that: one envelope fetch + two interp reads + a lerp per sample
// (docs/200_granular_engine_design.md §1).
//
// Division of labor (docs/198 §9.3):
//   - transport: the OWNING code advances buf.pos_q via movePos() at the
//     transport rate (tempo ratio + PLL mod, reverse-signed). This engine
//     never advances the transport; call renderIo() BEFORE movePos(), the
//     same convention as readLoop()/readLoopAhead().
//   - grains: window-relative read heads owned here, reading through the
//     state-free interp core buf._readInterpQ() after one _syncWin() per
//     frame. Grain rate = the user pitch (±2^(st/12)), captured at spawn so
//     a pitch edit lands click-free on the next grain (LilaC Q9).
//
// Phase alignment (design §3): a grain spawned at output time t0 has its
// Hann centroid at t0 + L/2 reading anchor + p·L/2, while the transport sits
// at pos(t0) + r·L/2. anchor = pos + (r − p)·L/2 makes the perceived source
// position track pos exactly — at r == p the engine degenerates to a sample-
// exact passthrough of the direct read (the host-test invariant).
template <typename LoopBuf>
struct GranularStretch {
  // Tunable (design §8): grain length L in 48k frames; hop = L/2. Even, and
  // small against the minimum play window (1 step) so the spawn lead stays
  // a fraction of the ring.
  constexpr static uint32_t default_grain_len = 1024;   // ~21.3 ms @48k

  struct Grain {
    q32_t src = 0;      // window-relative read pos (Q32.32)
    q32_t rate = 0;     // per 48k frame, signed (reverse reads backwards)
    bool active = false;
  };

  Grain _out_g;          // fading out (weight 1−e)
  Grain _in_g;           // fading in  (weight e)
  uint32_t _k = 0;       // 0..hop−1: crossfade phase
  uint32_t _hop = default_grain_len / 2;
  float _env_step = 256.0f / (float)(default_grain_len / 2);
  // Next spawn's rate. Written by the main loop (setPitch), read by the audio
  // context (_spawn) — a 64-bit cross-context pair, so the writer goes
  // through q32_atomic_store (#198 §3.2 PRIMASK discipline; plain read on
  // the IRQ side).
  alignas(8) q32_t _pitch_q = q32_one;

  // Edit-time (main loop): grains in flight keep their captured rate — a
  // pitch change lands click-free on the next spawn (LilaC Q9).
  void setPitch(q32_t p) { q32_atomic_store(_pitch_q, p); }

  // --- WSOLA alignment search (design §9) ------------------------------------
  // Plain OLA splices grains at arbitrary phase: the two live grains read
  // positions offset by a constant (r−p)·hop, which on real hardware (PLL-
  // wobbled r) is a few samples even at pitch ±0 — a hop-rate-modulated
  // 2-tap comb ("gritty" broadband noise, worst near the original pitch) —
  // and periodic material splices out of phase every hop (hop-rate sidebands
  // on every harmonic). The fix is the "S" of WSOLA: pick the spawn anchor
  // within ±wsola_half_range that best continues what the outgoing grain is
  // about to play.
  //
  // Budget discipline (v2, after the first device spike): SDRAM reads are
  // the scarce resource — one 125 µs audio IRQ carries SIX loop frames × four
  // tracks, so even "a few reads per frame" multiplies by 24. The candidate
  // region is therefore copied ONCE into the SRAM _scratch (a handful of
  // SDRAM reads per frame), and the AMDF scan runs entirely on the scratch
  // (cheap core-local float ops, also micro-budgeted). Two-stage scan
  // (coarse stride-2, then a ±2 stride-1 refine) keeps the op count small at
  // 1-sample final resolution. Stretch tracks ban OD, so the buffer content
  // is immutable during playback and the look-ahead copy is race-free (a
  // reseed cancels the search).
  //
  // Acceptance rule: the aligned anchor is used only when its AMDF beats the
  // nominal anchor's by wsola_accept_num — periodic/near-passthrough content
  // aligns (the artifact cases), while aperiodic material (where the edge of
  // the window would win a meaningless linear race, e.g. ramps) falls back
  // to the phase-exact nominal anchor. A mild multiplicative center bias
  // breaks periodic ties toward the nominal.
  bool wsola_enabled = true;                          // device A/B switch
  constexpr static uint32_t wsola_half_range = 96;    // ±samples (~2 ms)
  constexpr static uint32_t wsola_coarse_step = 2;    // coarse lag stride
  constexpr static uint32_t wsola_fine_reach = 2;     // refine best ±2 @ stride 1
  constexpr static uint32_t wsola_taps = 16;          // template length
  constexpr static float wsola_accept_num = 0.5f;     // best < 0.5 × nominal
  constexpr static float wsola_center_bias = 0.001f;  // per-sample tie penalty
  // Per-frame micro-budgets, per track (scaled up for tiny hops).
  // LilaC issue #208 (perf audit C1): 4/8 → 3/6 — the SAME total search work
  // spread over more frames (results identical, so quality-invariant; only
  // the per-IRQ worst-case slice shrinks). At the standard hop 512 the plan
  // still fits at scale 1: frames_at_1x 300 → 398, search_lead 308 → 406
  // ≤ hop. Tiny hops compress via _recalcSearchPlan exactly as before.
  constexpr static uint32_t wsola_fill_per_frame = 3;   // SDRAM reads
  constexpr static uint32_t wsola_taps_per_frame = 6;   // scratch AMDF taps
  // Scratch spans the candidate walk in BOTH directions (reverse grains read
  // backwards): taps·|p|max(4) margin on each side of the 2W lag range.
  constexpr static uint32_t wsola_scratch_len =
      2 * wsola_half_range + 2 * wsola_taps * 4 + 16;   // = 336

  enum class SearchPhase : uint8_t { idle, fill, ref_capture, coarse, fine, ready };
  SearchPhase _sphase = SearchPhase::idle;
  float _scratch[wsola_scratch_len];   // mono (L+R) candidate region, SRAM
  float _ref[wsola_taps];              // outgoing grain's predicted continuation
  q32_t _scratch_base = 0;   // window-relative pos of _scratch[0] (integer q32)
  q32_t _cand0_off = 0;      // lag-0 candidate offset inside the scratch (q32)
  q32_t _fill_cursor = 0;
  q32_t _ref_cursor = 0;
  q32_t _search_rate = 0;    // p frozen at search start
  uint32_t _fill_idx = 0;
  uint32_t _ref_idx = 0;
  uint32_t _lag = 0;         // current lag (samples, 0..2W)
  uint32_t _fine_end = 0;
  uint32_t _tap_idx = 0;     // partial-lag resume point
  float _amdf_acc = 0.0f;
  // Schedule state — the defaults MUST equal what _recalcSearchPlan derives
  // for the default hop, so they come from the same _plan* maths below
  // (LilaC issue #208 C1: a stale hand-set lead default silently starved the
  // search when the per-frame budgets shrank — the engine runs on these
  // defaults in production, setGrainLen is a tuning-only entry).
  uint32_t _fill_pf = wsola_fill_per_frame * _planScale(default_grain_len / 2);
  uint32_t _taps_pf = wsola_taps_per_frame * _planScale(default_grain_len / 2);
  uint32_t _search_lead = _planLead(default_grain_len / 2);
  float _best_metric = 0.0f;
  float _center_metric = 0.0f;
  uint32_t _best_lag = 0;

  // Tuning entry (spike/listening): resets the render state.
  void setGrainLen(uint32_t len) {
    if (len < 128) len = 128;
    len &= ~1u;                        // even → exact 50% hop
    _hop = len / 2;
    _env_step = 256.0f / (float)_hop;
    _recalcSearchPlan();
    reset();
  }
  uint32_t grainLen() const { return _hop * 2; }

  // Fit the amortized schedule into the frames before each hop; tiny hops
  // compress it by raising the per-frame budgets proportionally. constexpr
  // so the member defaults above are derived from the SAME maths (they are
  // the production values — setGrainLen/_recalcSearchPlan is tuning-only).
  constexpr static uint32_t _planFrames1x() {
    const uint32_t coarse_lags = wsola_half_range / wsola_coarse_step * 2 + 1;
    const uint32_t fine_lags = 2 * wsola_fine_reach + 1;
    return (wsola_scratch_len + wsola_fill_per_frame - 1) / wsola_fill_per_frame
         + (wsola_taps + wsola_fill_per_frame - 1) / wsola_fill_per_frame
         + ((coarse_lags + fine_lags) * wsola_taps + wsola_taps_per_frame - 1)
               / wsola_taps_per_frame
         + 8;
  }
  constexpr static uint32_t _planScale(uint32_t hop) {
    const uint32_t budget = (hop > 24) ? (hop - 16) : 8;
    return (_planFrames1x() + budget - 1) / budget;
  }
  constexpr static uint32_t _planLead(uint32_t hop) {
    const uint32_t lead = _planFrames1x() / _planScale(hop) + 8;
    return (lead > hop) ? hop : lead;
  }

  void _recalcSearchPlan() {
    const uint32_t scale = _planScale(_hop);
    _fill_pf = wsola_fill_per_frame * scale;
    _taps_pf = wsola_taps_per_frame * scale;
    _search_lead = _planLead(_hop);
  }

  // Reseed hook (docs/v2_timestretch §5): pos discontinuities (undo swap,
  // step-jump, stutter repin, window commit, loopset swap, …) drop the
  // in-flight grains; the next render spawns fresh and Hann-ramps in
  // (~hop/48k s) — click-free by construction. Cancels any in-flight
  // alignment search (its predictions are stale).
  void reset() {
    _out_g.active = false;
    _in_g.active = false;
    _k = 0;
    _sphase = SearchPhase::idle;
  }

  // One loop-frame render: emits the io-rate pair (canonical + half-step
  // sub-frame, issue #111 structure — state advances ONCE per call). Reads
  // buf.pos_q / buf.tape_speed_q directly (audio context, plain access).
  IGB_FAST_INLINE void renderIo(LoopBuf& buf, std::pair<float, float>* out2) {
    buf._syncWin();
    const q32_t wl = buf.winLenQ();
    if (wl <= 0) {
      out2[0] = {0.0f, 0.0f};
      out2[1] = {0.0f, 0.0f};
      return;
    }
    // Spawn/rotate at the FRAME HEAD so a fresh grain's first read happens
    // at the same pos its anchor was derived from — spawning at the frame
    // tail would lag the incoming grain by one transport step (a small but
    // measurable phase error the passthrough test catches).
    if (!_in_g.active) {
      _spawn(buf, wl);                    // first grain after reset()
    } else if (_k >= _hop) {              // hop: incoming becomes outgoing
      _out_g = _in_g;
      _spawn(buf, wl);
      _k = 0;
    }

    const float e0 = _env((float)_k * _env_step);
    const float e1 = _env(((float)_k + 0.5f) * _env_step);
    const auto o0 = _readG(buf, wl, _out_g, 0);
    const auto o1 = _readG(buf, wl, _out_g, 1);
    const auto i0 = _readG(buf, wl, _in_g, 0);
    const auto i1 = _readG(buf, wl, _in_g, 1);
    out2[0] = { (1.0f - e0) * o0.first  + e0 * i0.first,
                (1.0f - e0) * o0.second + e0 * i0.second };
    out2[1] = { (1.0f - e1) * o1.first  + e1 * i1.first,
                (1.0f - e1) * o1.second + e1 * i1.second };

    if (_out_g.active) _out_g.src = q32_wrap_once(_out_g.src + _out_g.rate, wl);
    _in_g.src = q32_wrap_once(_in_g.src + _in_g.rate, wl);
    ++_k;                                 // hop handled at the next frame head

    // Amortized WSOLA scan for the NEXT spawn (a few lags per frame; see the
    // member-block comment). Runs after the render so its SDRAM traffic sits
    // in the same budget slot every frame.
    if (wsola_enabled && _in_g.active && _k < _hop) _searchAdvance(buf, wl);
  }

  // --- internals -----------------------------------------------------------

  IGB_FAST_INLINE void _spawn(LoopBuf& buf, q32_t wl) {
    // anchor = pos + (r − p)·L/2 (design §3). |r−p|·L/2 can exceed one
    // window on extreme rate deltas × short windows, so wrap is a bounded
    // loop here (q32_wrap_once is a ±1-window helper).
    const q32_t nominal =
        buf.pos_q + (buf.tape_speed_q - _pitch_q) * (q32_t)_hop;
    q32_t anchor = nominal;
    if (_sphase == SearchPhase::ready && _search_rate == _pitch_q) {
      // Acceptance rule (member-block comment): only a decisively better
      // splice replaces the phase-exact nominal anchor.
      if (_best_metric < wsola_accept_num * _center_metric) {
        anchor = _scratch_base + _cand0_off
               + ((q32_t)(_best_lag) << 32)
               - ((q32_t)wsola_half_range << 32);
      }
    }
    _sphase = SearchPhase::idle;   // consume; the next cycle re-arms
    _in_g.src = _wrapBounded(anchor, wl);
    _in_g.rate = _pitch_q;
    _in_g.active = true;
  }

  // Alignment-tap read from the loop buffer: nearest-sample mono sum (L+R).
  // Interp is unnecessary for AMDF alignment; the ±0.5-sample quantization
  // is far below the comb offsets being corrected.
  IGB_FAST_INLINE float _tapAt(const LoopBuf& buf, q32_t p, q32_t wl) const {
    q32_t w = _wrapBounded(p, wl);
    if (w < 0) w = 0;
    const auto v = *(buf.buf + buf._winIdx(q32_idx(w)));
    return v.first + v.second;
  }

  // One AMDF lag evaluated against the SRAM scratch, resumable mid-lag via
  // _tap_idx (the per-frame budget can be smaller than one lag). Candidate
  // tap positions are scratch-relative: _cand0_off + (lag − W) + k·p.
  IGB_FAST_INLINE bool _lagStep(uint32_t budget_taps) {
    const q32_t base = _cand0_off
        + (((q32_t)_lag - (q32_t)wsola_half_range) << 32);
    q32_t cp = base + (q32_t)_tap_idx * _search_rate;
    uint32_t done = 0;
    while (_tap_idx < wsola_taps && done < budget_taps) {
      const uint32_t idx = (uint32_t)(cp >> 32);
      const float d =
          ((idx < wsola_scratch_len) ? _scratch[idx] : 0.0f) - _ref[_tap_idx];
      _amdf_acc += (d < 0.0f) ? -d : d;
      cp += _search_rate;
      ++_tap_idx;
      ++done;
    }
    if (_tap_idx < wsola_taps) return false;   // resume next frame
    // Lag complete: score with the scale-free center bias.
    const float off = (_lag > wsola_half_range)
                          ? (float)(_lag - wsola_half_range)
                          : (float)(wsola_half_range - _lag);
    const float metric = _amdf_acc * (1.0f + wsola_center_bias * off);
    if (_lag == wsola_half_range) _center_metric = _amdf_acc;
    if (metric < _best_metric) {
      _best_metric = metric;
      _best_lag = _lag;
    }
    _amdf_acc = 0.0f;
    _tap_idx = 0;
    return true;
  }

  IGB_FAST_INLINE void _searchAdvance(LoopBuf& buf, q32_t wl) {
    switch (_sphase) {
      case SearchPhase::idle: {
        if (_hop - _k > _search_lead) return;
        // Degenerate windows (shorter than the candidate span) skip the
        // search — real play windows are ≥ one step (thousands of samples).
        if ((uint32_t)(wl >> 32) < 2048) return;
        // Freeze the plan: predict where the outgoing grain (= the current
        // incoming one) and the nominal anchor will be at the hop. Content
        // is immutable during stretch playback (OD is gated off), so the
        // look-ahead reads stay valid; r may drift a hair over ≤10 ms, which
        // only biases the window the accepted lag was searched in.
        const q32_t remain = (q32_t)(_hop - _k);
        const q32_t r = buf.tape_speed_q;
        _search_rate = _pitch_q;
        // Timing: _in_g.src is post-advance for THIS frame while pos_q is
        // pre-movePos, and the spawn-frame renderIo runs before that frame's
        // movePos — so the grain advances `remain` more times but pos
        // advances `remain + 1` times before the spawn reads them.
        _ref_cursor = _in_g.src + remain * _search_rate;
        const q32_t anchor_pred =
            buf.pos_q + (remain + 1) * r + (r - _search_rate) * (q32_t)_hop;
        // Scratch covers [anchor_pred − W − taps·4, … + W + taps·4]: the
        // integer-floored base keeps candidate flooring identical to the
        // direct-read path (passthrough AMDF must stay exactly 0).
        const q32_t lo = anchor_pred
            - ((q32_t)(wsola_half_range + wsola_taps * 4) << 32);
        _scratch_base = (q32_t)((uint64_t)lo & ~0xFFFFFFFFull);
        _cand0_off = anchor_pred - _scratch_base;   // ≥ 0 by construction
        _fill_cursor = _scratch_base;
        _fill_idx = 0;
        _ref_idx = 0;
        _lag = 0;
        _tap_idx = 0;
        _amdf_acc = 0.0f;
        _best_metric = 3.4e38f;
        _center_metric = 0.0f;
        _best_lag = wsola_half_range;
        _sphase = SearchPhase::fill;
        return;
      }
      case SearchPhase::fill: {
        for (uint32_t n = 0; n < _fill_pf && _fill_idx < wsola_scratch_len; ++n) {
          _scratch[_fill_idx++] = _tapAt(buf, _fill_cursor, wl);
          _fill_cursor += q32_one;
        }
        if (_fill_idx >= wsola_scratch_len) _sphase = SearchPhase::ref_capture;
        return;
      }
      case SearchPhase::ref_capture: {
        for (uint32_t n = 0; n < _fill_pf && _ref_idx < wsola_taps; ++n) {
          _ref[_ref_idx++] = _tapAt(buf, _ref_cursor, wl);
          _ref_cursor += _search_rate;
        }
        if (_ref_idx >= wsola_taps) _sphase = SearchPhase::coarse;
        return;
      }
      case SearchPhase::coarse: {
        uint32_t budget = _taps_pf;
        while (budget > 0) {
          const uint32_t before = _tap_idx;
          if (!_lagStep(budget)) return;       // budget exhausted mid-lag
          budget -= (wsola_taps - before);
          if (_lag + wsola_coarse_step > 2 * wsola_half_range) {
            // Coarse pass done → refine around the best (stride 1).
            const uint32_t b = _best_lag;
            _lag = (b > wsola_fine_reach) ? (b - wsola_fine_reach) : 0;
            _fine_end = b + wsola_fine_reach;
            if (_fine_end > 2 * wsola_half_range)
              _fine_end = 2 * wsola_half_range;
            _sphase = SearchPhase::fine;
            return;
          }
          _lag += wsola_coarse_step;
        }
        return;
      }
      case SearchPhase::fine: {
        uint32_t budget = _taps_pf;
        while (budget > 0) {
          const uint32_t before = _tap_idx;
          if (!_lagStep(budget)) return;
          budget -= (wsola_taps - before);
          if (_lag >= _fine_end) {
            _sphase = SearchPhase::ready;
            return;
          }
          ++_lag;
        }
        return;
      }
      case SearchPhase::ready:
        return;
    }
  }

  IGB_FAST_INLINE std::pair<float, float> _readG(const LoopBuf& buf, q32_t wl,
                                                 const Grain& g, uint32_t sub) const {
    if (!g.active) return {0.0f, 0.0f};
    // Sub-frame 1 reads half a rate step ahead (io oversample, #111) —
    // wrapped per read since the base src is only wrapped once per frame.
    q32_t p = (sub == 0) ? g.src : q32_wrap_once(g.src + (g.rate >> 1), wl);
    if (p < 0) p = 0;   // negative-saturate like readLoopAhead (#198)
    return buf._readInterpQ(p);
  }

  static q32_t _wrapBounded(q32_t v, q32_t len) {
    while (v >= len) v -= len;
    while (v < 0)    v += len;
    return v;
  }

  // LilaC issue #208 (perf audit B4): the envelope LUT pointer. Defaults to
  // the in-class constexpr table (.rodata — host/parity paths unchanged); on
  // the device the owner repoints it at a zero-wait TCM mirror of the same
  // values (the table is read ×2 per rendered frame, and an XIP-flash
  // .rodata read is D-cache-missable on the hot path). Wiring, not state —
  // reset() must not touch it; owners re-wire after reconstruction (the
  // WowFlutterFx tape-ring class of hooks).
  const float* _env_lut = hann_ramp_lut;
  void setEnvLut(const float* lut257) { _env_lut = lut257; }

  // Half-Hann rise 0→1 over index 0..256, linear-interpolated. Literal table
  // (bit-identical host/device, .rodata — no boot-time trig); reads go
  // through _env_lut (same values wherever it points, so still bit-exact).
  IGB_FAST_INLINE float _env(float idx256) const {
    if (idx256 <= 0.0f) return 0.0f;
    if (idx256 >= 256.0f) return 1.0f;
    const uint32_t i = (uint32_t)idx256;
    const float t = idx256 - (float)i;
    const float* lut = _env_lut;
    return lut[i] + t * (lut[i + 1] - lut[i]);
  }

  constexpr static float hann_ramp_lut[257] = {
  0.00000000f, 0.00003765f, 0.00015059f, 0.00033881f, 0.00060227f, 0.00094094f, 0.00135477f, 0.00184369f,
  0.00240764f, 0.00304651f, 0.00376023f, 0.00454868f, 0.00541175f, 0.00634929f, 0.00736118f, 0.00844726f,
  0.00960736f, 0.01084131f, 0.01214893f, 0.01353002f, 0.01498437f, 0.01651176f, 0.01811197f, 0.01978474f,
  0.02152983f, 0.02334698f, 0.02523591f, 0.02719634f, 0.02922797f, 0.03133049f, 0.03350360f, 0.03574696f,
  0.03806023f, 0.04044307f, 0.04289512f, 0.04541601f, 0.04800535f, 0.05066277f, 0.05338785f, 0.05618019f,
  0.05903937f, 0.06196495f, 0.06495650f, 0.06801357f, 0.07113569f, 0.07432240f, 0.07757322f, 0.08088765f,
  0.08426519f, 0.08770535f, 0.09120759f, 0.09477140f, 0.09839623f, 0.10208155f, 0.10582679f, 0.10963139f,
  0.11349477f, 0.11741637f, 0.12139558f, 0.12543180f, 0.12952444f, 0.13367286f, 0.13787646f, 0.14213459f,
  0.14644661f, 0.15081188f, 0.15522973f, 0.15969950f, 0.16422052f, 0.16879211f, 0.17341358f, 0.17808423f,
  0.18280336f, 0.18757026f, 0.19238420f, 0.19724448f, 0.20215035f, 0.20710107f, 0.21209590f, 0.21713409f,
  0.22221488f, 0.22733751f, 0.23250119f, 0.23770516f, 0.24294863f, 0.24823081f, 0.25355090f, 0.25890811f,
  0.26430163f, 0.26973064f, 0.27519434f, 0.28069188f, 0.28622245f, 0.29178522f, 0.29737934f, 0.30300398f,
  0.30865828f, 0.31434140f, 0.32005248f, 0.32579066f, 0.33155507f, 0.33734485f, 0.34315913f, 0.34899703f,
  0.35485766f, 0.36074016f, 0.36664362f, 0.37256717f, 0.37850991f, 0.38447095f, 0.39044938f, 0.39644431f,
  0.40245484f, 0.40848006f, 0.41451906f, 0.42057093f, 0.42663476f, 0.43270965f, 0.43879466f, 0.44488890f,
  0.45099143f, 0.45710134f, 0.46321772f, 0.46933963f, 0.47546616f, 0.48159639f, 0.48772939f, 0.49386423f,
  0.50000000f, 0.50613577f, 0.51227061f, 0.51840361f, 0.52453384f, 0.53066037f, 0.53678228f, 0.54289866f,
  0.54900857f, 0.55511110f, 0.56120534f, 0.56729035f, 0.57336524f, 0.57942907f, 0.58548094f, 0.59151994f,
  0.59754516f, 0.60355569f, 0.60955062f, 0.61552905f, 0.62149009f, 0.62743283f, 0.63335638f, 0.63925984f,
  0.64514234f, 0.65100297f, 0.65684087f, 0.66265515f, 0.66844493f, 0.67420934f, 0.67994752f, 0.68565860f,
  0.69134172f, 0.69699602f, 0.70262066f, 0.70821478f, 0.71377755f, 0.71930812f, 0.72480566f, 0.73026936f,
  0.73569837f, 0.74109189f, 0.74644910f, 0.75176919f, 0.75705137f, 0.76229484f, 0.76749881f, 0.77266249f,
  0.77778512f, 0.78286591f, 0.78790410f, 0.79289893f, 0.79784965f, 0.80275552f, 0.80761580f, 0.81242974f,
  0.81719664f, 0.82191577f, 0.82658642f, 0.83120789f, 0.83577948f, 0.84030050f, 0.84477027f, 0.84918812f,
  0.85355339f, 0.85786541f, 0.86212354f, 0.86632714f, 0.87047556f, 0.87456820f, 0.87860442f, 0.88258363f,
  0.88650523f, 0.89036861f, 0.89417321f, 0.89791845f, 0.90160377f, 0.90522860f, 0.90879241f, 0.91229465f,
  0.91573481f, 0.91911235f, 0.92242678f, 0.92567760f, 0.92886431f, 0.93198643f, 0.93504350f, 0.93803505f,
  0.94096063f, 0.94381981f, 0.94661215f, 0.94933723f, 0.95199465f, 0.95458399f, 0.95710488f, 0.95955693f,
  0.96193977f, 0.96425304f, 0.96649640f, 0.96866951f, 0.97077203f, 0.97280366f, 0.97476409f, 0.97665302f,
  0.97847017f, 0.98021526f, 0.98188803f, 0.98348824f, 0.98501563f, 0.98646998f, 0.98785107f, 0.98915869f,
  0.99039264f, 0.99155274f, 0.99263882f, 0.99365071f, 0.99458825f, 0.99545132f, 0.99623977f, 0.99695349f,
  0.99759236f, 0.99815631f, 0.99864523f, 0.99905906f, 0.99939773f, 0.99966119f, 0.99984941f, 0.99996235f,
  1.00000000f,
  };
};

}
