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
  q32_t _pitch_q = q32_one;   // next spawn's rate (edit-time setter)

  // Edit-time (main loop): grains in flight keep their captured rate.
  void setPitch(q32_t p) { _pitch_q = p; }

  // Tuning entry (spike/listening): resets the render state.
  void setGrainLen(uint32_t len) {
    if (len < 128) len = 128;
    len &= ~1u;                        // even → exact 50% hop
    _hop = len / 2;
    _env_step = 256.0f / (float)_hop;
    reset();
  }
  uint32_t grainLen() const { return _hop * 2; }

  // Reseed hook (docs/v2_timestretch §5): pos discontinuities (undo swap,
  // step-jump, stutter repin, window commit, loopset swap, …) drop the
  // in-flight grains; the next render spawns fresh and Hann-ramps in
  // (~hop/48k s) — click-free by construction.
  void reset() {
    _out_g.active = false;
    _in_g.active = false;
    _k = 0;
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
  }

  // --- internals -----------------------------------------------------------

  IGB_FAST_INLINE void _spawn(LoopBuf& buf, q32_t wl) {
    // anchor = pos + (r − p)·L/2 (design §3). |r−p|·L/2 can exceed one
    // window on extreme rate deltas × short windows, so wrap is a bounded
    // loop here (q32_wrap_once is a ±1-window helper).
    const q32_t lead = (buf.tape_speed_q - _pitch_q) * (q32_t)_hop;
    _in_g.src = _wrapBounded(buf.pos_q + lead, wl);
    _in_g.rate = _pitch_q;
    _in_g.active = true;
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

  // Half-Hann rise 0→1 over index 0..256, linear-interpolated. Literal table
  // (bit-identical host/device, .rodata — no boot-time trig).
  IGB_FAST_INLINE static float _env(float idx256) {
    if (idx256 <= 0.0f) return 0.0f;
    if (idx256 >= 256.0f) return 1.0f;
    const uint32_t i = (uint32_t)idx256;
    const float t = idx256 - (float)i;
    return hann_ramp_lut[i] + t * (hann_ramp_lut[i + 1] - hann_ramp_lut[i]);
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
