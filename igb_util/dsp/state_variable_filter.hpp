#pragma once

#include <cstdint>
#include <cstddef>
#include <igb_util/macro.hpp>
#include <igb_util/algorithm.hpp>

namespace igb::dsp {

// LilaC issue #201 (Phase C): stereo state-variable filter, TPT (topology-
// preserving transform / trapezoidal-integrator) form.
//
// Why TPT SVF and not the resident BiQuadFilter (biquad_filter.hpp):
//   - modulation robustness: the trapezoidal integrators keep the filter
//     passive under per-block coefficient changes (LilaC sweeps the cutoff
//     every 125 µs audio block for auto-wah), where a Direct-Form biquad can
//     ring or transiently blow up when its feedback coefficients jump at
//     high Q;
//   - LP / BP / HP fall out of ONE state per sample, so a filter-type switch
//     is state-preserving (crossfade the output taps, no re-ring) and costs
//     nothing extra;
//   - BiQuadFilter's coefficient path calls std::sin/std::cos at runtime —
//     a libm libcall, banned anywhere in the LilaC audio IRQ (docs/198 §6).
//     Here the only per-block coefficient math is mul/div (single-precision
//     FDIV is a hardware instruction on the M7); the prewarp curve comes
//     from the offline LUTs below.
//
// Reference: the linear analysis in V. Zavalishin, "The Art of VA Filter
// Design" (the widely used a1/a2/a3 arrangement, e.g. Cytomic's
// SvfLinearTrapOptimised2). Per channel and sample, with damping k = 1/Q:
//   v3 = v0 - ic2;  v1 = a1*ic1 + a2*v3;  v2 = ic2 + a2*ic1 + a3*v3
//   ic1 = 2*v1 - ic1;  ic2 = 2*v2 - ic2
//   lp = v2,  bp0db = k*v1,  hp = v0 - k*v1 - v2
// Unconditionally stable for g > 0, k > 0 (bilinear image of a passive
// analog prototype) — bounded input keeps the state bounded even when g
// jumps across its whole range every block (unit-tested).

// --- v01 -> prewarped g cutoff curve -----------------------------------------
//
// fc(v01) = 20 * 900^v01 : exponential 20 Hz .. 18 kHz — an audible full-range
// sweep for auto-wah. g(v01) = tan(pi * fc / fs), tabulated per sample rate
// (LilaC runs the io path at 96 kHz and the host-parity path at 48 kHz; two
// small tables are simpler and more accurate than one normalized-frequency
// table, whose tan() curvature near the 48k top end would need denser knots).
// 129 knots + lerp: worst-case cutoff error ~0.2 % (a few cents) at the 48k
// top end, far below audibility for a performance filter.
//
// Offline literals (svf_cutoff_lut_gen.rb, pitch_semitone_lut precedent):
// bit-identical host/device — no reliance on toolchain constexpr libm
// folding. constexpr -> .rodata -> QSPI flash (D-cacheable, zero SRAM, #106);
// read once per BLOCK, never per frame.

constexpr float svf_cutoff_min_hz = 20.0f;
constexpr float svf_cutoff_max_hz = 18000.0f;
constexpr size_t svf_g_tbl_last = 128;   // 129 entries, lerp over 128 intervals

constexpr float svf_g_tbl_48k[svf_g_tbl_last + 1] = {
  0.00130899769f, 0.00138044442f, 0.00145579082f, 0.00153524973f,
  0.00161904563f, 0.00170741523f, 0.00180060818f, 0.00189888774f,
  0.00200253156f, 0.00211183242f, 0.00222709911f, 0.00234865726f,
  0.00247685027f, 0.0026120403f, 0.00275460927f, 0.00290495993f,
  0.00306351706f, 0.00323072859f, 0.00340706692f, 0.00359303024f,
  0.00378914392f, 0.00399596204f, 0.00421406891f, 0.00444408074f,
  0.00468664741f, 0.00494245424f, 0.00521222402f, 0.00549671898f,
  0.00579674297f, 0.00611314377f, 0.00644681541f, 0.00679870082f,
  0.00716979436f, 0.00756114478f, 0.00797385808f, 0.00840910071f,
  0.00886810285f, 0.00935216192f, 0.00986264626f, 0.010400999f,
  0.0109687423f, 0.0115674812f, 0.0121989091f, 0.0128648116f,
  0.0135670722f, 0.0143076776f, 0.0150887231f, 0.0159124189f,
  0.0167810965f, 0.0176972149f, 0.0186633684f, 0.0196822934f,
  0.0207568769f, 0.0218901644f, 0.0230853691f, 0.0243458809f,
  0.0256752767f, 0.027077331f, 0.0285560264f, 0.030115566f,
  0.0317603859f, 0.0334951682f, 0.0353248554f, 0.0372546653f,
  0.039290107f, 0.0414369981f, 0.0437014826f, 0.0460900506f,
  0.0486095586f, 0.0512672522f, 0.0540707894f, 0.0570282662f,
  0.0601482443f, 0.0634397802f, 0.0669124574f, 0.0705764205f,
  0.0744424127f, 0.0785218165f, 0.0828266974f, 0.0873698525f,
  0.0921648629f, 0.0972261518f, 0.102569048f, 0.108209857f,
  0.114165938f, 0.120455792f, 0.127099156f, 0.134117115f,
  0.141532223f, 0.149368636f, 0.157652276f, 0.166410998f,
  0.175674802f, 0.185476055f, 0.195849765f, 0.206833882f,
  0.218469659f, 0.230802064f, 0.243880267f, 0.257758211f,
  0.272495287f, 0.288157131f, 0.304816578f, 0.322554808f,
  0.341462719f, 0.361642599f, 0.383210163f, 0.406297056f,
  0.431053945f, 0.457654372f, 0.486299606f, 0.517224775f,
  0.550706731f, 0.587074219f, 0.626721174f, 0.670124361f,
  0.717867056f, 0.77067139f, 0.829443231f, 0.895335733f,
  0.969841245f, 1.05492758f, 1.15324581f, 1.26845771f,
  1.40577161f, 1.57286027f, 1.78152124f, 2.05088995f,
  2.41421356f,
};

constexpr float svf_g_tbl_96k[svf_g_tbl_last + 1] = {
  0.000654498563f, 0.000690221883f, 0.000727895025f, 0.000767624415f,
  0.000809522285f, 0.000853706993f, 0.000900303359f, 0.000949443013f,
  0.00100126477f, 0.00105591503f, 0.00111354818f, 0.00117432701f,
  0.00123842324f, 0.00130601792f, 0.00137730202f, 0.0014524769f,
  0.00153175494f, 0.00161536008f, 0.00170352852f, 0.00179650932f,
  0.00189456516f, 0.00199797305f, 0.0021070251f, 0.0022220294f,
  0.00234331084f, 0.00247121203f, 0.00260609431f, 0.00274833873f,
  0.00289834714f, 0.00305654333f, 0.00322337422f, 0.00339931113f,
  0.00358485111f, 0.00378051836f, 0.00398686567f, 0.00420447603f,
  0.00443396425f, 0.00467597872f, 0.00493120322f, 0.00520035887f,
  0.00548420618f, 0.00578354716f, 0.00609922766f, 0.00643213969f,
  0.006783224f, 0.00715347272f, 0.00754393218f, 0.00795570588f,
  0.0083899576f, 0.00884791473f, 0.00933087173f, 0.00984019381f,
  0.0103773208f, 0.0109437714f, 0.0115411471f, 0.0121711372f,
  0.0128355234f, 0.0135361848f, 0.0142751036f, 0.0150543704f,
  0.0158761903f, 0.0167428894f, 0.0176569212f, 0.0186208739f,
  0.0196374778f, 0.0207096131f, 0.0218403185f, 0.0230327997f,
  0.0242904389f, 0.0256168048f, 0.027015663f, 0.0284909871f,
  0.0300469706f, 0.0316880392f, 0.0334188641f, 0.0352443764f,
  0.0371697818f, 0.0392005766f, 0.0413425647f, 0.0436018758f,
  0.0459849847f, 0.0484987318f, 0.0511503454f, 0.0539474654f,
  0.0568981686f, 0.0600109959f, 0.0632949822f, 0.0667596874f,
  0.0704152315f, 0.0742723312f, 0.0783423407f, 0.0826372952f,
  0.0871699595f, 0.0919538798f, 0.097003442f, 0.102333935f,
  0.10796162f, 0.11390381f, 0.120178954f, 0.126806736f,
  0.13380818f, 0.141205773f, 0.149023601f, 0.157287504f,
  0.166025254f, 0.175266752f, 0.18504426f, 0.195392664f,
  0.206349784f, 0.21795672f, 0.230258274f, 0.243303425f,
  0.257145896f, 0.271844831f, 0.287465578f, 0.304080643f,
  0.321770821f, 0.340626561f, 0.360749624f, 0.382255102f,
  0.405273898f, 0.429955789f, 0.456473247f, 0.485026234f,
  0.51584828f, 0.549214259f, 0.585450441f, 0.624947642f,
  0.668178638f,
};

// v01 -> g via the given table (block-rate: one scale, one lerp).
IGB_FAST_INLINE float svf_g_from_v01(const float* tbl /* [svf_g_tbl_last+1] */,
                                     float v01) {
  if (v01 <= 0.0f) return tbl[0];
  if (v01 >= 1.0f) return tbl[svf_g_tbl_last];
  const float x = v01 * (float)svf_g_tbl_last;
  const uint32_t i = (uint32_t)x;
  return igb::lerp(tbl[i], tbl[i + 1], x - (float)i);
}

// --- resonance curve ---------------------------------------------------------
//
// v01 -> damping k = 1/Q, linear 2.0 .. 0.1 (Q 0.5 .. 10, peak up to +20 dB).
// SAFE ceiling: self-oscillation needs k <= 0, so k_min = 0.1 keeps the filter
// strictly passive — the state stays finite even when the cutoff jumps across
// its full range every block at max resonance (unit-tested). The linear-in-k
// map is perceptually gentle at the bottom and steepens toward the top
// (Q = 1/k), which suits a one-knob performance resonance.

constexpr float svf_damp_max = 2.0f;   // reso 0   -> Q = 0.5
constexpr float svf_damp_min = 0.1f;   // reso 1.0 -> Q = 10 (no self-osc)

IGB_FAST_INLINE float svf_damp_from_v01(float v01) {
  v01 = igb::clamp(v01, 0.0f, 1.0f);
  return svf_damp_max + (svf_damp_min - svf_damp_max) * v01;
}

// --- the filter ---------------------------------------------------------------

struct StateVariableFilter {
  // One sample's three simultaneous outputs. bp is the 0 dB-peak-gain band
  // pass (k*v1): its center gain stays unity while resonance narrows the band
  // — safe for a performance filter (the raw v1 band pass would peak at Q).
  struct Outs {
    float lp;
    float bp;
    float hp;
  };

  // Coefficients (block rate, written by setCoef only). Defaults are a benign
  // g≈0 stub; owners recompute via setCoef before the first audible process
  // (LilaC: FilterFx::blockFinalize with _coef_dirty=true initial state).
  float _a1 = 1.0f;
  float _a2 = 0.0f;
  float _a3 = 0.0f;
  float _k = svf_damp_max;
  // Trapezoidal integrator state, stereo.
  float _ic1_l = 0.0f;
  float _ic2_l = 0.0f;
  float _ic1_r = 0.0f;
  float _ic2_r = 0.0f;

  // Block-rate coefficient update: a handful of mul + one FDIV (hardware
  // instruction) — no libcalls. g from svf_g_from_v01, k from
  // svf_damp_from_v01 (or any g > 0, k > 0 pair).
  IGB_FAST_INLINE void setCoef(float g, float k) {
    _k = k;
    _a1 = 1.0f / (1.0f + g * (g + k));
    _a2 = g * _a1;
    _a3 = g * _a2;
  }

  void reset() {
    _ic1_l = 0.0f;
    _ic2_l = 0.0f;
    _ic1_r = 0.0f;
    _ic2_r = 0.0f;
  }

  // Per-frame stereo tick: computes all three outputs per channel (they are
  // byproducts of the same state update — the owner picks/crossfades taps).
  IGB_FAST_INLINE void process(float l, float r, Outs& out_l, Outs& out_r) {
    out_l = _tick(l, _ic1_l, _ic2_l);
    out_r = _tick(r, _ic1_r, _ic2_r);
  }

  IGB_FAST_INLINE Outs _tick(float v0, float& ic1, float& ic2) {
    const float v3 = v0 - ic2;
    const float v1 = _a1 * ic1 + _a2 * v3;
    const float v2 = ic2 + _a2 * ic1 + _a3 * v3;
    ic1 = 2.0f * v1 - ic1;
    ic2 = 2.0f * v2 - ic2;
    return { v2, _k * v1, v0 - _k * v1 - v2 };
  }
};

}
