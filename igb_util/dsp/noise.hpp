#pragma once

#include <cmath>
#include <array>
#include <algorithm>
#include <igb_util/macro.hpp>
#include <igb_util/random.hpp>
#include <igb_util/dsp/dsp_tbl_func.hpp>
#include <igb_util/dsp/config.hpp>

namespace igb::dsp {

// Closed form of dsp_func_perlin_5order_tbl: 1 − smootherstep(u) =
// 1 − (6u⁵ − 15u⁴ + 10u³), u ∈ [0, 1]. ~8 FLOPs and NO memory access —
// the table lives in .rodata (QSPI XIP on LilaC) and costs a D-cache line
// fill per draw whenever other streams churn the cache (LilaC #202: the
// per-block noise draws measured ~10µs/IRQ of QSPI fills on a busy cache).
// Also exact where the 256-entry nearest-neighbor table quantizes.
IGB_FAST_INLINE float perlin_5order_shape(float u /* 0.0 ~ 1.0 */) {
  const float u3 = u * u * u;
  return 1.0f - u3 * (u * (6.0f * u - 15.0f) + 10.0f);
}

struct PerlinNoise {
  float freq = 10.0f;
  float delta = 0.0f;
  float phase = 0.0f;

  float a1 = 0.0f; /* 0.0 ~ 4.0 */
  float a2 = 0.0f; /* 0.0 ~ 4.0 */

  float a_max = 4.0f;

  // Private random stream: a seeded instance replays the same wavelet
  // sequence deterministically (reconstruct + init(seed) — init alone keeps
  // x/y evolved, see RandomXorshift). The unseeded init() below draws the
  // seed from the global stream, preserving the legacy behavior.
  igb::RandomXorshift rng;

  void init(float _freq) {
    init(_freq, igb::rand_u32());
  }

  void init(float _freq, uint32_t seed) {
    initAt(_freq, Config::getSamplingRateF(), seed);
  }

  // Explicit-rate variant: callers stepping process() at a non-audio cadence
  // (e.g. once per control block) pass that cadence as `sample_rate` instead
  // of relying on the global Config sampling rate.
  void initAt(float _freq, float sample_rate, uint32_t seed) {
    phase = 0.0f;
    rng = igb::RandomXorshift{};
    rng.init(seed);
    changeFreqAt(_freq, sample_rate);
    a1 = (rng.getf() - 0.5f) * a_max * 2.0f;
    a2 = (rng.getf() - 0.5f) * a_max * 2.0f;
  }

  void changeFreq(float _freq) {
    changeFreqAt(_freq, Config::getSamplingRateF());
  }

  void changeFreqAt(float _freq, float sample_rate) {
    freq = _freq;
    delta = freq / sample_rate;
  }

  // always_inline: runs on audio-IRQ paths (ITCM) — an out-of-line body in
  // flash would be long-called through a veneer per call.
  IGB_FAST_INLINE float process() {
    phase += delta;
    if (phase >= 1.0f) {
      phase -= (uint32_t)phase;
      a1 = a2;
      a2 = (rng.getf() - 0.5f) * a_max * 2.0f;
    }
    // perlin_5order_shape == the perlin_5order_fast table's closed form
    // (see above): phase ∈ [0,1), the mirrored arm reads at 1−phase.
    float wavelet_v1 = perlin_5order_shape(phase) * (phase * a1);
    float phase2 = -1.0f + phase;
    float wavelet_v2 = perlin_5order_shape(-phase2) * (phase2 * a2);

    // Manual lerp: std::lerp is not always_inline and gets outlined to flash
    // when the (ITCM) caller exceeds GCC's inline budget (LilaC #202 audit).
    return wavelet_v1 + (wavelet_v2 - wavelet_v1) * phase;
  }
};

template<typename NoiseType = PerlinNoise, size_t order_size = 5>
struct FractalNoise {
  float freq = 10.0f;
  float decay = 0.5f;

  std::array<NoiseType, order_size> noises;

  void init(float _freq, float _decay = 0.5f) {
    decay = _decay;
    freq = _freq;
    float freq_tmp = freq;
    for (size_t i = 0; i < order_size; ++i) {
      noises[i].init(freq_tmp);
      freq_tmp *= 2.0f;
    }
  }

  // Deterministic + explicit-rate variant (see PerlinNoise::initAt): octave i
  // seeds `seed + (i+1)·golden` so the per-octave streams differ but the
  // whole stack replays from one seed.
  void initAt(float _freq, float _decay, float sample_rate, uint32_t seed) {
    decay = _decay;
    freq = _freq;
    float freq_tmp = freq;
    for (size_t i = 0; i < order_size; ++i) {
      noises[i].initAt(freq_tmp, sample_rate, seed + 0x9E3779B9u * (uint32_t)(i + 1));
      freq_tmp *= 2.0f;
    }
  }

  void changeFreq(float _freq) {
    freq = _freq;
    float freq_tmp = freq;
    for (size_t i = 0; i < order_size; ++i) {
      noises[i].changeFreq(freq_tmp);
      freq_tmp *= 2.0f;
    }
  }

  IGB_FAST_INLINE float process() {
    float v = 0.0f;
    float gain = 1.0f;
    for (size_t i = 0; i < order_size; ++i) {
      v += noises[i].process() * gain;
      gain *= decay;
    }
    return v;
  }
};

// Voss-McCartney Algorithm
// ref: https://www.firstpr.com.au/dsp/pink-noise/
// ref: https://www.modwiggler.com/forum/viewtopic.php?t=283571
struct PinkNoise {
  constexpr static uint32_t order_size = 16;
  uint32_t values[order_size];
  uint32_t count = 0;
  uint32_t sum = 0; 

  void init() {
    for (uint8_t i = 0; i < order_size; ++i) {
      values[i] = igb::rand_u32();
      sum += values[i];
    }
  }

  uint32_t processU32() {
    uint16_t num_zeroes = __builtin_ctz(count | ((uint32_t)1 << order_size));
    sum -= values[num_zeroes];
    values[num_zeroes] = igb::rand_u32() >> 8;
    sum += values[num_zeroes];
    ++count;
    uint32_t total = sum + (igb::rand_u32() >> 8);
    return total << 4;
  }

  float process() {
    uint32_t v = processU32();
    return (float)((float)v / (float)0xFFFFFFFFUL) * 2.0f - 1.0f;
  }
};

// ============================================================================
// Velvet noise (Karjalainen & Järveläinen 2007)
// ============================================================================
struct VelvetNoise {
  uint32_t gridPeriod = 2;
  float amp = 0.0f;
  uint32_t impulsePos = 0;
  float impulseSign = 0.0f;
  uint32_t gridIdx = 0;

  void init(float densityHz = 2000.0f) {
    gridPeriod = std::max((uint32_t)2, (uint32_t)std::floor(Config::getSamplingRateF() / std::max(50.0f, densityHz)));
    amp = std::sqrt((float)gridPeriod);     // RMS = 1 normalisation
    _rollImpulse();
  }

  void update(float densityHz) {
    const uint32_t gp = std::max((uint32_t)2, (uint32_t)std::floor(Config::getSamplingRateF() / std::max(50.0f, densityHz)));
    if (gp == gridPeriod) return;
    gridPeriod = gp;
    amp = std::sqrt((float)gp);
    if (gridIdx >= gp) gridIdx = 0;
    if (impulsePos >= gp) _rollImpulse();
  }

  void _rollImpulse() {
    impulsePos = igb::rand_f() * gridPeriod;
    impulseSign = igb::rand_f() < 0.5 ? -1.0 : 1.0;
  }

  float process() {
    float out = 0;
    if (gridIdx == impulsePos) {
      out = impulseSign * amp;
    }
    gridIdx++;
    if (gridIdx >= gridPeriod) {
      gridIdx = 0;
      _rollImpulse();
    }
    return out;
  }
};

}
