#pragma once

#include <cmath>
#include <array>
#include <igb_util/random.hpp>
#include <igb_util/dsp/dsp_tbl_func.hpp>
#include <igb_util/dsp/config.hpp>

namespace igb::dsp {

struct PerlinNoise {
  float freq = 10.0f;
  float delta = 0.0f;
  float phase = 0.0f;

  float a1 = 0.0f; /* 0.0 ~ 4.0 */
  float a2 = 0.0f; /* 0.0 ~ 4.0 */

  float a_max = 4.0f;

  void init(float _freq) {
    phase = 0.0f;
    changeFreq(_freq);
    a1 = (igb::rand_f() - 0.5f) * a_max * 2.0f;
    a2 = (igb::rand_f() - 0.5f) * a_max * 2.0f;
  }

  void changeFreq(float _freq) {
    freq = _freq;
    delta = freq / Config::getSamplingRateF();
  }

  float process() {
    phase += delta;
    if (phase >= 1.0f) {
      phase -= (uint32_t)phase;
      a1 = a2;
      a2 = (igb::rand_f() - 0.5f) * a_max * 2.0f;
    }
    float wavelet_v1 = igb::dsp::perlin_5order_fast(phase) * (phase * a1);
    float phase2 = -1.0f + phase;
    float wavelet_v2 = igb::dsp::perlin_5order_fast(phase2) * (phase2 * a2);
    
    return std::lerp(wavelet_v1, wavelet_v2, phase);
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

  void changeFreq(float _freq) {
    freq = _freq;
    float freq_tmp = freq;
    for (size_t i = 0; i < order_size; ++i) {
      noises[i].changeFreq(freq_tmp);
      freq_tmp *= 2.0f;
    }
  }

  float process() {
    float v = 0.0f;
    float gain = 1.0f;
    for (size_t i = 0; i < order_size; ++i) {
      v += noises[i].process() * gain;
      gain *= decay;
    }
    return v;
  }
};

}
