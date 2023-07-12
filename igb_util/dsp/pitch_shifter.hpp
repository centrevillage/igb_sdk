#pragma once

#include <cmath>
#include <igb_util/dsp/delay_line.hpp>

namespace igb::dsp {

template <bool sin_interpolate>
struct PitchShifterImpl {
  size_t window_size = 0;
  size_t crossfade_duration = 0;

  igb::dsp::DelayLine delay;

  size_t write_pos = 0;
  double pos_delta = 0.0f;
  double pos_delta_cur = 0.0f;
  double pos = 0.0f;
  double pos_filter_coeff = 0.99f;
  double zero_pos_filter_coeff = 0.99f;

  constexpr static size_t spacer_size = 0;

  void init(float* target_buf, size_t size /* greater than spacer_size + 1 */) {
    delay.init(target_buf, size);
    window_size = (size - spacer_size) / 2;
    crossfade_duration = window_size / 4;
    pos = (double)window_size / 2.0;
  }

  void shift(double semitone) {
    pos_delta = 1.0 - std::pow(2.0, semitone / 12.0);
  }

  float process(float x) {
    if (pos_delta == 0.0) {
      pos = (1.0f - zero_pos_filter_coeff) * ((double)window_size / 2.0) + zero_pos_filter_coeff * pos;
    } else {
      //pos = (pos + pos_delta_cur + window_size) % window_size;
      pos = std::fmod((pos + pos_delta_cur + (double)window_size), (double)window_size);
    }
    float d1 = delay.readF(pos);
    float d2 = delay.readF(pos + (double)window_size);
    pos_delta_cur = (1.0 - pos_filter_coeff) * pos_delta + pos_filter_coeff * pos_delta_cur;
    float rate = std::min(pos / (double)crossfade_duration, 1.0);

    float y;
    if (sin_interpolate) {
      float rate_d1 = std::sin(igb::numbers::pi / 2.0f * rate); 
      float rate_d2 = std::cos(igb::numbers::pi / 2.0f * rate);
      y = d1 * rate_d1 + d2 * rate_d2;
    } else {
      y = d1 * rate + d2 * (1.0f - rate);
    }

    delay.write(x);

    return y;
  }
};

typedef PitchShifterImpl<false> PitchShifter;


}
