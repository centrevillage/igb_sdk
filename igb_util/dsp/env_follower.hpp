#pragma once

#include <cmath>
#include <igb_util/macro.hpp>
#include <igb_util/math.hpp>
#include <igb_util/dsp/config.hpp>
#include <igb_util/dsp/math.hpp>

namespace igb::dsp {

struct EnvFollower {
  float attack_coeff = 1.0f;
  float release_coeff = 1.0f;

  struct Context {
    float y1 = 0.0f;
  };

  void init(float attack_time, float release_time) {
    changeAttack(attack_time);
    changeRelease(release_time);
  }

  void init(float attack_time, float release_time, uint32_t sampling_rate) {
    changeAttack(attack_time, sampling_rate);
    changeRelease(release_time, sampling_rate);
  }

  void changeAttack(float attack_time, uint32_t sampling_rate) {
    attack_coeff = igb::dsp::tau2pole(attack_time, sampling_rate);
  }

  void changeAttack(float attack_time) {
    attack_coeff = igb::dsp::tau2pole(attack_time);
  }

  void changeRelease(float release_time, uint32_t sampling_rate) {
    release_coeff = igb::dsp::tau2pole(release_time, sampling_rate);
  }

  void changeRelease(float release_time) {
    release_coeff = igb::dsp::tau2pole(release_time);
  }

  // IGB_FAST_INLINE: audio-IRQ callers (LilaCRepeater's block-head meter / mod
  // env flush) must not long-call this — an out-of-line body lands in QSPI
  // flash and gets veneer-called from the IRQ every block, which is the #201 /
  // #225 cold-I-fetch failure regardless of how rarely it runs.
  IGB_FAST_INLINE float process(Context& ctx, float x) {
    x = __builtin_fabsf(x);   // vabs, not a call: see the note above
    float coeff = (x > ctx.y1) ? attack_coeff : release_coeff;
    ctx.y1 = (1.0f - coeff) * x + coeff * ctx.y1;
    return ctx.y1;
  }
};

}
