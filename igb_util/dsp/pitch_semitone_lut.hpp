#pragma once

#include <cstdint>
#include <igb_util/dsp/q32_pos.hpp>

namespace igb::dsp {

// LilaC issue #200 (Phase B): semitone -> playback-rate ratio, ±24 st (±2 oct).
// Literal doubles (2^(n/12)) rather than a runtime pow: bit-identical on host
// and device, .rodata-resident (zero SRAM on QSPI-flash targets), and unit-
// testable against the closed-form values. Conversion to Q32.32 happens at
// edit time (main loop) via q32_pitch_ratio — never in the audio hot path.
constexpr int8_t pitch_semitone_range = 24;
constexpr double pitch_semitone_ratio[2 * pitch_semitone_range + 1] = {
  0.25,                // -24 st
  0.2648657735898238,  // -23 st
  0.28061551207734325, // -22 st
  0.29730177875068026, // -21 st
  0.3149802624737183,  // -20 st
  0.3337099635425086,  // -19 st
  0.3535533905932738,  // -18 st
  0.3745767692191704,  // -17 st
  0.3968502629920499,  // -16 st
  0.42044820762685725, // -15 st
  0.44544935907016964, // -14 st
  0.47193715634084676, // -13 st
  0.5,                 // -12 st
  0.5297315471796477,  // -11 st
  0.5612310241546865,  // -10 st
  0.5946035575013605,  //  -9 st
  0.6299605249474366,  //  -8 st
  0.6674199270850172,  //  -7 st
  0.7071067811865476,  //  -6 st
  0.7491535384383408,  //  -5 st
  0.7937005259840998,  //  -4 st
  0.8408964152537145,  //  -3 st
  0.8908987181403393,  //  -2 st
  0.9438743126816935,  //  -1 st
  1.0,                 //  +0 st
  1.0594630943592953,  //  +1 st
  1.122462048309373,   //  +2 st
  1.189207115002721,   //  +3 st
  1.2599210498948732,  //  +4 st
  1.3348398541700344,  //  +5 st
  1.4142135623730951,  //  +6 st
  1.4983070768766815,  //  +7 st
  1.5874010519681994,  //  +8 st
  1.681792830507429,   //  +9 st
  1.7817974362806785,  // +10 st
  1.8877486253633868,  // +11 st
  2.0,                 // +12 st
  2.1189261887185906,  // +13 st
  2.244924096618746,   // +14 st
  2.378414230005442,   // +15 st
  2.5198420997897464,  // +16 st
  2.6696797083400687,  // +17 st
  2.8284271247461903,  // +18 st
  2.996614153753363,   // +19 st
  3.174802103936399,   // +20 st
  3.363585661014858,   // +21 st
  3.563594872561357,   // +22 st
  3.775497250726774,   // +23 st
  4.0,                 // +24 st
};

// Edit-time semitone -> Q32.32 rate. Clamps out-of-range input (input-path
// parity with the domain clamp); `negative` folds the track's is_reverse in
// (a reverse grain reads backwards at the same |ratio| — LilaC Q2).
IGB_FAST_INLINE q32_t q32_pitch_ratio(int8_t semitone, bool negative = false) {
  if (semitone < -pitch_semitone_range) semitone = -pitch_semitone_range;
  if (semitone >  pitch_semitone_range) semitone =  pitch_semitone_range;
  double r = pitch_semitone_ratio[semitone + pitch_semitone_range];
  return q32_from_double(negative ? -r : r);
}

}
