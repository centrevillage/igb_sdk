#pragma once

#include <stdint.h>
#include <algorithm>
#include <array>

namespace igb {
namespace sdk {

struct ScaleQuantizer {
  uint8_t max_octave = 10;

  uint16_t _scale_bit = 0x0FFF;
  std::array<int8_t, 12> _scale_map = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int16_t _max_note = max_octave * 12 - 1;
  int16_t _min_note = 0;

  void updateScale(uint16_t scale_bit) {
    if (_scale_bit != scale_bit) {
      _scale_bit = scale_bit;
      _reconstruct();
    }
  }

  uint8_t quantize(uint16_t note) const noexcept {
    return std::clamp((int16_t)(note + _scale_map[note % 12]), _min_note, _max_note);
  }

  void _reconstruct() {
    // reconstruct _scale_map
    for (uint8_t i=0; i<12; ++i) {
      _scale_map[i] = 127;
    }
    for (uint8_t i=0;i<12;++i) {
      if (_scale_bit & (1 << i)) {
        _scale_map[i] = 0;
      }
    }
    for (uint8_t i=12;i>0;--i) {
      if (_scale_bit & (1 << i)) {
        _max_note = ((max_octave-1)*12) + i;
        break;
      }
    }
    for (uint8_t i=0;i<12;++i) {
      if (_scale_bit & (1 << i)) {
        _min_note = i;
        break;
      }
    }
    uint8_t found_127 = 1;
    while (found_127) {
      found_127 = 0;

      for (uint8_t i=0;i<12;++i) {
        if (_scale_map[i] != 127) {
          uint8_t left = (i+11)%12;
          uint8_t right = (i+1)%12;
          if (_scale_map[left] == 127) {
            _scale_map[left] = _scale_map[i]+1;
          }
          if (_scale_map[right] == 127) {
            _scale_map[right] = _scale_map[i]-1;
            ++i;
          }
        } else {
          found_127 = 1;
        }
      }
    }
  }
};

}
}

