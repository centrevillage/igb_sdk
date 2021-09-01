#ifndef IGB_SDK_UTIL_QUANTIZER_H
#define IGB_SDK_UTIL_QUANTIZER_H

#include <stdint.h>
#include <algorithm>

namespace igb {
namespace sdk {

struct ScaleQuantizer {
  uint8_t max_octave = 10;

  uint16_t _scale_bit = 0x0FFF;
  int8_t _scale_map[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int8_t _max_note = max_octave * 12 - 1;
  int8_t _min_note = 0;

  void updateScale(uint16_t scale_bit) {
    if (_scale_bit != scale_bit) {
      _scale_bit = scale_bit;
      _reconstruct();
    }
  }

  uint8_t quantize(uint8_t note) {
    return std::clamp((int8_t)(note + _scale_map[note % 12]), _min_note, _max_note);
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
          } else if (_scale_map[right] == 127) {
            _scale_map[right] = _scale_map[i]-1;
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

#endif /* IGB_SDK_UTIL_QUANTIZER_H */
