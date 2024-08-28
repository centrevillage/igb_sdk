#pragma once

#include <stdint.h>
#include <algorithm>
#include <array>

namespace igb {
namespace sdk {

// Neighbor Quantizer
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

  uint8_t quantize(uint16_t note /* 0 ~ 119 */) const noexcept {
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

// Weighted Quantizer
template<uint16_t resolution_rate = 8 /* 1 ~ 43 */>
struct WeightedScaleQuantizer {
  uint8_t max_octave = 10;

  uint16_t _scale_bit = 0x0FFF;
  std::array<uint8_t, 12> _priorities; // 0~127
  constexpr static uint16_t _scale_map_size = 12 * resolution_rate;
  std::array<int16_t, _scale_map_size> _scale_map;
  int16_t _max_note = (max_octave * 12 * resolution_rate) - 1;
  int16_t _min_note = 0;

  void updateScale(const std::array<uint8_t, 12>& priorities) {
    bool is_changed = false;
    for (uint8_t i = 0; i < 12; ++i) {
      if (_priorities[i] != priorities[i]) {
        is_changed = true;
        break;
      }
    }
    if (is_changed) {
      uint16_t new_scale_bit = 0; 
      for (uint16_t i = 0; i < 12; ++i) {
        new_scale_bit |= ((priorities[i] > 0 ? 1 : 0) << i);
      }
      if (new_scale_bit) { // avoid illegal scale
        _priorities = priorities;
        _scale_bit = new_scale_bit;
        _reconstruct();
      }
    }
  }

  uint8_t quantize(uint16_t note /* 0 ~ 12 * 10 * resolution_rate  */) const noexcept {
    return std::clamp((int16_t)(note + _scale_map[note % _scale_map_size]), _min_note, _max_note);
  }

  void _reconstruct() {
    // reconstruct _scale_map
    // don't use float type for no-fpu mcu 

    for (uint8_t i=12;i>0;--i) {
      if (_scale_bit & (1 << i)) {
        _max_note = (((max_octave-1)*12) + i) * resolution_rate;
        break;
      }
    }
    for (uint8_t i=0;i<12;++i) {
      if (_scale_bit & (1 << i)) {
        _min_note = i * resolution_rate;
        break;
      }
    }
    
    uint16_t priority_sum = 0;
    for (uint8_t i = 0; i < 12; ++i) {
      priority_sum += _priorities[i];
    }
    std::array<uint16_t, 12> key_range_sizes;
    uint16_t assigned_size = 0;
    for (uint16_t i = 0; i < 12; ++i) {
      if (_scale_bit & (1 << i)) {
        key_range_sizes[i] = _priorities[i] * _scale_map_size / priority_sum;
        assigned_size += key_range_sizes[i];
      } else {
        key_range_sizes[i] = 0;
      }
    }
    uint16_t remain_size = _scale_map_size - assigned_size;
    // TODO: optimizing by sort algorithm
    for (; remain_size > 0; --remain_size) {
      uint16_t min_size = _scale_map_size;
      int8_t min_idx = -1;
      for (int8_t i = 0; i < 12; ++i) {
        if (_scale_bit & (1 << i)) {
          if (key_range_sizes[i] < min_size) {
            min_size = key_range_sizes[i];
            min_idx = i;
          }
        }
      }
      if (min_idx >= 0) {
        key_range_sizes[min_idx] += 1;
      }
    }

    uint16_t scale_map_idx = 0;
    for (int16_t key = 0; key < 12; ++key) {
      uint16_t key_range_size = key_range_sizes[key];
      int16_t target_key_value = key * resolution_rate;
      for (uint16_t j = 0; j < key_range_size; ++j) {
        _scale_map[scale_map_idx] = -(scale_map_idx - target_key_value);
        ++scale_map_idx;
      }
    }
  }
};

}
}

