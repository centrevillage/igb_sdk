#ifndef IGB_UTIL_DIGI_TAPE_H
#define IGB_UTIL_DIGI_TAPE_H

#include <cstdint>
#include <cstddef>
#include <igb_util/macro.hpp>

namespace igb {

struct DigiTape {
  double tape_speed = 1.0;
  double pos = 0.0;
  double tape_head_read_pos = 16.0;

  float dummy_buf = 0.0f;
  float* buf = nullptr;
  size_t tape_size = 1;

  DigiTape() {
    buf = &dummy_buf;
    tape_size = 1;
  }

  DigiTape(float* target_buf, size_t size) : buf(target_buf), tape_size(size) { }

  void init(float* target_buf, size_t size) {
    buf = target_buf;
    tape_size = size;
  }

  IGB_FAST_INLINE void changeSpeed(double speed) {
    tape_speed = speed;
    //uint32_t pos_u32 = (uint32_t)pos;
    //pos = (double)pos_u32;
    //*(buf + pos_u32) = 0.0f;
  }

  IGB_FAST_INLINE void _move() {
    // move pos
    double next_pos = pos + tape_speed;
    double tape_size_d = (double)tape_size;
    if (next_pos >= tape_size_d) {
      pos = next_pos - tape_size_d;
    } else {
      pos = next_pos;
    }
  }

  // TODO: optimize
  IGB_FAST_INLINE void write(float value) {
    uint32_t pos_u32 = (uint32_t)pos;
    double pos_frac = pos - (double)pos_u32;

    if (tape_speed < 1.0) {
      if (pos_frac + tape_speed > 1.0) {
        uint32_t pos_u32_next = (pos_u32 + 1) % tape_size;
        double rate = (1.0 - pos_frac) / tape_speed;
        *(buf + pos_u32) += value * tape_speed * rate;
        *(buf + pos_u32_next) = value * tape_speed * (1.0 - rate);
      } else if (pos_frac == 0.0) {
        *(buf + pos_u32) = value * tape_speed;
      } else {
        *(buf + pos_u32) += value * tape_speed;
      }
    } else {
      if (pos_frac == 0.0) {
        *(buf + pos_u32) = value;
      } else {
        *(buf + pos_u32) += value * (1.0 - pos_frac);
      }
      double next_pos = pos + tape_speed;
      uint32_t next_pos_u32 = (uint32_t)next_pos;
      double next_pos_frac = next_pos - (double)next_pos_u32;
      for (uint32_t pos_u32_tmp = pos_u32 + 1; pos_u32_tmp < next_pos_u32; ++pos_u32_tmp) {
        *(buf + (pos_u32_tmp % tape_size)) = value;
      }
      *(buf + (next_pos_u32 % tape_size)) = value * next_pos_frac;
    }

    _move();
  }

  IGB_FAST_INLINE float read() {
    double read_pos = pos + tape_head_read_pos;
    uint32_t read_pos_u32 = (uint32_t)read_pos;
    uint32_t read_pos_u32_next = (read_pos_u32 + 1) % tape_size;
    double pos_frac = read_pos - (double)read_pos_u32;
    read_pos_u32 = read_pos_u32 % tape_size;

    return (*(buf + read_pos_u32)) * (1.0 - pos_frac)
      + (*(buf + read_pos_u32_next)) * pos_frac;
  }
};

}

#endif /* IGB_UTIL_DIGI_TAPE_H */
