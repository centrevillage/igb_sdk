#pragma once

#include <cstdint>
#include <cstddef>
#include <utility>
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

struct DigiTapeStereo {
  double tape_speed = 1.0;
  double pos = 0.0;
  double tape_head_read_pos = 16.0;

  std::pair<float, float> dummy_buf = std::make_pair(0.0f, 0.0f);
  std::pair<float, float>* buf = nullptr;
  size_t tape_size = 1;

  DigiTapeStereo() {
    buf = &dummy_buf;
    tape_size = 1;
  }

  DigiTapeStereo(std::pair<float, float>* target_buf, size_t size) : buf(target_buf), tape_size(size) { }

  void init(std::pair<float, float>* target_buf, size_t size) {
    buf = target_buf;
    tape_size = size;
  }

  IGB_FAST_INLINE void changeSpeed(double speed) {
    tape_speed = speed;
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
  IGB_FAST_INLINE void write(std::pair<float, float> value) {
    uint32_t pos_u32 = (uint32_t)pos;
    double pos_frac = pos - (double)pos_u32;

    if (tape_speed < 1.0) {
      if (pos_frac + tape_speed > 1.0) {
        uint32_t pos_u32_next = (pos_u32 + 1) % tape_size;
        double rate = (1.0 - pos_frac) / tape_speed;
        (*(buf + pos_u32)).first += value.first * tape_speed * rate;
        (*(buf + pos_u32)).second += value.second * tape_speed * rate;
        (*(buf + pos_u32_next)).first = value.first * tape_speed * (1.0 - rate);
        (*(buf + pos_u32_next)).second = value.second * tape_speed * (1.0 - rate);
      } else if (pos_frac == 0.0) {
        (*(buf + pos_u32)).first = value.first * tape_speed;
        (*(buf + pos_u32)).second = value.second * tape_speed;
      } else {
        (*(buf + pos_u32)).first += value.first * tape_speed;
        (*(buf + pos_u32)).second += value.second * tape_speed;
      }
    } else {
      if (pos_frac == 0.0) {
        (*(buf + pos_u32)).first = value.first;
        (*(buf + pos_u32)).second = value.second;
      } else {
        (*(buf + pos_u32)).first += value.first * (1.0 - pos_frac);
        (*(buf + pos_u32)).second += value.second * (1.0 - pos_frac);
      }
      double next_pos = pos + tape_speed;
      uint32_t next_pos_u32 = (uint32_t)next_pos;
      double next_pos_frac = next_pos - (double)next_pos_u32;
      for (uint32_t pos_u32_tmp = pos_u32 + 1; pos_u32_tmp < next_pos_u32; ++pos_u32_tmp) {
        (*(buf + (pos_u32_tmp % tape_size))).first = value.first;
        (*(buf + (pos_u32_tmp % tape_size))).second = value.second;
      }
      (*(buf + (next_pos_u32 % tape_size))).first = value.first * next_pos_frac;
      (*(buf + (next_pos_u32 % tape_size))).second = value.second * next_pos_frac;
    }

    _move();
  }

  IGB_FAST_INLINE std::pair<float, float> read() {
    double read_pos = pos + tape_head_read_pos;
    uint32_t read_pos_u32 = (uint32_t)read_pos;
    uint32_t read_pos_u32_next = (read_pos_u32 + 1) % tape_size;
    double pos_frac = read_pos - (double)read_pos_u32;
    read_pos_u32 = read_pos_u32 % tape_size;

    std::pair<float, float> v1 = *(buf + read_pos_u32);
    std::pair<float, float> v2 = *(buf + read_pos_u32_next);
    return std::make_pair<float, float>(v1.first * (1.0 - pos_frac) + v2.first * pos_frac, v1.second * (1.0 - pos_frac) + v2.second * pos_frac);
  }
};

}

