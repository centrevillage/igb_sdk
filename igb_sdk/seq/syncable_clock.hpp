#pragma once

#include <functional>
#include <numeric>
#include <tuple>
#include <igb_util/for_each.hpp>

namespace igb {
namespace sdk {

// TimerCls = HardTimerStm32, etc...
template<typename TimerCls, uint32_t out_clocks_size = 1, uint32_t src_clocks_size = 2>
struct SeqSyncableClock {
  constexpr static uint32_t timeout_tick = (uint32_t)TimerCls::secToTick(10.0f);
  static_assert(TimerCls::cc_ch_count >= (out_clocks_size + 1), "cc ch size is not sufficient.");
  constexpr static uint32_t int_clock_cc_idx = TimerCls::cc_ch_count - 1;

  float bpm = 120.0f;
  float int_interval_tick = 0.0f;

  struct ClockConf {
    uint16_t step_per_beat = 4;
    std::function<void(void)> on_update = [](){};
  };
  struct SrcConf {
    uint16_t clock_per_beat = 4;
    float filter_coeff = 0.0f; // 0 == filter off, 0 .. 1.0f == filter on
    bool internal = false;
  };
  constexpr static SrcConf midi_clock_conf = SrcConf {
    .clock_per_beat = 24,
    .filter_coeff = 0.9f
  };
  constexpr static SrcConf trig_clock_conf = SrcConf {
    .clock_per_beat = 4,
    .filter_coeff = 0.0f
  };
  constexpr static SrcConf tap_clock_conf = SrcConf {
    .clock_per_beat = 1,
    .filter_coeff = 0.0f
  };

  struct ClockState {
    uint8_t step_per_beat = 4;
    float interval_tick = TimerCls::secToTick(SeqSyncableClock::bpmToIntervalSec(120.0f, step_per_beat));
    std::function<void(void)> on_update = [](){};

    uint16_t clock_per_beat = 4;
    float filter_coeff = 0.0f;

    uint16_t clock_div = 1;

    // for int&ext clock sync
    uint16_t clk_count = 0; // clear when received reset event
    uint16_t clk_count_max = 0;
    uint16_t ipl_count = 0;
    uint16_t ipl_count_max = 0;
    float clock_to_step_coeff = 0.0f;

    // for bpm detection
    //float expected_interval_tick = 0.0f; // 0 == not expected
    //float expected_beat_interval_tick = 0.0f; // 0 == not expected

    inline float expectedIntervalTick(uint32_t expected_clock_interval_tick) {
      return (float)expected_clock_interval_tick * (float)clock_to_step_coeff;
    }

    inline void resetCounts() {
      clk_count = 0;
      ipl_count = 0;
    }

    inline void _updateCompletionCount() {
      const auto cpb = clock_per_beat * clock_div;
      const auto gcd_num = std::gcd(step_per_beat, cpb);
      clk_count_max = (cpb / gcd_num) - 1;
      ipl_count_max = (step_per_beat / gcd_num) - 1;
      clock_to_step_coeff = (float)(cpb) / (float)step_per_beat;
    }

    inline void changeStepPerBeat(uint16_t new_step_per_beat) {
      if (new_step_per_beat > 0) {
        step_per_beat = new_step_per_beat;
        _updateCompletionCount();
      }
    }

    inline void changeClockPerBeat(uint8_t new_clock_per_beat) {
      if (new_clock_per_beat > 0) {
        clock_per_beat = new_clock_per_beat;
        _updateCompletionCount();
      }
    }

    inline void changeClockDiv(uint16_t new_clock_div) {
      if (new_clock_div > 0) {
        clock_div = new_clock_div;
        _updateCompletionCount();
      }
    }
  };

  struct ExtSrcState {
    bool is_first_clock_arrived = false;
    uint32_t prev_tick = 0;
    uint32_t expected_clock_interval_tick = 0; // 0 == not expected
  };

  TimerCls _timer;
  std::array<ClockState, out_clocks_size> _clockStates;
  std::array<SrcConf, src_clocks_size> _src_confs;
  ExtSrcState _ext_src_state;
  uint8_t _selected_src_clock_idx = 0;
  uint8_t _selected_int_clock_idx = 0;
  bool _int_active = false;
  bool _is_start = false;

  void init(float _bpm, auto&& clockConfs, auto&& srcConfs) {
    bpm = _bpm;
    _timer.init();
    for_each_tuple(srcConfs, [this](auto&& idx, auto&& value) {
      _initSrcConf(idx, value);
    });
    for_each_tuple(clockConfs, [this](auto&& idx, auto&& value) {
      _initClockConf(idx, value);
    });
    _timer.enable();
  }

  inline void setBpm(float _bpm) {
    bpm = _bpm;
    float interval_sec = bpmToIntervalSec(bpm, _src_confs[_selected_int_clock_idx].clock_per_beat);
    setIntIntervalSec(interval_sec);
  }

  inline float getBpm() {
    return bpm;
  }

  inline void _initClockConf(uint8_t idx, auto&& conf) {
    _clockStates[idx].changeStepPerBeat(conf.step_per_beat);
    _clockStates[idx].step_per_beat = conf.step_per_beat;
    float interval_sec = bpmToIntervalSec(bpm, conf.step_per_beat);
    _clockStates[idx].interval_tick = TimerCls::secToTick(interval_sec);
    _clockStates[idx].on_update = conf.on_update;
    _timer.setIntervalTick(idx, _clockStates[idx].interval_tick);
    _timer.setupTimer(idx, interval_sec, [this, idx](){onUpdateIntTimer(idx);});
  }

  inline void _initIntClock() {
    if (_selected_int_clock_idx >= src_clocks_size) { return; }
    float interval_sec = bpmToIntervalSec(bpm, _src_confs[_selected_int_clock_idx].clock_per_beat);
    setIntIntervalSec(interval_sec);
    _timer.setupTimer(int_clock_cc_idx, interval_sec, [this](){ receiveClock(_selected_int_clock_idx); });
  }

  inline void _initSrcConf(uint8_t idx, auto&& conf) {
    _src_confs[idx] = conf;
    if (conf.internal) {
      selectIntClock(idx);
    }
  }

  inline std::optional<uint8_t> getActiveSrcClockIdx() const {
    return _selected_src_clock_idx;
  }

  inline void selectIntClock(uint8_t idx) {
    _selected_int_clock_idx = idx;
    _initIntClock();
  }
  inline void enableIntClock() {
    _int_active = true;
  }
  inline void disableIntClock() {
    _int_active = false;
  }
  inline bool isIntClockEnabled() {
    return _int_active;
  }
  inline void selectSrcClockIdx(uint8_t idx) {
    for (auto& state : _clockStates) {
      state.resetCounts();
      state.changeClockPerBeat(_src_confs[idx].clock_per_beat);
      state.filter_coeff = _src_confs[idx].filter_coeff;
    }
    _ext_src_state.is_first_clock_arrived = false;
    _selected_src_clock_idx = idx;
  }

  inline void start() {
    if (_is_start) {
      return;
    }
    resetClocks();
    if (isIntClockEnabled()) {
      for (auto& state : _clockStates) {
        //state.on_update();
        state.interval_tick = int_interval_tick * (float)(_src_confs[_selected_int_clock_idx].clock_per_beat) / (float)state.step_per_beat;
      }
      _timer.setIntervalTick(int_clock_cc_idx, int_interval_tick);
      _timer.start(int_clock_cc_idx);
    }
    _is_start = true;
  }

  inline void stop() {
    if (!_is_start) {
      return;
    }
    //if (isIntClockEnabled()) {
      _timer.stop(int_clock_cc_idx);
      for (uint8_t i = 0; i < out_clocks_size; ++i) {
        _timer.stop(i);
      }
    //}
    _is_start = false;
    resetClocks();
  }

  inline void resetClocks() {
    for (auto& state : _clockStates) {
      state.resetCounts();
    }
  }

  inline ClockState& clockState(uint8_t clock_idx) { return _clockStates[clock_idx]; }

  inline void resetCounts(uint8_t clock_idx) {
    _clockStates[clock_idx].resetCounts();
  }

  inline void receiveClock(uint8_t idx) {
    if (_selected_src_clock_idx != idx) {
      return;
    }
    const uint32_t t = _timer.tick();

    if (_ext_src_state.is_first_clock_arrived) {
      const uint32_t diff_tick = (t - _ext_src_state.prev_tick);
      expectClockInterval(diff_tick);
    }

    for (uint8_t i = 0; i < out_clocks_size; ++i) {
      updateClockState(i);
    }

    if (!_ext_src_state.is_first_clock_arrived) {
      _ext_src_state.is_first_clock_arrived = true;
    }
    _ext_src_state.prev_tick = t;
  }

  inline void updateClockState(uint8_t idx) {
    if (!_is_start) { return; }
    auto& state = _clockStates[idx];
    if (_ext_src_state.is_first_clock_arrived) {
      float t = state.expectedIntervalTick(_ext_src_state.expected_clock_interval_tick);
      state.interval_tick = t;
      _timer.setIntervalTick(idx, state.interval_tick * 0.99f); // NOTE: the magic number 0,99 is to prevent ext&int clock collision
    }
    if (state.clk_count == 0) {
      // sync int&ext clock
      _timer.stop(idx);
      state.on_update();
      state.ipl_count = state.ipl_count_max;
      state.clk_count = state.clk_count_max;
      if (state.ipl_count != 0) {
        _timer.start(idx);
      }
    } else {
      state.clk_count--;
    }
  }

  inline void expectClockInterval(uint32_t diff_tick) {
    if (diff_tick < timeout_tick) {
      const auto filter_coeff = _src_confs[_selected_src_clock_idx].filter_coeff;
      if (_ext_src_state.expected_clock_interval_tick > 0 || filter_coeff > 0.0f) { 
        _ext_src_state.expected_clock_interval_tick = (_ext_src_state.expected_clock_interval_tick * filter_coeff) + ((float)diff_tick * (1.0f - filter_coeff));
      } else {
        _ext_src_state.expected_clock_interval_tick = diff_tick;
      }
    }
  }

  void onUpdateIntTimer(uint8_t clock_idx) {
    auto& state = _clockStates[clock_idx];
    if (state.ipl_count > 0) {
      state.ipl_count--;
      state.on_update();
    } 
    if (state.ipl_count == 0) {
      _timer.stop(clock_idx); 
    }
  }

  void checkTimeout() {
    const auto t = _timer.tick();
    if (_ext_src_state.is_first_clock_arrived) {
      if ((t - _ext_src_state.prev_tick) > timeout_tick) {
        _ext_src_state.is_first_clock_arrived = false;
      }
    }
  }

  inline void changeStepPerBeat(uint8_t idx, uint16_t step_per_beat) {
    _clockStates[idx].changeStepPerBeat(step_per_beat);
  }

  inline void changeClockPerBeat(uint8_t idx, uint8_t clock_per_beat) {
    _clockStates[idx].changeClockPerBeat(clock_per_beat);
  }

  inline void changeClockDiv(uint8_t idx, uint8_t clock_div) {
    _clockStates[idx].changeClockDiv(clock_div);
  }

  constexpr static float bpmToIntervalSec(float _bpm, uint16_t step_per_beat) {
    const float sec_per_beat = 60.0f / _bpm;
    const float sec_per_step = sec_per_beat / step_per_beat;
    return sec_per_step;
  }

  constexpr static float intervalSecToBpm(float interval_sec, uint16_t step_per_beat) {
    const float sec_per_beat = interval_sec * (float)step_per_beat;
    return 60.0f / sec_per_beat;
  }

  inline float getIntervalSec(uint8_t idx) const {
    return _timer.getIntervalSec(idx);
  }

  inline float getIntervalTick(uint8_t idx) const {
    return _timer.getIntervalTick(idx);
  }

  inline float getIntIntervalSec() const {
    return TimerCls::tickToSec(int_interval_tick);
  }

  inline float getIntIntervalTick() const {
    return int_interval_tick;
  }

  inline void setIntIntervalSec(float sec) {
    setIntIntervalTick(TimerCls::secToTick(sec));
  }
  inline void setIntIntervalTick(float tick) {
    int_interval_tick = tick;
    _timer.setIntervalTick(int_clock_cc_idx, int_interval_tick);
  }

  inline uint32_t tick() {
    return _timer.tick();
  }

  inline bool isStart() const {
    return _is_start;
  }

  inline void irqHandler() {
    _timer.irqHandler();
    checkTimeout();
  }
};

}
}


