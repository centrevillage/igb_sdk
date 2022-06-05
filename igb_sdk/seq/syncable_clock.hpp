#ifndef IGB_SDK_SEQ_SYNCABLE_CLOCK_H
#define IGB_SDK_SEQ_SYNCABLE_CLOCK_H

#include <functional>
#include <numeric>

namespace igb {
namespace sdk {

// TimerCls = HardTimerStm32, etc...
template<typename TimerCls, uint32_t ext_clocks_size = 2>
struct SeqSyncableClock {
  constexpr static uint32_t timeout_tick = (uint32_t)TimerCls::secToTick(10.0f);
  static_assert(TimerCls::sub_timer_count > 0, "sub_timer_count must greater than 0");
  constexpr static uint32_t sub_timer_interval_tick = (uint32_t)TimerCls::secToTick(1.0f);
  constexpr static uint8_t sub_timer_idx = 0;

  std::function<void(void)> on_update = [](){};

  uint16_t step_per_beat = 4;
  float _interval_tick = TimerCls::secToTick(bpmToIntervalSec(120.0f));

  struct IntConf {
    float bpm = 120.0f;
    uint16_t step_per_beat = 4;
    std::function<void(void)> on_update = [](){};
  };
  struct ExtConf {
    uint16_t clock_per_beat = 4;
    float filter_coeff = 0.0f; // 0 == filter off, 0 .. 1.0f == filter on
  };
  constexpr static ExtConf midi_clock_conf = ExtConf {
    .clock_per_beat = 24,
    .filter_coeff = 0.9f
  };
  constexpr static ExtConf trig_clock_conf = ExtConf {
    .clock_per_beat = 4,
    .filter_coeff = 0.0f
  };
  constexpr static ExtConf tap_clock_conf = ExtConf {
    .clock_per_beat = 1,
    .filter_coeff = 0.0f
  };

  struct ExtState {
    ExtConf conf;

    // for int&ext clock sync
    bool is_first_clock_arrived = false;
    uint16_t clk_count = 0; // clear when received reset event
    uint16_t clk_count_max = 0;
    uint16_t ipl_count = 0;
    uint16_t ipl_count_max = 0;

    // for bpm detection
    float expected_interval_tick = 0.0f; // 0 == not expected
    uint32_t prev_tick = 0;
  };

  TimerCls _timer;
  std::array<ExtState, ext_clocks_size> _extClockStates;
  uint8_t _selected_ext_clock_idx = 0;
  bool _is_active_internal = true;
  bool _is_start = false;

  void init(auto&& intConf, auto&&... confs) {
    initExtConf(0, confs...);

    changeStepPerBeat(intConf.step_per_beat);
    _interval_tick = TimerCls::secToTick(bpmToIntervalSec(intConf.bpm));
    on_update = intConf.on_update;

    // TODO: sec で渡すべきところで tick で渡しているのでバグ？
    _timer.init(_interval_tick, [this](){onUpdateIntTimer();});

    _timer.initSubTimer(sub_timer_idx, sub_timer_interval_tick, [this](){checkTimeout();});
    _timer.startSubTimer(sub_timer_idx);
  }

  inline void initExtConf(uint8_t idx) {}
  inline void initExtConf(uint8_t idx, auto&& first, auto&&... rest) {
    if (idx < ext_clocks_size) { 
      _initExtConf(idx, first);
      initExtConf(idx+1, rest...);
    }
  }
  inline void _initExtConf(uint8_t idx, auto&& conf) {
    _extClockStates[idx].conf = conf;
  }

  inline bool isIntActive() const {
    return _is_active_internal;
  }
  inline std::optional<uint8_t> getActiveExtClockIdx() const {
    if (isIntActive()) {
      return std::nullopt;
    }
    return _selected_ext_clock_idx;
  }
  inline void selectExtClockIdx(uint8_t idx) {
    _selected_ext_clock_idx = idx;
    _is_active_internal = false;
  }
  inline void selectIntClock() {
    _is_active_internal = true;
  }

  inline void start() {
    if (_is_start) {
      return;
    }
    resetClocks();
    if (isIntActive()) {
      _timer.setIntervalTick(_interval_tick);
      on_update();
      _timer.start();
    }
    _is_start = true;
  }

  inline void stop() {
    if (!_is_start) {
      return;
    }
    _is_start = false;
    if (isIntActive()) {
      _timer.stop();
    }
    resetClocks();
  }

  inline void resetClocks() {
    for (uint8_t i = 0; i < ext_clocks_size; ++i) {
      _extClockStates[i].clk_count = 0;
      _extClockStates[i].ipl_count = 0;
    }
  }

  inline void receiveExtClock(uint8_t idx) {
    const uint32_t t = _timer.tick();
    auto& state = _extClockStates[idx];
    if (_selected_ext_clock_idx == idx) {
      updateSelectedExtClockState(state);
    }
    expectInterval(state, t);
  }

  inline void updateSelectedExtClockState(auto& state) {
    if (_is_start && state.clk_count == 0) {
      // sync int&ext clock
      _timer.stop();
      state.ipl_count = state.ipl_count_max;
      state.clk_count = state.clk_count_max;
      on_update();
      if (state.is_first_clock_arrived) {
        _timer.setIntervalTick(state.expected_interval_tick);
      } else {
        _timer.setIntervalTick(_interval_tick);
      }
      if (state.ipl_count != 0) {
        _timer.start();
      }
    } else {
      state.clk_count--;
    }
  }

  inline void expectInterval(auto& state, uint32_t new_tick) {
    if (!state.is_first_clock_arrived) {
      state.is_first_clock_arrived = true;
    } else {
      const uint32_t diff_tick = (new_tick - state.prev_tick);
      if (diff_tick < timeout_tick) {
        const float bar_tick = (float)diff_tick * (float)state.conf.clock_per_beat;
        const float interval_tick = bar_tick / (float)step_per_beat;
        if (state.expected_interval_tick > 0.0f) { 
          const auto filter_coeff = state.conf.filter_coeff;
          state.expected_interval_tick = (state.expected_interval_tick * filter_coeff) + ((float)interval_tick * (1.0f - filter_coeff));
        } else {
          state.expected_interval_tick = interval_tick;
        }
      }
    }
    state.prev_tick = new_tick;
  }

  void onUpdateIntTimer() {
    if(isIntActive()) {
      _timer.setIntervalTick(_interval_tick);
    } else {
      auto& state = _extClockStates[_selected_ext_clock_idx];
      if (state.ipl_count > 0) {
        state.ipl_count--;
      } 
      if (state.ipl_count == 0) {
        _timer.stop(); 
      }
      _timer.setIntervalTick(state.expected_interval_tick);
    }
    on_update();
  }

  void checkTimeout() {
    const auto t = _timer.tick();
    for (auto& state : _extClockStates) {
      if (state.is_first_clock_arrived) {
        if ((t - state.prev_tick) > timeout_tick) {
          state.is_first_clock_arrived = false;
        }
      }
    }
  }

  // step_per_beat == clock_per_beat の時は補完不要なので ipl_count_max == 0
  // clock_per_beat % step_per_beat == 0 の時も補完不要なので ipl_count_max == 0、しかし clk_count_max != 0 の場合がありうる（clock divideが必要)
  // 上記のどちらでもない場合、少なくとも1bar単位では外部/内部クロックタイミングが一致する
  // ex:
  // ipl_count_max = 0; clk_count_max = 5; step_per_beat = 4, clock_per_beat = 24,
  // ipl_count_max = 0; clk_count_max = 0; step_per_beat = 4, clock_per_beat = 4,
  // ipl_count_max = 3; clk_count_max = 0; step_per_beat = 4, clock_per_beat = 1,
  // ipl_count_max = 7; clk_count_max = 0; step_per_beat = 8, clock_per_beat = 1,
  // ipl_count_max = 3; clk_count_max = 2; step_per_beat = 4, clock_per_beat = 3,
  // ipl_count_max = 2; clk_count_max = 3; step_per_beat = 3, clock_per_beat = 4,
  inline void changeStepPerBeat(uint16_t new_step_per_beat) {
    if (new_step_per_beat > 0) {
      step_per_beat = new_step_per_beat;
      for (auto& state : _extClockStates) {
        const auto clock_per_beat = state.conf.clock_per_beat;
        const auto mod1 = step_per_beat % clock_per_beat;
        const auto mod2 = clock_per_beat % step_per_beat;

        if (mod1 && mod2) { // sync interval is one bar
          state.clk_count_max = clock_per_beat - 1;
          state.ipl_count_max = step_per_beat - 1;
        } else if (mod1) {  // sync interval is one step
          state.clk_count_max = (clock_per_beat / step_per_beat) - 1;
          state.ipl_count_max = 0;
        } else if (mod2) {  // sync interval is one clock
          state.clk_count_max = 0;
          state.ipl_count_max = step_per_beat - 1;
        } else {            // step_per_beat == clock_per_beat
          state.clk_count_max = 0;
          state.ipl_count_max = 0;
        }
      }
    }
  }

  constexpr float bpmToIntervalSec(float bpm) {
    const float sec_per_beat = 60.0f / bpm;
    const float sec_per_step = sec_per_beat / (float)step_per_beat;
    return sec_per_step;
  }

  constexpr float intervalSecToBpm(float interval_sec) {
    const float sec_per_beat = interval_sec * (float)step_per_beat;
    return 60.0f / sec_per_beat;
  }

  inline float getIntervalSec() const {
    return _timer.getIntervalSec();
  }

  inline float getIntervalTick() const {
    return _timer.getIntervalTick();
  }

  inline float getIntIntervalSec() const {
    return TimerCls::tickToSec(_interval_tick);
  }

  inline float getIntIntervalTick() const {
    return _interval_tick;
  }

  inline void setIntIntervalSec(float sec) {
    setIntervalTick(TimerCls::secToTick(sec));
  }
  inline void setIntIntervalTick(float tick) {
    _interval_tick = tick;
  }
  inline void setIntIntervalTick(uint32_t tick) {
    _interval_tick = (float)tick;
  }

  inline uint32_t tick() {
    return _timer.tick();
  }

  inline bool isStart() const {
    return _is_start;
  }

  inline void irqHandler() {
    _timer.irqHandler();
  }

};

}
}


#endif /* IGB_SDK_SEQ_SYNCABLE_CLOCK_H */
