#pragma once

#include <functional>
#include <algorithm>
#include <numeric>
#include <tuple>
#include <igb_util/for_each.hpp>

namespace igb {
namespace sdk {

enum class ClockSyncAlgorithm {
  jump = 0,
  smooth
};

//- 位相補正処理
//  - クロック抜けを防止するためにint&ext clock syncの処理を見直し
//    - クロックの位相差を補正する処理を追加
//      - クロックイベントの発行数が変化しないよう、位相遅れや進みを検出する
//
//- モジュレーション処理
//  - クロックモジュレーションはモジュレータークラスを型パラメータに持つ
//    - モジュレーターは0-1.0の区間の値を入力すると、位相の進み/遅れ量を返すファンクタ
//  - 各クロックステートにクロックモジュレーション用のステートを追加する。
//    - クロックモジュレーション用のステートは、メインステートとのクロック位相差がモジュレーション量に応じて変わるようにnextCcを調整する
//
//- リアルタイム変更時の例外処理
//  - クロックモジュレーションのパラメータが変更された場合は現在の周期の再生が終わった直後に反映される
//    - アプリケーション側の実装
//      - ライブラリはモジュレーションの周期中の位置をコールバック関数の引数に渡す 
//  - ステップレングスや手動＆外部トリガーリセットが途中で行われた場合、クロックモジュレーションされていないクロックを基準として処理を行う。結果として間に合わないクロックについてはスキップされる。
//    - アプリケーションにはコールバック関数にmodクロックかnormalクロックかのフラグを引数として渡される
//      - アプリケーションはモジュレーションの周期を、リセットのタイミングでリセットする義務を負う
//        - モジュレーションのリセットが行われた時、モジュレーションステートはメインステートと同期される

//template <class T>
//concept ClockModifier = requires (T& x) {
//  typename T::Config;
//};

// TimerCls = HardTimerStm32, etc...
template<typename TimerCls, uint32_t out_clocks_size, uint32_t src_clocks_size, typename ClockModType, ClockSyncAlgorithm sync_algorithm = ClockSyncAlgorithm::jump, typename float_t = float>
struct SeqSyncableModClock {
  constexpr static uint32_t timeout_tick = (uint32_t)TimerCls::secToTick(10.0f);
  static_assert(TimerCls::cc_ch_count >= (out_clocks_size + 1), "cc ch size is not sufficient.");
  constexpr static uint32_t int_clock_cc_idx = TimerCls::cc_ch_count - 1;

  float_t bpm = 120.0f;
  float_t int_interval_tick = 0.0f;

  struct ClockConf {
    uint16_t step_per_beat = 4;
    std::function<void(void)> on_update = [](){};
  };
  struct SrcConf {
    uint16_t clock_per_beat = 4;
    float_t filter_coeff = 0.0f; // 0 == filter off, 0 .. 1.0f == filter on
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
    float_t interval_tick = TimerCls::secToTick(SeqSyncableModClock::bpmToIntervalSec(120.0f, step_per_beat));
    std::function<void(void)> on_update = [](){};

    uint16_t clock_per_beat = 4;
    float_t filter_coeff = 0.0f;

    uint16_t clock_div = 1;

    // for int&ext clock sync
    uint16_t clk_count = 0; // clear when received reset event
    uint16_t clk_count_max = 0;
    uint16_t ipl_count = 0;
    uint16_t ipl_count_max = 0;
    float_t clock_to_step_coeff = 0.0f;
    uint32_t clk_count_0_tick = 0;
    uint32_t ipl_prev_tick = 0;

    ClockModType clock_mod;
    uint16_t clock_mod_cycle = 0;
    uint16_t clock_mod_idx = 0;
    float_t prev_clock_mod_phase_diff = 0.0;
    float_t base_interval_tick = 0.0;

    inline float_t clockModulatedIntervalTick(float_t t) {
      if (!isClockModulated()) {
        return t;
      }
      clock_mod_idx = (clock_mod_idx + 1) % clock_mod_cycle;
      float_t clock_mod_phase = (float_t)clock_mod_idx / (float_t)clock_mod_cycle;
      float_t phase_diff = clock_mod.getPhaseDiff(clock_mod_phase);
      float_t phase_diff_delta = phase_diff - prev_clock_mod_phase_diff;
      float_t delta_mod_tick = phase_diff_delta * (float_t)t * (float_t)clock_mod_cycle * 0.5;
      float_t interval_tick_result = std::clamp<float_t>((float_t)t - delta_mod_tick, 1.0, (float_t)0x7FFFFFFF);
      prev_clock_mod_phase_diff = phase_diff;
      return interval_tick_result;
    }

    inline float_t expectedIntervalTick(float_t expected_clock_interval_tick) {
      base_interval_tick = (float_t)expected_clock_interval_tick * (float_t)clock_to_step_coeff;
      return base_interval_tick;
    }

    inline void resetCounts() {
      clk_count = 0;
      ipl_count = 0;
      clock_mod_idx = 0;
      prev_clock_mod_phase_diff = 0.0;
    }

    inline void _updateCompletionCount() {
      auto cpb = clock_per_beat * clock_div;
      const auto gcd_num = std::gcd(step_per_beat, cpb);
      if (isClockModulated()) {
        uint16_t ipl_count_size = (step_per_beat / gcd_num);
        uint16_t lcm_num = std::lcm(clock_mod_cycle, ipl_count_size);
        uint16_t rate = lcm_num / ipl_count_size;
        ipl_count_size *= rate;
        clk_count_max = (cpb / gcd_num) * rate - 1;
        ipl_count_max = ipl_count_size - 1;
      } else {
        clk_count_max = (cpb / gcd_num) - 1;
        ipl_count_max = (step_per_beat / gcd_num) - 1;
      }
      clock_to_step_coeff = (float_t)(cpb) / (float_t)step_per_beat;
    }

    inline void changeClockModCycle(uint16_t new_clock_mod_cycle) {
      clock_mod_cycle = new_clock_mod_cycle;
      _updateCompletionCount();
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

    inline float_t getSpeedRate() const {
      return (float_t)(ipl_count_max + 1) / (float_t)(clk_count_max + 1);
    }

    inline float_t calcPhaseDiff(uint32_t _tick) const {
      uint32_t clk_count_from0 = (clk_count_max - clk_count);
      float_t target_phase = (float_t)clk_count_from0 / (float_t)(clk_count_max + 1);
      uint32_t ipl_count_from0 = (ipl_count_max - ipl_count);
      // NOTE: ここで interval_tick は clock modulation の結果毎回異なりうる！
      //       クロックディバイドとクロックモジュレーションが有効な場合、
      //       この問題により、現在の位相を正確に算出できない。
      float_t phase = (
        (float_t)ipl_count_from0 + ((float_t)(_tick - ipl_prev_tick) / interval_tick)
      ) / (float_t)(ipl_count_max + 1);
      if (clock_mod_cycle > 1) {
        target_phase = std::clamp<float_t>(target_phase + prev_clock_mod_phase_diff, 0.0, 1.0); 
      }
      float_t phase_diff = phase - target_phase;
      return phase_diff;
    }

    inline bool isClockModulated() const {
      return (clock_mod_cycle > 1);
    }
  };

  struct ExtSrcState {
    bool is_first_clock_arrived = false;
    uint32_t prev_tick = 0;
    float_t expected_clock_interval_tick = 0.0; // 0 == not expected
  };

  TimerCls _timer;
  std::array<ClockState, out_clocks_size> _clockStates;
  std::array<SrcConf, src_clocks_size> _src_confs;
  ExtSrcState _ext_src_state;
  uint8_t _selected_src_clock_idx = 0;
  uint8_t _selected_int_clock_idx = 0;
  bool _int_active = false;
  bool _is_start = false;

  void init(float_t _bpm, auto&& clockConfs, auto&& srcConfs) {
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

  inline void setBpm(float_t _bpm) {
    bpm = _bpm;
    float_t interval_sec = bpmToIntervalSec(bpm, _src_confs[_selected_int_clock_idx].clock_per_beat);
    setIntIntervalSec(interval_sec);
  }

  inline float_t getBpm() {
    return bpm;
  }

  inline void _initClockConf(uint8_t idx, auto&& conf) {
    _clockStates[idx].changeStepPerBeat(conf.step_per_beat);
    _clockStates[idx].step_per_beat = conf.step_per_beat;
    float_t interval_sec = bpmToIntervalSec(bpm, conf.step_per_beat);
    _clockStates[idx].interval_tick = TimerCls::secToTick(interval_sec);
    _clockStates[idx].on_update = conf.on_update;
    _timer.setIntervalTick(idx, _clockStates[idx].interval_tick);
    _timer.setupTimer(idx, interval_sec,
      [this, idx](){
        onUpdateIntTimer(idx, _timer.getCcTick(idx));
        //onUpdateIntTimer(idx, tick());
      }
    );
  }

  inline void _initIntClock() {
    if (_selected_int_clock_idx >= src_clocks_size) { return; }
    float_t interval_sec = bpmToIntervalSec(bpm, _src_confs[_selected_int_clock_idx].clock_per_beat);
    setIntIntervalSec(interval_sec);
    _timer.setupTimer(int_clock_cc_idx, interval_sec, [this](){
      receiveClock(_selected_int_clock_idx, _timer.getCcTick(int_clock_cc_idx));
      //receiveClock(_selected_int_clock_idx, tick());
    });
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
      for (uint32_t idx = 0; idx < out_clocks_size; ++idx) {
        auto& state = _clockStates[idx];
        const uint32_t t = state.expectedIntervalTick(int_interval_tick);
        //state.interval_tick = (state.isClockModulated() ? state.clockModulatedIntervalTick(t) : t);
        state.interval_tick = t;
        //_timer.setIntervalTick(idx, state.interval_tick);
      }
      _timer.setIntervalTick(int_clock_cc_idx, int_interval_tick);
      uint32_t t = tick();
      _timer.start(int_clock_cc_idx, t);
      _is_start = true;
      receiveClock(_selected_int_clock_idx, t); // trigger first clock
    } else {
      _is_start = true;
    }
  }

  inline void stop() {
    if (!_is_start) {
      return;
    }
    _timer.stop(int_clock_cc_idx);
    for (uint8_t i = 0; i < out_clocks_size; ++i) {
      _timer.stop(i);
    }
    _is_start = false;
    resetClocks();
    _ext_src_state.is_first_clock_arrived = false;
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

  inline void receiveClock(uint8_t idx, uint32_t _tick) {
    if (_selected_src_clock_idx != idx) {
      return;
    }
    const uint32_t t = _tick;

    if (_ext_src_state.is_first_clock_arrived) {
      const uint32_t diff_tick = (t - _ext_src_state.prev_tick);
      expectClockInterval(diff_tick);
    }

    for (uint8_t i = 0; i < out_clocks_size; ++i) {
      updateClockState(i, t);
    }

    if (!_ext_src_state.is_first_clock_arrived) {
      _ext_src_state.is_first_clock_arrived = true;
    }
    _ext_src_state.prev_tick = t;
  }

  inline void updateClockState(uint8_t idx, uint32_t _tick) {
    if (!_is_start) { return; }
    if (sync_algorithm == ClockSyncAlgorithm::smooth) {
      _updateClockStateSmoothSync(idx, _tick);
    } else { // jump
      _updateClockStateJumpSync(idx, _tick);
    }
  }

  inline void _updateClockStateSmoothSync(uint8_t idx, uint32_t _tick) {
    auto& state = _clockStates[idx];
    if (state.clk_count == 0) {
      state.clk_count += state.clk_count_max;
      if (!_timer.isStart(idx)) {
        _timer.start(idx, _tick);
        onUpdateIntTimer(idx, _tick);
      }
      state.clk_count_0_tick = _tick;
    } else {
      state.clk_count--;
    }
    if (_ext_src_state.is_first_clock_arrived) {
      float_t t = state.expectedIntervalTick(_ext_src_state.expected_clock_interval_tick);
      t = state.clockModulatedIntervalTick(t);
      const float_t phase_diff = state.calcPhaseDiff(_tick); // tickベースのクロック位相差
      const float_t phase_diff_per_interval = phase_diff / t;
      float_t adj_rate = std::clamp<double>(1.0 + (phase_diff_per_interval * 0.3), 0.7, 1.3); // インターバル補正率
      state.interval_tick = t * adj_rate;
      _timer.setIntervalTick(idx, state.interval_tick);
    }
  }

  inline void _updateClockStateJumpSync(uint8_t idx, uint32_t _tick) {
    auto& state = _clockStates[idx];
    if (state.clk_count == 0) {
      while (state.ipl_count > 0) {
        // consume remains events
        state.interval_tick = state.clockModulatedIntervalTick(state.base_interval_tick);
        _timer.setIntervalTick(idx, state.interval_tick);
        state.on_update();
        state.ipl_count--;
      }
      state.interval_tick = state.clockModulatedIntervalTick(state.base_interval_tick);
      _timer.setIntervalTick(idx, state.interval_tick);
      state.on_update();
      state.ipl_count = state.ipl_count_max;
      state.clk_count = state.clk_count_max;
      if (state.ipl_count != 0 && !_timer.isStart(idx)) {
        _timer.start(idx, _tick);
      }
      state.clk_count_0_tick = _tick;
    } else {
      state.clk_count--;
    }
    if (_ext_src_state.is_first_clock_arrived) {
      const float_t t = state.expectedIntervalTick(_ext_src_state.expected_clock_interval_tick);
      if (state.isClockModulated()) {
      } else {
        state.interval_tick = t;
        _timer.setIntervalTick(idx, state.interval_tick);
      }
    }
  }

  inline void expectClockInterval(uint32_t diff_tick) {
    if (diff_tick < timeout_tick) {
      const auto filter_coeff = _src_confs[_selected_src_clock_idx].filter_coeff;
      if (_ext_src_state.expected_clock_interval_tick > 0 || filter_coeff > 0.0f) { 
        _ext_src_state.expected_clock_interval_tick = (_ext_src_state.expected_clock_interval_tick * filter_coeff) + ((float_t)diff_tick * (1.0f - filter_coeff));
      } else {
        _ext_src_state.expected_clock_interval_tick = diff_tick;
      }
    }
  }

  void onUpdateIntTimer(uint8_t clock_idx, uint32_t t) {
    auto& state = _clockStates[clock_idx];
    if (sync_algorithm == ClockSyncAlgorithm::jump) {
      if (state.ipl_count > 0) {
        state.ipl_count--;
        state.interval_tick = state.clockModulatedIntervalTick(state.base_interval_tick);
        _timer.setIntervalTick(clock_idx, state.interval_tick);
        state.on_update();
      } else {
        _timer.stop(clock_idx); 
      }
    } else {
      state.on_update();
      state.interval_tick = state.clockModulatedIntervalTick(state.base_interval_tick);
      _timer.setIntervalTick(clock_idx, state.interval_tick);
      state.ipl_prev_tick = t;
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

  constexpr static float_t bpmToIntervalSec(float_t _bpm, uint16_t step_per_beat) {
    const float_t sec_per_beat = 60.0f / _bpm;
    const float_t sec_per_step = sec_per_beat / step_per_beat;
    return sec_per_step;
  }

  constexpr static float_t intervalSecToBpm(float_t interval_sec, uint16_t step_per_beat) {
    const float_t sec_per_beat = interval_sec * (float_t)step_per_beat;
    return 60.0f / sec_per_beat;
  }

  inline float_t getIntervalSec(uint8_t idx) const {
    return _timer.getIntervalSec(idx);
  }

  inline float_t getIntervalTick(uint8_t idx) const {
    return _timer.getIntervalTick(idx);
  }

  inline float_t getIntIntervalSec() const {
    return TimerCls::tickToSec(int_interval_tick);
  }

  inline float_t getIntIntervalTick() const {
    return int_interval_tick;
  }

  inline void setIntIntervalSec(float_t sec) {
    setIntIntervalTick(TimerCls::secToTick(sec));
  }
  inline void setIntIntervalTick(float_t tick) {
    int_interval_tick = tick;
    _timer.setIntervalTick(int_clock_cc_idx, int_interval_tick);
  }
  inline void setClockModCycle(uint8_t idx, uint16_t cycle) {
    auto& state = _clockStates[idx]; 
    state.changeClockModCycle(cycle);
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

  inline void process() {
    _timer.process();
    checkTimeout();
  }

  ClockModType& clockMod(uint8_t idx) {
    return _clockStates[idx].clock_mod;
  }
  const ClockModType& clockMod(uint8_t idx) const {
    return _clockStates[idx].clock_mod;
  }
};

}
}


