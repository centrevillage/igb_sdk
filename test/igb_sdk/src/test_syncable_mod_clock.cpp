#include <catch2/catch_test_macros.hpp>
#include <igb_sdk/util/soft_cc_timer.hpp>
#include <igb_sdk/seq/syncable_mod_clock.hpp>
#include <igb_util/dsp/dsp_func.hpp>
#include <iostream>
#include <bitset>
#include <array>

using namespace igb;
using namespace igb::sdk;

struct TestTim {
  uint32_t _count = 0;
  bool _is_start = false;

  void count(uint32_t c) {
    _count = c;
  }

  uint32_t count() const {
    return _count;
  }

  void countUp() {
    ++_count;
  }

  bool enable() const {
    return _is_start;
  }

  void start() {
    _is_start = true;
  }

  void stop() {
    _is_start = false;
  }
};

struct TestClockMod {
  float _amp = 1.0;

  void setAmp(float amp) {
    _amp = amp;
  }

  float getPhaseDiff(float clock_mod_phase) const {
    return -(_amp * dsp::tri_wave_uni(clock_mod_phase));
  }
};

typedef SoftCcTimer<TestTim, uint32_t /* count_t */, 3 /* cc_ch */, 1000 /* tim_base_clock */, double>  TimerCls;
typedef SeqSyncableModClock<TimerCls, 2 /* out_sise */, 1 /* src_size */, TestClockMod, ClockSyncAlgorithm::jump, double> ClockCls;
typedef SeqSyncableModClock<TimerCls, 2 /* out_sise */, 1 /* src_size */, TestClockMod, ClockSyncAlgorithm::smooth, double> ClockSmoothSyncCls;

TEST_CASE("SyncableModClock") {
  SECTION( "jump sync" ) {
    ClockCls clock;

    bool is_debug = false;
    uint32_t step = 0;
    uint32_t clk = 0;
    
    clock.init(
      120.0,
      std::make_tuple(
        ClockCls::ClockConf {
          .step_per_beat = 4,
          .on_update = [&is_debug, &step, &clock]() {
            ++step;
            if (is_debug) {
              //std::cout << step << ",";
              auto& state = clock._clockStates[0];
              //std::cout << state.interval_tick << "[" << clock.tick() << "]" << "(" << state.clock_mod_idx << ")" << "<" << state.prev_clock_mod_phase_diff << ">" << "[[" << state.clk_count << "]]" << "((" << state.ipl_count << "))" << ",";
              std::cout << state.interval_tick << "[" << clock.tick() << "]" << "(" << state.clock_mod_idx << ")" << "<" << state.prev_clock_mod_phase_diff << ">" << ",";
            }
          }
        },
        ClockCls::ClockConf {
          .step_per_beat = 4,
          .on_update = [&is_debug, &clk, &clock]() {
            ++clk;
            if (is_debug) {
              //std::cout << "+" << clk << ",";
              //std::cout << "+" << clock._clockStates[1].interval_tick << ",";
            }
          }
        }
      ),
      std::make_tuple(
        ClockCls::SrcConf { // int clock
          .clock_per_beat = 4,
          .filter_coeff = 0.0f,
          .internal = true
        }
      )
    );
    clock.selectSrcClockIdx(0);
    clock.enableIntClock();
    auto& tim = clock._timer._tim;
    tim.count(0);
    std::cout << "clock ===" << std::endl;
    is_debug = true;
    clock.start();
    for (uint16_t msec = 0; msec < 2000; ++msec) {
      clock.process();
      tim.countUp();
    }
    std::cout << std::endl;
    //std::cout << "expected interval: " << clock._ext_src_state.expected_clock_interval_tick << std::endl;
    //std::cout << "int interval tick: " << clock.int_interval_tick << std::endl;
    //std::cout << "int timer interval tick: " << clock._timer._cc_states[2].interval_tick << std::endl;
    //std::cout << "int cc tick: " << clock._timer.getCcTick(2) << std::endl;
    //std::cout << "tick: " << clock.tick() << std::endl;
    //std::cout << "bpm: " << clock.bpm << std::endl;
    REQUIRE(step == 16);

    // reset
    clock.stop();
    clock.resetClocks();
    step = 0;
    clk = 0;
    tim.count(0);

    // clock div / multi
    is_debug = true;
    std::cout << "clock div/multi ===" << std::endl;
    clock.changeStepPerBeat(0, 6);
    clock.changeClockDiv(0, 2);
    clock.start();
    for (uint16_t msec = 0; msec < 2000; ++msec) {
      clock.process();
      tim.countUp();
    }
    std::cout << std::endl;
    REQUIRE(step == (16*6/8));

    // reset
    clock.stop();
    clock.resetClocks();
    step = 0;
    clk = 0;
    tim.count(0);

    // clock mod
    std::cout << "clock div/multi with clock mod ===" << std::endl;
    clock.setClockModCycle(0, 2);
    clock.clockMod(0).setAmp(0.33);
    clock.start();
    for (uint16_t msec = 0; msec < 2000; ++msec) {
      clock.process();
      tim.countUp();
    }
    std::cout << std::endl;
    REQUIRE(step == (16*6/8));
  }

  SECTION( "smooth sync" ) {
    ClockSmoothSyncCls clock;

    bool is_debug = false;
    uint32_t step = 0;
    uint32_t clk = 0;
    
    clock.init(
      120.0,
      std::make_tuple(
        ClockSmoothSyncCls::ClockConf {
          .step_per_beat = 4,
          .on_update = [&is_debug, &step, &clock]() {
            ++step;
            if (is_debug) {
              //std::cout << step << ",";
              auto& state = clock._clockStates[0];
              //std::cout << state.interval_tick << "[" << clock.tick() << "]" << "(" << state.clock_mod_idx << ")" << "<" << state.prev_clock_mod_phase_diff << ">" << "[[" << state.clk_count << "]]" << "((" << state.ipl_count << "))" << ",";
              std::cout << state.interval_tick << "[" << clock.tick() << "]" << "(" << state.clock_mod_idx << ")" << "<" << state.prev_clock_mod_phase_diff << ">" << ":" << state.test_phase_diff << ":" << ",";
            }
          }
        },
        ClockSmoothSyncCls::ClockConf {
          .step_per_beat = 4,
          .on_update = [&is_debug, &clk, &clock]() {
            ++clk;
            if (is_debug) {
              //std::cout << "+" << clk << ",";
              //std::cout << "+" << clock._clockStates[1].interval_tick << ",";
            }
          }
        }
      ),
      std::make_tuple(
        ClockSmoothSyncCls::SrcConf { // int clock
          .clock_per_beat = 4,
          .filter_coeff = 0.0f,
          .internal = true
        }
      )
    );
    clock.selectSrcClockIdx(0);
    clock.enableIntClock();
    auto& tim = clock._timer._tim;
    tim.count(0);
    std::cout << "clock ===" << std::endl;
    is_debug = true;
    clock.start();
    for (uint16_t msec = 0; msec < 2000; ++msec) {
      clock.process();
      tim.countUp();
    }
    std::cout << std::endl;
    //std::cout << "expected interval: " << clock._ext_src_state.expected_clock_interval_tick << std::endl;
    //std::cout << "int interval tick: " << clock.int_interval_tick << std::endl;
    //std::cout << "int timer interval tick: " << clock._timer._cc_states[2].interval_tick << std::endl;
    //std::cout << "int cc tick: " << clock._timer.getCcTick(2) << std::endl;
    //std::cout << "tick: " << clock.tick() << std::endl;
    //std::cout << "bpm: " << clock.bpm << std::endl;
    REQUIRE(step == 16);

    // reset
    clock.stop();
    clock.resetClocks();
    step = 0;
    clk = 0;
    tim.count(0);

    // clock div / multi
    is_debug = true;
    std::cout << "clock div/multi ===" << std::endl;
    clock.changeStepPerBeat(0, 6);
    clock.changeClockDiv(0, 2);
    clock.start();
    for (uint16_t msec = 0; msec < 2000; ++msec) {
      clock.process();
      tim.countUp();
    }
    std::cout << std::endl;
    REQUIRE(step == (16*6/8));

    // reset
    clock.stop();
    clock.resetClocks();
    step = 0;
    clk = 0;
    tim.count(0);

    // clock mod
    std::cout << "clock div/multi with clock mod ===" << std::endl;
    clock.setClockModCycle(0, 2);
    clock.clockMod(0).setAmp(0.33);
    clock.start();
    for (uint16_t msec = 0; msec < 2000*3; ++msec) {
      clock.process();
      tim.countUp();
    }
    std::cout << std::endl;
    REQUIRE(step == (16*6/8)*3);
  }
}
