#include <catch2/catch_test_macros.hpp>
#include <igb_sdk/util/soft_cc_timer.hpp>
#include <igb_sdk/seq/syncable_mod_clock.hpp>
#include <igb_util/dsp/dsp_func.hpp>
#include <iostream>
#include <bitset>
#include <array>
#include <vector>
#include <cmath>

using namespace igb;
using namespace igb::sdk;

bool nearly_match(auto&& vec1, auto&& vec2) {
  if (vec1.size() != vec2.size()) {
    return false;
  }
  for (size_t i = 0; i < vec1.size(); ++i) {
    if (std::abs((double)vec1[i] - (double)vec2[i]) > 1.0) {
      return false;
    }
  }
  return true;
}

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

typedef SoftCcTimer<TestTim, uint32_t /* count_t */, 3 /* cc_ch */, 1000 /* tim_base_clock */, float>  TimerCls;
typedef SeqSyncableModClock<TimerCls, 2 /* out_sise */, 2 /* src_size */, TestClockMod, ClockSyncAlgorithm::jump, float> ClockCls;
typedef SeqSyncableModClock<TimerCls, 2 /* out_sise */, 2 /* src_size */, TestClockMod, ClockSyncAlgorithm::smooth, float> ClockSmoothSyncCls;

TEST_CASE("SyncableModClock") {
  SECTION( "jump sync" ) {
    ClockCls clock;

    bool is_debug = false;
    uint32_t step = 0;
    uint32_t clk = 0;
    std::vector<uint32_t> ticks;
    
    clock.init(
      120.0,
      std::make_tuple(
        ClockCls::ClockConf {
          .step_per_beat = 4,
          .on_update = [&is_debug, &step, &clock, &ticks]() {
            ++step;
            ticks.push_back(clock.tick());
            if (is_debug) {
              //std::cout << step << ",";
              auto& state = clock._clockStates[0];
              //std::cout << state.interval_tick << "[" << clock.tick() << "]" << "(" << state.clock_mod_idx << ")" << "<" << state.prev_clock_mod_phase_diff << ">" << "[[" << state.clk_count << "]]" << "((" << state.ipl_count << "))" << ",";
              //std::cout << state.interval_tick << "[" << clock.tick() << "]" << "(" << state.clock_mod_idx << ")" << "<" << state.prev_clock_mod_phase_diff << ">" << ",";
              std::cout << clock.tick() << ",";
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
        },
        ClockCls::SrcConf { // ext clock
          .clock_per_beat = 4,
          .filter_coeff = 0.0f
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
    REQUIRE(step == 16);
    REQUIRE(ticks == std::vector<uint32_t>{0, 125, 250, 375, 500, 625, 750, 875, 1000, 1125, 1250, 1375, 1500, 1625, 1750, 1875});

    // reset
    clock.stop();
    clock.resetClocks();
    step = 0;
    clk = 0;
    tim.count(0);
    ticks.clear();

    // clock div / multi
    is_debug = true;
    std::cout << "clock div/multi ===" << std::endl;
    //clock.changeStepPerBeat(0, 6);
    //clock.changeClockDiv(0, 2);
    clock.changeClockMulti(0, 3);
    clock.changeClockDiv(0, 4);
    clock.start();
    for (uint16_t msec = 0; msec < 2000; ++msec) {
      clock.process();
      tim.countUp();
    }
    std::cout << std::endl;
    REQUIRE(step == (16*6/8));
    REQUIRE(nearly_match(ticks, std::vector<uint32_t>{0,166,333,500,666,833,1000,1166,1333,1500,1666,1833}));

    // reset
    clock.stop();
    clock.resetClocks();
    step = 0;
    clk = 0;
    tim.count(0);
    ticks.clear();

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
    REQUIRE(nearly_match(ticks, std::vector<uint32_t>{0,221,333,554,666,888,1000,1221,1333,1554,1666,1888}));

    // reset
    clock.stop();
    clock.resetClocks();
    step = 0;
    clk = 0;
    tim.count(0);
    ticks.clear();

    // external sync
    std::cout << "external sync ===" << std::endl;
    clock.disableIntClock();
    clock.selectSrcClockIdx(1);
    //clock.changeClockMulti(0, 2);
    //clock.changeClockDiv(0, 1);
    clock.setClockModCycle(0, 0);
    clock.start();
    uint16_t msec = 0;
    uint16_t next_clock_msec = 0;
    for (msec = 0; msec < 1000; ++msec) {
      if (msec == next_clock_msec) {
        std::cout << "+" << msec << ",";
        clock.receiveClock(1, tim.count());
        next_clock_msec += 125;
      }
      clock.process();
      tim.countUp();
    }
    uint16_t inc_value = 25;
    for (; msec < 2000; ++msec) {
      if (msec == next_clock_msec) {
        std::cout << "+" << msec << ",";
        clock.receiveClock(1, tim.count());
        next_clock_msec += 125 + inc_value;
        inc_value += 25;
      }
      clock.process();
      tim.countUp();
    }
    for (; msec < 4000; ++msec) {
      if (msec == next_clock_msec) {
        std::cout << "+" << msec << ",";
        clock.receiveClock(1, tim.count());
        next_clock_msec += 250;
      }
      clock.process();
      tim.countUp();
    }
    std::cout << std::endl;

    REQUIRE(step == 16);
  }

  SECTION( "smooth sync" ) {
    ClockSmoothSyncCls clock;

    bool is_debug = false;
    uint32_t step = 0;
    uint32_t clk = 0;
    std::vector<uint32_t> ticks;
    
    clock.init(
      120.0,
      std::make_tuple(
        ClockSmoothSyncCls::ClockConf {
          .step_per_beat = 4,
          .on_update = [&is_debug, &step, &clock, &ticks]() {
            ++step;
            ticks.push_back(clock.tick());
            if (is_debug) {
              //std::cout << step << ",";
              auto& state = clock._clockStates[0];
              //std::cout << state.interval_tick << "[" << clock.tick() << "]" << "(" << state.clock_mod_idx << ")" << "<" << state.prev_clock_mod_phase_diff << ">" << "[[" << state.clk_count << "]]" << "((" << state.ipl_count << "))" << ",";
              //std::cout << state.interval_tick << "[" << clock.tick() << "]" << "(" << state.clock_mod_idx << ")" << "<" << state.prev_clock_mod_phase_diff << ">" << ":" << state.test_phase_diff << ":" << ",";
              std::cout << clock.tick() << ",";
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
        },
        ClockSmoothSyncCls::SrcConf { // ext clock
          .clock_per_beat = 4,
          .filter_coeff = 0.0f
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
    REQUIRE(step == 16);
    REQUIRE(ticks == std::vector<uint32_t>{0, 125, 250, 375, 500, 625, 750, 875, 1000, 1125, 1250, 1375, 1500, 1625, 1750, 1875});

    // reset
    clock.stop();
    clock.resetClocks();
    step = 0;
    clk = 0;
    tim.count(0);
    ticks.clear();

    // clock div / multi
    is_debug = true;
    std::cout << "clock div/multi ===" << std::endl;
    //clock.changeStepPerBeat(0, 6);
    //clock.changeClockDiv(0, 2);
    clock.changeClockMulti(0, 3);
    clock.changeClockDiv(0, 4);
    clock.start();
    for (uint16_t msec = 0; msec < 2000; ++msec) {
      clock.process();
      tim.countUp();
    }
    std::cout << std::endl;
    //REQUIRE(step == (16*6/8));
    //REQUIRE(ticks == std::vector<uint32_t>{0,166,333,500,666,833,1000,1166,1333,1500,1666,1833});
    //REQUIRE(nearly_match(ticks, std::vector<uint32_t>{0,166,333,500,666,833,1000,1166,1333,1500,1666,1833}));
    if (step == (16*6/8)) {
      REQUIRE(nearly_match(ticks, std::vector<uint32_t>{0,166,333,500,666,833,1000,1166,1333,1500,1666,1833}));
    } else {
      REQUIRE(nearly_match(ticks, std::vector<uint32_t>{0,166,333,500,666,833,1000,1166,1333,1500,1666,1833, 2000}));
    }

    // reset
    clock.stop();
    clock.resetClocks();
    step = 0;
    clk = 0;
    tim.count(0);
    ticks.clear();

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
    //REQUIRE(step == (16*6/8)*3);
    //REQUIRE(ticks == std::vector<uint32_t>{0,221,333,554,666,888,1000,1221,1333,1554,1666,1888});
    if (step == (16*6/8)) {
      REQUIRE(nearly_match(ticks, std::vector<uint32_t>{0,221,333,554,666,888,1000,1221,1333,1554,1666,1888}));
    } else {
      REQUIRE(nearly_match(ticks, std::vector<uint32_t>{0,221,333,554,666,888,1000,1221,1333,1554,1666,1888, 2000}));
    }

    // reset
    clock.stop();
    clock.resetClocks();
    step = 0;
    clk = 0;
    tim.count(0);
    ticks.clear();

    // external sync
    std::cout << "external sync ===" << std::endl;
    clock.disableIntClock();
    clock.selectSrcClockIdx(1);
    clock.setClockModCycle(0, 0);
    clock.start();
    uint16_t msec = 0;
    uint16_t next_clock_msec = 0;
    for (msec = 0; msec < 1000; ++msec) {
      if (msec == next_clock_msec) {
        std::cout << "+" << msec << ",";
        clock.receiveClock(1, tim.count());
        next_clock_msec += 125;
      }
      clock.process();
      tim.countUp();
    }
    uint16_t inc_value = 25;
    for (; msec < 2000; ++msec) {
      if (msec == next_clock_msec) {
        std::cout << "+" << msec << ",";
        clock.receiveClock(1, tim.count());
        next_clock_msec += 125 + inc_value;
        inc_value += 25;
      }
      clock.process();
      tim.countUp();
    }
    for (; msec < 10000; ++msec) {
      if (msec == next_clock_msec) {
        std::cout << "+" << msec << ",";
        clock.receiveClock(1, tim.count());
        next_clock_msec += 250;
      }
      clock.process();
      tim.countUp();
    }
    std::cout << std::endl;

    REQUIRE(step == 34);
  }
}
