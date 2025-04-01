#pragma once

#include <igb_sdk/base.hpp>
#include <array>
#include <functional>

namespace igb {
namespace sdk {

template <size_t MAX_TIMER_SIZE>
struct SoftTimer {
  enum class State : uint8_t {
    inactive = 0,
    interval,
    oneshot
  };

  struct TimerState {
    uint32_t interval = 1000; // tick or msec
    std::function<void(void)> callback = nullptr;
    State state = State::inactive;
    uint32_t _start_msec = 0;
  };

  std::array<TimerState, MAX_TIMER_SIZE> states;

  inline void inactivate(size_t timer_idx) {
    if (timer_idx < MAX_TIMER_SIZE) {
      states[timer_idx].state = State::inactive;
    }
  }

  inline void inactivateAll() {
    for (auto& s : states) {
      s.state = State::inactive;
    }
  }

  inline void activate(size_t timer_idx, State state = State::interval) {
    if (timer_idx < MAX_TIMER_SIZE) {
      states[timer_idx].state = state;
    }
  }

  inline void changeInterval(size_t timer_idx, uint32_t interval) {
    if (timer_idx < MAX_TIMER_SIZE) {
      states[timer_idx].interval = interval;
    }
  }

  inline size_t intervalCallback(uint32_t interval, uint32_t current_msec_, auto&& cb) {
    size_t timer_idx = _getFreeState();
    auto& new_state = states[timer_idx];
    new_state.interval = interval;
    new_state.state = State::interval;
    new_state.callback = cb;
    new_state._start_msec = current_msec_;
    return timer_idx;
  }

  inline size_t oneshotCallback(uint32_t interval, uint32_t current_msec_, auto&& cb) {
    size_t timer_idx = _getFreeState();
    auto& new_state = states[timer_idx];
    new_state.interval = interval;
    new_state.state = State::oneshot;
    new_state.callback = cb;
    new_state._start_msec = current_msec_;
    return timer_idx;
  }

  void process(uint32_t current_msec_) {
    for (auto& s : states) {
      if (s.state != State::inactive) {
        uint32_t current_interval = current_msec_ - s._start_msec;
        if (current_interval >= s.interval) {
          if (s.callback) {
            s.callback();
          }
          if (s.state == State::oneshot) {
            s.state = State::inactive;
          } else {
            s._start_msec += s.interval;
          }
        }
      }
    }
  }

  size_t _getFreeState() {
    for (size_t i = 0; i < MAX_TIMER_SIZE; ++i) {
      auto& s = states[i];
      if (s.state == State::inactive) {
        return i;
      }
    }
    // oneshot のものから優先的に消す
    for (size_t i = 0; i < MAX_TIMER_SIZE; ++i) {
      auto& s = states[i];
      if (s.state == State::oneshot) {
        return i;
      }
    }
    return 0; // どれにもマッチしない場合は最初のStateを返す
  }
};

struct SoftTimerSingle {
  SoftTimer<1> base;

  void inactivate() {
    base.inactivate(0);
  }

  void activate() {
    base.activate(0);
  }

  inline void start() {
    activate();
  }

  inline void stop() {
    inactivate();
  }

  void changeInterval(uint32_t interval) {
    base.changeInterval(0, interval);
  }

  size_t intervalCallback(uint32_t interval, uint32_t current_msec_, auto&& callback) {
    return base.intervalCallback(interval, current_msec_, callback);
  }

  size_t oneshotCallback(uint32_t interval, uint32_t current_msec_, auto&& callback) {
    return base.oneshotCallback(interval, current_msec_, callback);
  }

  void process(uint32_t current_msec_) {
    base.process(current_msec_);
  }
};

}
}

