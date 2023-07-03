#pragma once

namespace igb::dsp {

#if !defined(IGB_FIXED_SAMPLING_RATE)
  struct Config {
    class Instance {
      private:
        Instance() = default;
        ~Instance() = default;

      public:
        uint32_t sampling_rate = 48000;
        float sampling_rate_f = 48000.0f;

        void setSamplingRate(uint32_t _sampling_rate) {
          sampling_rate = _sampling_rate;
          sampling_rate_f = (float)_sampling_rate;
        }

        static Instance& instance() {
          static Instance obj;
          return obj;
        }
    };

    static void setSamplingRate(uint32_t sampling_rate) {
      Instance::instance().setSamplingRate(sampling_rate);
    }

    static uint32_t getSamplingRate() {
      return Instance::instance().sampling_rate;
    }

    static float getSamplingRateF() {
      return Instance::instance().sampling_rate_f;
    }
  };
#else
  struct Config {
    constexpr static uint32_t getSamplingRate() {
      return IGB_FIXED_SAMPLING_RATE;
    }

    constexpr static float getSamplingRateF() {
      return (float)IGB_FIXED_SAMPLING_RATE;
    }
  };
#endif
}

