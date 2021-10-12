#ifndef IGB_SDK_DEVICE_EXT_MEM_ACCESSOR_H
#define IGB_SDK_DEVICE_EXT_MEM_ACCESSOR_H

#include <functional>
#include <igb_util/ring_buf.hpp>

namespace igb {
namespace sdk {

template<typename MemCls /* FramMb85rSPI, etc... */>
struct ExtMemAccessor {
  MemCls memory;

  enum class ReqType : uint8_t {
    read = 0,
    write
  };
  struct ReqInfo {
    ReqType req_type;
    uint8_t* buf;
    uint32_t buf_size;
    uint32_t mem_address;
    std::function<void(void)> callback = nullptr;
  };

  RingBuf<ReqInfo, 64> requests;

  void init() {
    memory.init();
  }

  void request(auto&& info) {
    requests.add(info);
  }

  void process() {
    if (!memory.isProcessing()) {
      const auto next_request = requests.get();
      if (next_request) {
        const auto r = next_request.value();
        if (r.req_type == ReqType::read) {
          memory.requestRead(r.buf, r.buf_size, r.mem_address, r.callback);
        } else { // write
          memory.requestWrite(r.buf, r.buf_size, r.mem_address, r.callback);
        }
      }
    }
    memory.process();
  }
};

}
}

#endif /* IGB_SDK_DEVICE_EXT_MEM_ACCESSOR_H */
