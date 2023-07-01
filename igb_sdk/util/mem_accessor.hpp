#ifndef IGB_SDK_UTIL_MEM_ACCESSOR_H
#define IGB_SDK_UTIL_MEM_ACCESSOR_H

#include <functional>
#include <igb_util/ring_buf.hpp>

namespace igb {
namespace sdk {

template<typename MemCls /* FramMb85rSPI, etc... */, size_t data_buf_size, size_t request_buf_size = 32>
struct MemAccessor {
  MemCls memory;

  uint8_t buf[data_buf_size];

  enum class ReqType : uint8_t {
    read = 0,
    write
  };
  struct ReqInfo {
    ReqType req_type;
    uint32_t mem_address;
    size_t buf_size;
    std::function<size_t(uint8_t* buf, size_t size)> serialize_func = [](uint8_t* buf, size_t size){ return 0; };
    std::function<void(void)> complete_func = nullptr;
  };

  RingBuf<ReqInfo, request_buf_size> requests;
  std::optional<ReqInfo> next_request = std::nullopt;

  void init() {
    memory.init();
  }

  void request(auto&& info) {
    requests.add(info);
  }

  void process() {
    if (!memory.isProcessing()) {
      next_request = requests.get();
      if (next_request) {
        const auto& r = next_request.value();
        if (r.req_type == ReqType::read) {

          memory.requestRead(buf, r.buf_size, r.mem_address, [this, &r](){
            r.serialize_func(buf, data_buf_size);
            if (r.complete_func) {
              r.complete_func();
            }
          });
        } else { // write
          r.serialize_func(buf, data_buf_size);
          memory.requestWrite(buf, r.buf_size, r.mem_address, [this, &r](){
            if (r.complete_func) {
              r.complete_func();
            }
          });
        }
      }
    }
    memory.process();
  }

  inline bool isProcessing() const {
    return memory.isProcessing();
  }

  inline bool isRequestEmpty() const {
    return requests.size() == 0;
  }
};

}
}

#endif /* IGB_SDK_UTIL_MEM_ACCESSOR_H */
