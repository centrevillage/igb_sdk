#pragma once

#include <igb_sdk/base.hpp>
#include <functional>
#include <algorithm>
#include <igb_util/macro.hpp>
#include <igb_util/null_functor.hpp>

namespace igb {
namespace sdk {

template<typename I2C_TYPE>
struct FramMb85Rc04I2c {
  I2C_TYPE i2c;

  constexpr static size_t i2c_address_base = 0x50;
  const size_t block_size = 128;

  uint8_t _address_byte = i2c_address_base;

  enum class Address : uint8_t {
    first  = i2c_address_base,
    second = i2c_address_base | (1 << 1),
    third  = i2c_address_base | (2 << 1),
    fourth = i2c_address_base | (3 << 1)
  };

  enum class AccessState : uint8_t {
    none = 0,
    write,
    read
  };

  AccessState _ram_access_state = AccessState::none;
  uint8_t* _ram_target_buf = nullptr;
  uint32_t _ram_buf_index = 0;
  uint32_t _ram_buf_process_index = 0;
  uint32_t _ram_buf_size = 0;
  uint32_t _ram_address = 0;
  // 関数ポインタでなくstd::functionの方が扱いやすいが、パフォーマンス低下が気になるので・・・
  // 将来的にはfunction_refとかに置き換えたい
  //void (*_ram_callback)(void) = nullptr;
  std::function<void(void)> _ram_callback = nullptr;

  enum class ProcessState : uint8_t {
    ready = 0,
    begin,
    receiveHead,
    transfer,
    end
  };

  ProcessState _process_state = ProcessState::ready;

  inline bool init(Address address = Address::first) {
    _address_byte = static_cast<uint8_t>(address);
    if (!i2c.checkSlave(_address_byte)) {
      return false;
    }
    return true;
  }

  void requestRead(uint8_t* buf, uint32_t buf_size, uint32_t read_address, auto&& callback) {
    if (_ram_access_state == AccessState::none) {
      _ram_target_buf = buf;
      _ram_address = read_address;
      _ram_callback = callback;
      _ram_buf_size = buf_size;
      _ram_buf_index = 0;
      _ram_buf_process_index = 0;
      _ram_access_state = AccessState::read;
      _process_state = ProcessState::ready;
    }
  }

  //inline void requestWrite(uint8_t* buf, uint32_t buf_size, uint32_t write_address, void (*callback)(void)) {
  void requestWrite(uint8_t* buf, uint32_t buf_size, uint32_t write_address, auto&& callback) {
    if (_ram_access_state == AccessState::none) {
      _ram_target_buf = buf;
      _ram_address = write_address;
      _ram_callback = callback;
      _ram_buf_size = buf_size;
      _ram_buf_index = 0;
      _ram_buf_process_index = 0;
      _ram_access_state = AccessState::write;
      _process_state = ProcessState::ready;
    }
  }

  inline void process() {
    switch(_ram_access_state) {
      case AccessState::none:
        break;
      case AccessState::read:
        _processRead();
        break;
      case AccessState::write:
        _processWrite();
        break;
      default:
        break;
    }
  }

  inline bool isProcessing() const {
    return _ram_access_state != AccessState::none;
  }

  inline void _processRead() {
    switch (_process_state) {
      case ProcessState::ready:
        {
          i2c.beginSending(_address_byte | ((_ram_address & 0x100) ? 1 : 0), 1);
          _process_state = ProcessState::begin;
        }
        break;
      case ProcessState::begin:
        if (i2c.isSendable()) {
          i2c.sendU8(_ram_address & 0xFF);
          _ram_buf_process_index = 0;
          _process_state = ProcessState::receiveHead;
        }
        break;
      case ProcessState::receiveHead:
        if (i2c.isTransferEnd()) {
          uint8_t process_size = std::min<uint32_t>(block_size, _ram_buf_size - _ram_buf_index);
          i2c.beginReading(_address_byte | ((_ram_address & 0x100) ? 1 : 0), process_size);
          _process_state = ProcessState::transfer;
        }
        break;
      case ProcessState::transfer:
        if (i2c.isReceivable()) {
          _ram_target_buf[_ram_buf_index++] = i2c.receiveU8();
          _ram_address++;
          _ram_buf_process_index++;
          if (_ram_buf_process_index >= block_size || _ram_buf_index >= _ram_buf_size) {
            _process_state = ProcessState::end;
          }
        }
        break;
      case ProcessState::end:
        {
          if (i2c.isTransferEnd()) {
            _process_state = ProcessState::ready; 
            if (_ram_buf_index >= _ram_buf_size) {
              // complete
              _ram_access_state = AccessState::none;
              if (_ram_callback) {
                _ram_callback();
              }
            }
          }
        }
        break;
    }
  }

  inline void _processWrite() {
    switch (_process_state) {
      case ProcessState::ready:
        {
          uint8_t process_size = std::min<uint32_t>(block_size, _ram_buf_size - _ram_buf_index);
          i2c.beginSending(_address_byte | ((_ram_address & 0x100) ? 1 : 0), process_size + 1);
          _process_state = ProcessState::begin;
        }
        break;
      case ProcessState::begin:
        if (i2c.isSendable()) {
          i2c.sendU8(_ram_address & 0xFF);
          _ram_buf_process_index = 0;
          _process_state = ProcessState::transfer;
        }
        break;
      case ProcessState::receiveHead:
        break;
      case ProcessState::transfer:
        if (i2c.isSendable()) {
          i2c.sendU8(_ram_target_buf[_ram_buf_index++]);
          _ram_address++;
          _ram_buf_process_index++;
          if (_ram_buf_process_index >= block_size || _ram_buf_index >= _ram_buf_size) {
            _process_state = ProcessState::end;
          }
        }
        break;
      case ProcessState::end:
        {
          if (i2c.isTransferEnd()) {
            _process_state = ProcessState::ready; 
            if (_ram_buf_index >= _ram_buf_size) {
              // complete
              _ram_access_state = AccessState::none;
              if (_ram_callback) {
                _ram_callback();
              }
            }
          }
        }
        break;
    }
  }
};

}
}

