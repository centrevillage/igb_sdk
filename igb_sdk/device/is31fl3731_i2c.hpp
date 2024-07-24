#pragma once

#include <igb_sdk/base.hpp>
#include <igb_util/ring_buf.hpp>

namespace igb {
namespace sdk {

template<typename I2C_TYPE>
struct Is31Fl3731I2c {
  I2C_TYPE i2c;

  constexpr static size_t i2c_address_base      = 0x74;
  constexpr static uint8_t page_function_reg    = 0x0B;
  constexpr static uint8_t reg_config           = 0x00;
  constexpr static uint8_t reg_config_picture   = 0x00;
  constexpr static uint8_t reg_config_autoplay  = 0x08;
  constexpr static uint8_t reg_config_audioplay = 0x18;
  constexpr static uint8_t conf_picture         = 0x00;
  constexpr static uint8_t conf_autoframe       = 0x04;
  constexpr static uint8_t conf_audio           = 0x08;
  constexpr static uint8_t reg_pictureframe     = 0x01;
  constexpr static uint8_t reg_shutdown         = 0x0A;
  constexpr static uint8_t reg_audiosync        = 0x06;
  constexpr static uint8_t command_register     = 0xFD;

  uint8_t _address_byte = i2c_address_base;
  uint8_t _frame = 0;

  struct LedCmd {
    uint8_t num = 0;
    uint8_t power = 0;
  };

  enum class LedCmdState : uint8_t {
    ready = 0,
    begin,
    processingNum,
    processingPower,
  };

  RingBuf256<LedCmd> _cmd_buf; 

  std::optional<LedCmd> _current_cmd;
  LedCmdState _current_cmd_state = LedCmdState::ready;

  enum class Address : uint8_t {
    first = i2c_address_base,
    second,
    third,
    fourth,
  };

  bool init(Address address = Address::first) {
    //i2c.init();
    delay_msec(10);

    _address_byte = static_cast<uint8_t>(address);

    if (!i2c.checkSlave(_address_byte)) {
      return false;
    }

    writeReg(page_function_reg, reg_shutdown, 0x00);

    delay_msec(10);

    writeReg(page_function_reg, reg_shutdown, 0x01);

    writeReg(page_function_reg, reg_config, reg_config_picture);
    showCurrentFrame();
    clearCurrentFrame();

    for (uint8_t f = 0; f < 8; f++) {
      for (uint8_t i = 0; i <= 0x11; i++) {
        writeReg(f, i, 0xFF);
      }
    }
    writeReg(page_function_reg, reg_audiosync, 0);

    movePage(_frame);

    return true;
  }

  void writeReg(uint8_t page, uint8_t cmd, uint8_t data) {
    movePage(page);
    i2c.beginSending(_address_byte, 2);
    i2c.sendU8sync(cmd);
    i2c.sendU8sync(data);
    i2c.endSending();
  }

  uint8_t readReg(uint8_t page, uint8_t reg) {
    movePage(page);
    i2c.beginSending(_address_byte, 1);
    i2c.sendU8sync(reg);
    i2c.endSending();

    i2c.beginReading(_address_byte, 1);
    uint8_t result = i2c.receiveU8sync();
    i2c.endReading();

    return result;
  }

  void movePage(uint8_t page) {
    i2c.beginSending(_address_byte, 2);
    i2c.sendU8sync(command_register);
    i2c.sendU8sync(page);
    i2c.endSending();
  }

  void selectFrame(uint8_t frame) {
    _frame = frame;
  }

  void showFrame(uint8_t frame) {
    writeReg(page_function_reg, reg_pictureframe, frame);
  }

  void showCurrentFrame() {
    showFrame(_frame);
  }

  void clearCurrentFrame() {
    movePage(_frame);


    for (uint8_t i = 0; i < 6; i++) {
      i2c.beginSending(_address_byte, 25);
      i2c.sendU8sync(0x24 + i * 24);
      for (uint8_t j = 0; j < 24; j++) {
        i2c.sendU8sync(0);
      }
      i2c.endSending();
    }
  }

  void setLed(uint8_t lednum, uint8_t power) {
    writeReg(_frame, 0x24 + lednum, power);
  }

  void addLedCmd(uint8_t lednum, uint8_t power) {
    _cmd_buf.add(LedCmd { lednum, power } );
  }

  void processLedCmd() {
    switch (_current_cmd_state) {
      case LedCmdState::ready:
        if (!_current_cmd) {
          _current_cmd = _cmd_buf.get();
        }
        if (_current_cmd) {
          i2c.beginSending(_address_byte, 2);
          _current_cmd_state = LedCmdState::begin;
        }
        break;
      case LedCmdState::begin:
        if (i2c.isSendable()) {
          i2c.sendU8(0x24 + _current_cmd.value().num);
          _current_cmd_state = LedCmdState::processingNum;
        }
        break;
      case LedCmdState::processingNum:
        if (i2c.isSendable()) {
          i2c.sendU8(_current_cmd.value().power);
          _current_cmd_state = LedCmdState::processingPower;
        }
        break;
      case LedCmdState::processingPower:
        if (i2c.isTransferEnd()) {
          _current_cmd = std::nullopt;
          _current_cmd_state = LedCmdState::ready;
        }
        break;
      default:
        break;
    }
  }

  bool isCmdEmpty() const {
    return _cmd_buf.size() == 0 && !_current_cmd;
  }
};

}
}

