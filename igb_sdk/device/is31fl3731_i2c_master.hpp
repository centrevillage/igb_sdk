#ifndef IGB_SDK_DEVICE_IS31FL3731_I2C_MASTER_HPP
#define IGB_SDK_DEVICE_IS31FL3731_I2C_MASTER_HPP

#include <igb_sdk/base.hpp>

namespace igb {
namespace sdk {

template<typename I2C_TYPE>
struct Is31Fl3731I2cMaster {
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


  I2C_TYPE i2c_master;

  uint8_t _address_byte = i2c_address_base;
  uint8_t _frame = 0;

  enum class Address : uint8_t {
    first = i2c_address_base,
    second,
    third,
    fourth,
  };

  bool init(Address address = Address::first) {
    i2c_master.init();
    delay_msec(10);

    _address_byte = static_cast<uint8_t>(address);

    //if (!i2c.checkSlave(_address_byte)) {
    //  return false;
    //}

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

    return true;
  }

  void writeReg(uint8_t page, uint8_t cmd, uint8_t data) {
    movePage(page);
    uint8_t _data[2];
    _data[0] = cmd; //command
    _data[1] = data; //reg
    i2c_master.writeData(_address_byte, _data, 2);
  }

  uint8_t readReg(uint8_t page, uint8_t reg) {
    movePage(page);

    i2c_master.requestDataSync(_address_byte, reg, 1);

    return i2c_master._rx_buf[0];
  }

  void movePage(uint8_t page) {
    uint8_t data[2];
    data[0] = command_register; //command
    data[1] = page; //reg
    i2c_master.writeData(_address_byte, data, 2);
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

    uint8_t data[25];
    for (uint8_t i = 0; i < 6; i++) {
      data[0] = 0x24 + i * 24; //command
      for (uint8_t j = 0; j < 24; j++) {
        data[i+1] = 0;
      }
      i2c_master.writeData(_address_byte, data, 25);
    }
  }

  void setLed(uint8_t lednum, uint8_t power) {
    writeReg(_frame, 0x24 + lednum, power);
  }
  // TODO:
};

}
}

#endif /* IGB_SDK_DEVICE_IS31FL3731_I2C_MASTER_HPP */
