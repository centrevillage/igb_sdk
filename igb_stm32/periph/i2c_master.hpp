#ifndef IGB_STM32_PERIPH_I2C_MASTER_H
#define IGB_STM32_PERIPH_I2C_MASTER_H

#include <igb_stm32/periph/i2c.hpp>
#include <igb_stm32/periph/systick.hpp>
#include <igb_stm32/hal.hpp> // TODO: remove hal dependency

#if defined(I2C1_IRQn) && !defined(I2C1_EV_IRQn)
#define I2C1_EV_IRQn I2C1_IRQn
#endif

namespace igb {
namespace stm32 {

// Currently, it's support only I2C1 periph, so I2cType must be I2cType::i2c1
template<I2cType type, uint8_t own_address, GpioPinType scl_pin, GpioPinType sda_pin>
struct I2cMaster {
  constexpr static size_t buf_size = 16;
  constexpr static uint32_t timeout_tick = 1000;

  I2c<type, scl_pin, sda_pin> i2c;
  I2C_HandleTypeDef _handle;
  size_t _tx_data_idx = 0;
  size_t _tx_data_size = 0;
  size_t _rx_data_idx = 0;
  size_t _rx_data_size = 0;
  uint8_t _tx_buf[buf_size];
  uint8_t _rx_buf[buf_size];
  bool _is_transmitting = false;
  uint8_t _slave_tx_addr = 0; // 7bit-address << 1

  void init() {
    initPeriphClock();
    initBusClock();
    initGpio();
    initConfigI2c();
  }

  //bool writeData(uint8_t slave_addr, uint8_t* data_array, size_t data_array_size, bool is_send_stop = true) {
  //  if (is_send_stop) {
  //    _handle.XferOptions = I2C_OTHER_AND_LAST_FRAME;
  //  } else {
  //    _handle.XferOptions = I2C_OTHER_FRAME;
  //  }
  //  uint32_t tickstart = HAL_GetTick();
  //  uint32_t delta = 0;
  //  uint32_t err = 0;
  //  uint32_t XferOptions = _handle.XferOptions; // save XferOptions value, because handle can be modified by HAL, which cause issue in case of NACK from slave
  //  if (HAL_I2C_Master_Seq_Transmit_IT(&_handle, slave_addr, data_array, data_array_size, XferOptions) != HAL_OK) {
  //    return false;
  //  }
  //  // wait for transfer completion
  //  while ((HAL_I2C_GetState(&_handle) != HAL_I2C_STATE_READY) && (delta < timeout_tick)) {
  //    delta = (HAL_GetTick() - tickstart);
  //    if (HAL_I2C_GetError(&_handle) != HAL_I2C_ERROR_NONE) {
  //      break;
  //    }
  //  }

  //  err = HAL_I2C_GetError(&_handle);

  //  if ((delta >= timeout_tick) || ((err & HAL_I2C_ERROR_TIMEOUT) == HAL_I2C_ERROR_TIMEOUT)) {
  //    // timeout
  //    return false;
  //  }
  //  if ((err & HAL_I2C_ERROR_AF) == HAL_I2C_ERROR_AF) {
  //    //ret = I2C_NACK_DATA;
  //    return false;
  //  }
  //  if (err != HAL_I2C_ERROR_NONE) {
  //    //ret = I2C_ERROR;
  //    return false;
  //  }
  //  return true;
  //}

  bool writeData(uint8_t slave_addr, uint8_t* data_array, size_t data_array_size) {
    uint32_t tickstart = HAL_GetTick();
    uint32_t delta = 0;
    uint32_t err = 0;
    if (HAL_I2C_Master_Transmit_IT(&_handle, slave_addr << 1, data_array, data_array_size) != HAL_OK) {
      return false;
    }

    // wait for transfer completion
    while ((HAL_I2C_GetState(&_handle) != HAL_I2C_STATE_READY) && (delta < timeout_tick)) {
      delta = (HAL_GetTick() - tickstart);
      if (HAL_I2C_GetError(&_handle) != HAL_I2C_ERROR_NONE) {
        break;
      }
    }

    err = HAL_I2C_GetError(&_handle);

    if ((delta >= timeout_tick) || ((err & HAL_I2C_ERROR_TIMEOUT) == HAL_I2C_ERROR_TIMEOUT)) {
      // timeout
      return false;
    }
    if ((err & HAL_I2C_ERROR_AF) == HAL_I2C_ERROR_AF) {
      //ret = I2C_NACK_DATA;
      return false;
    }
    if (err != HAL_I2C_ERROR_NONE) {
      //ret = I2C_ERROR;
      return false;
    }
    return true;
  }

  void requestDataSync(uint8_t slave_addr, uint32_t reg_addr, uint8_t data_size, bool is_send_stop = true) {
    uint8_t readSize = 0;

    if (data_size > buf_size) {
      return;
    }

    for (size_t i = 0; i < data_size; ++i) {
      _rx_buf[i] = 0;
    }
    _rx_data_size = data_size;

    _slave_tx_addr = slave_addr << 1;
    _is_transmitting = true;
    _tx_data_idx = 0;
    _tx_data_size = 0;
    addData(reg_addr);
    transmit(false);
    read(is_send_stop);
  }

  void initPeriphClock() {
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
  }

  void initBusClock() {
    const auto& i2c_info = STM32_PERIPH_INFO.i2c[static_cast<size_t>(type)];
    i2c_info.bus.enableBusClock();
    i2c_info.bus.forceResetBusClock();
    i2c_info.bus.releaseResetBusClock();
  }

  void initGpio() {
    i2c.prepareGpio(scl_pin);
    i2c.prepareGpio(sda_pin);
  }

  bool initConfigI2c() {
    _handle.Instance = I2C1;
    _handle.Init.Timing = 0x00702025; // 100kHz
    _handle.Init.OwnAddress1 = own_address << 1;
    _handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    _handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    _handle.Init.OwnAddress2 = 0;
    _handle.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    _handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    _handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    _handle.State = HAL_I2C_STATE_RESET;
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);

    if (HAL_I2C_Init(&_handle) != HAL_OK) {
      return false;
    }
    if (HAL_I2CEx_ConfigAnalogFilter(&_handle, I2C_ANALOGFILTER_DISABLE) != HAL_OK) {
      return false;
    }
    if (HAL_I2CEx_ConfigDigitalFilter(&_handle, 0) != HAL_OK) {
      return false;
    }
    
    return true;
  }

  void addData(uint8_t data) {
    _tx_buf[_tx_data_idx] = data;
    ++_tx_data_idx;
    _tx_data_size = _tx_data_idx;
  }

  void clearData() {
    _tx_data_idx = 0;
    _tx_data_size = 0;
  }

  bool transmit(bool is_send_stop = true) {
    if (is_send_stop) {
      _handle.XferOptions = I2C_OTHER_AND_LAST_FRAME;
    } else {
      _handle.XferOptions = I2C_OTHER_FRAME;
    }

    uint32_t tickstart = HAL_GetTick();
    uint32_t delta = 0;
    uint32_t err = 0;
    if (_tx_data_size == 0) {
      //    ret = i2c_IsDeviceReady(obj, dev_address, 1);
      return (HAL_I2C_IsDeviceReady(&_handle, _slave_tx_addr, 1, timeout_tick) == HAL_OK);
    } else {
      uint32_t XferOptions = _handle.XferOptions; // save XferOptions value, because handle can be modified by HAL, which cause issue in case of NACK from slave
      if (HAL_I2C_Master_Seq_Transmit_IT(&_handle, _slave_tx_addr, _tx_buf, _tx_data_size, XferOptions) == HAL_OK) {
        // wait for transfer completion
        while ((HAL_I2C_GetState(&_handle) != HAL_I2C_STATE_READY) && (delta < timeout_tick)) {
          delta = (HAL_GetTick() - tickstart);
          if (HAL_I2C_GetError(&_handle) != HAL_I2C_ERROR_NONE) {
            break;
          }
        }

        err = HAL_I2C_GetError(&_handle);

        if ((delta >= timeout_tick) || ((err & HAL_I2C_ERROR_TIMEOUT) == HAL_I2C_ERROR_TIMEOUT)) {
          // timeout
          return false;
        }
        if ((err & HAL_I2C_ERROR_AF) == HAL_I2C_ERROR_AF) {
          //ret = I2C_NACK_DATA;
          return false;
        }
        if (err != HAL_I2C_ERROR_NONE) {
          //ret = I2C_ERROR;
          return false;
        }
      } else {
        return false;
      }
    }
    clearData();
    _is_transmitting = false;
  }

  bool read(bool is_send_stop = true) {
    if (is_send_stop == 0) {
      _handle.XferOptions = I2C_OTHER_FRAME ;
    } else {
      _handle.XferOptions = I2C_OTHER_AND_LAST_FRAME;
    }

    uint32_t tickstart = HAL_GetTick();
    uint32_t delta = 0;
    uint32_t err = 0;
    uint32_t XferOptions = _handle.XferOptions; // save XferOptions value, because handle can be modified by HAL, which cause issue in case of NACK from slave
    if (HAL_I2C_Master_Seq_Receive_IT(&_handle, _slave_tx_addr, _rx_buf, _rx_data_size, XferOptions) == HAL_OK) {
      // wait for transfer completion
      while ((HAL_I2C_GetState(&_handle) != HAL_I2C_STATE_READY) && (delta < timeout_tick)) {
        delta = (HAL_GetTick() - tickstart);
        if (HAL_I2C_GetError(&_handle) != HAL_I2C_ERROR_NONE) {
          break;
        }
      }
      err = HAL_I2C_GetError(&_handle);
      if ((delta >= timeout_tick) || ((err & HAL_I2C_ERROR_TIMEOUT) == HAL_I2C_ERROR_TIMEOUT)) {
        //ret = I2C_TIMEOUT;
        return false;
      } else {
        if ((err & HAL_I2C_ERROR_AF) == HAL_I2C_ERROR_AF) {
          //ret = I2C_NACK_DATA;
          return false;
        } else if (err != HAL_I2C_ERROR_NONE) {
          //ret = I2C_ERROR;
          return false;
        }
      }
    }
    _rx_data_idx = 0;

    return true;
  }
};

}
}

#endif /* IGB_STM32_PERIPH_I2C_MASTER_H */
