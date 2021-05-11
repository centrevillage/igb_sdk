#ifndef IGB_STM32_PERIPH_I2C_SLAVE_H
#define IGB_STM32_PERIPH_I2C_SLAVE_H

#include <igb_stm32/periph/i2c.hpp>
#include <igb_stm32/hal.hpp> // TODO: remove hal dependency

namespace igb {
namespace stm32 {

enum class I2cSlaveState : uint8_t {
  transmit = 0,
  receive,
  listen,
};

// Currently, it's support only I2C1 periph, so I2cType must be I2cType::i2c1
template<I2cType type, uint8_t own_address, GpioPinType scl_pin, GpioPinType sda_pin>
struct I2cSlave {
  constexpr static size_t buf_size = 1;

  I2c<type, scl_pin, sda_pin> i2c;
  I2C_HandleTypeDef _handle;
  size_t _data_idx = 0;
  uint8_t _tx_buf[buf_size];
  uint8_t _rx_buf[buf_size];

  I2cSlaveState state = I2cSlaveState::transmit;

  void init() {
    initPeriphClock();
    initBusClock();
    initGpio();
    initConfigI2c();
  }

  void initBusClock() {
    const auto& i2c_info = STM32_PERIPH_INFO.i2c[static_cast<size_t>(type)];
    i2c_info.bus.enableBusClock();
    i2c_info.bus.forceResetBusClock();
    i2c_info.bus.releaseResetBusClock();
  }

  void initPeriphClock() {
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
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
    HAL_NVIC_SetPriority(I2C1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(I2C1_IRQn);

    if (HAL_I2C_Init(&_handle) != HAL_OK) {
      return false;
    }
    if (HAL_I2CEx_ConfigAnalogFilter(&_handle, I2C_ANALOGFILTER_DISABLE) != HAL_OK) {
      return false;
    }
    if (HAL_I2CEx_ConfigDigitalFilter(&_handle, 0) != HAL_OK) {
      return false;
    }
    HAL_I2C_EnableListen_IT(&_handle);
    state = I2cSlaveState::listen;
    
    return true;
  }
};

} /* stm32 */
} /* igb */

// Must call just one time.
#define IGB_DEFINE_I2C1_SLAVE_IRQ_HANDLER(instance, listen_cplt_cb, tx_complete_cb, rx_complete_cb) \
  extern "C" {\
    void I2C1_IRQHandler(void);\
    void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode);\
    void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c);\
    void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c);\
    void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c);\
    void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);\
  }\
  void I2C1_IRQHandler(void) {\
    HAL_I2C_EV_IRQHandler(&(instance._handle));\
    HAL_I2C_ER_IRQHandler(&(instance._handle));\
  }\
  void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode) {\
    if ((instance.state == igb::stm32::I2cSlaveState::receive) && (instance._data_idx != 0)) {\
      instance.state = igb::stm32::I2cSlaveState::listen;\
      instance._data_idx = 0;\
    }\
    if (AddrMatchCode == hi2c->Init.OwnAddress1) {\
      if (TransferDirection == I2C_DIRECTION_RECEIVE) {\
        instance.state = igb::stm32::I2cSlaveState::transmit;\
        HAL_I2C_Slave_Seq_Transmit_IT(hi2c, instance._tx_buf, instance.buf_size, I2C_LAST_FRAME);\
      } else {\
        instance._data_idx = 0;\
        instance.state = igb::stm32::I2cSlaveState::receive;\
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, instance._rx_buf, instance.buf_size, I2C_NEXT_FRAME);\
      }\
    }\
  }\
  void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {\
    instance.state = I2cSlaveState::listen;\
    instance._data_idx = 0;\
    listen_cplt_cb();\
    HAL_I2C_EnableListen_IT(hi2c); \
  }\
  void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {\
    if (instance._data_idx < instance.buf_size) {\
      instance._data_idx++;\
    }\
    rx_complete_cb();\
    if (instance.state == igb::stm32::I2cSlaveState::receive) {\
      HAL_I2C_Slave_Seq_Receive_IT(hi2c, instance._rx_buf, instance.buf_size, I2C_NEXT_FRAME);\
    }\
  }\
  void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {\
    instance._tx_buf[0] = 0;\
  }\
  void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {\
    HAL_I2C_EnableListen_IT(hi2c);\
  }

#endif /* IGB_STM32_PERIPH_I2C_SLAVE_H */
