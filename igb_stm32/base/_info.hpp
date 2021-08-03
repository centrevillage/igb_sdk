struct PeriphBusInfo {
  const BusType   bus_type;
  const uint32_t  periph_bit;

  inline void enableBusClock() const {
    auto p_enr = STM32_BUS_TO_ENR_ADDRESS[ static_cast<uint32_t>(bus_type) ];
    (*p_enr) |= periph_bit;
    __IO auto tmp = (*p_enr); // delay until clock enabled
  }

  inline void forceResetBusClock() const {
    auto p_rstr = STM32_BUS_TO_RSTR_ADDRESS[ static_cast<uint32_t>(bus_type) ];
    (*p_rstr) |= periph_bit;
    __IO auto tmp = (*p_rstr); // delay until clock enabled
  }

  inline void releaseResetBusClock() const {
    auto p_rstr = STM32_BUS_TO_RSTR_ADDRESS[ static_cast<uint32_t>(bus_type) ];
    (*p_rstr) &= ~periph_bit;
    __IO auto tmp = (*p_rstr); // delay until clock enabled
  }
};

struct GpioAfInfo {
  GPIO_TypeDef* const p_gpio;
  const uint16_t      pin_bit;
  const uint8_t       af_idx;
};

#ifdef STM32_PERIPHGRP_RCC_EXISTS
struct RccInfo {
  const PeriphType   periph_type;
  RCC_TypeDef* const p_rcc;
  const uint32_t addr;
};
#endif

#ifdef STM32_PERIPHGRP_GPIO_EXISTS
struct GpioInfo {
  const PeriphType    periph_type;
  GPIO_TypeDef* const p_gpio;
  const uint32_t addr;
  const PeriphBusInfo bus;
};
#endif

#ifdef STM32_PERIPHGRP_TIM_EXISTS
enum class TimCategory {
  UNKNOWN = 0,
  ADVANCED,
  GENERAL,
  BASIC,
};
struct TimInfo {
  const PeriphType      periph_type;
  const TimCategory     category;
  TIM_TypeDef* const    p_tim;
  const uint32_t addr;
  const IRQn_Type       irqn;
  const PeriphBusInfo   bus;
};
#endif

#ifdef STM32_PERIPHGRP_SPI_EXISTS
struct SpiInfo {
  const PeriphType   periph_type;
  SPI_TypeDef* const p_spi;
  const uint32_t addr;
  const PeriphBusInfo bus;
};
#endif

#ifdef STM32_PERIPHGRP_I2C_EXISTS
struct I2cInfo {
  const PeriphType   periph_type;
  I2C_TypeDef* const p_i2c;
  const uint32_t addr;
  const PeriphBusInfo bus;
};
#endif

#ifdef STM32_PERIPHGRP_USART_EXISTS
struct UsartInfo {
  const PeriphType   periph_type;
  USART_TypeDef* const p_usart;
  const uint32_t addr;
  const PeriphBusInfo bus;
};
#endif

#ifdef STM32_PERIPHGRP_DAC_EXISTS
struct DacInfo {
  const PeriphType   periph_type;
  DAC_TypeDef* const p_dac;
  const uint32_t addr;
  const PeriphBusInfo bus;
};
#endif

#ifdef STM32_PERIPHGRP_ADC_EXISTS
struct AdcInfo {
  const PeriphType   periph_type;
  ADC_TypeDef* const p_adc;
  const uint32_t addr;
  const IRQn_Type irqn;
  const PeriphBusInfo bus;
};
#endif

#ifdef STM32_PERIPHGRP_TSC_EXISTS
struct TscInfo {
  const PeriphType   periph_type;
  TSC_TypeDef* const p_tsc;
  const uint32_t addr;
  const PeriphBusInfo bus;
};
#endif

#ifdef STM32_PERIPHGRP_DMA_EXISTS
struct DmaChannelInfo {
  bool exists = true;
  DMA_Channel_TypeDef* const p_dma_channel;
  const uint32_t addr;
  const IRQn_Type irqn;
};
struct DmaInfo {
  const PeriphType   periph_type;
  DMA_TypeDef* const p_dma;
  const uint32_t addr;
  const PeriphBusInfo bus;
  const std::array<DmaChannelInfo, 8> channels;
};
#endif
