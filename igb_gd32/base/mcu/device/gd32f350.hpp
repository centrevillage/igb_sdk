#ifndef IGB_GD32_BASE_MCU_DEVICE_GD32F350_H
#define IGB_GD32_BASE_MCU_DEVICE_GD32F350_H

#include <igb_gd32/base.hpp>
#include <array>
#include <optional>

#define PERIPH_BASE           0x40000000UL

#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000UL)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000UL)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x08000000UL)
#define AHB3PERIPH_BASE       (PERIPH_BASE + 0x10000000UL)

// AHB1 peripherails
//#define TIM2_BASE             (APB1PERIPH_BASE + 0x00000000UL)
//#define TIM3_BASE             (APB1PERIPH_BASE + 0x00000400UL)
//#define TIM4_BASE             (APB1PERIPH_BASE + 0x00000800UL)
//#define TIM6_BASE             (APB1PERIPH_BASE + 0x00001000UL)
//#define TIM7_BASE             (APB1PERIPH_BASE + 0x00001400UL)
//#define RTC_BASE              (APB1PERIPH_BASE + 0x00002800UL)
//#define WWDG_BASE             (APB1PERIPH_BASE + 0x00002C00UL)
//#define IWDG_BASE             (APB1PERIPH_BASE + 0x00003000UL)
//#define I2S2ext_BASE          (APB1PERIPH_BASE + 0x00003400UL)
//#define SPI2_BASE             (APB1PERIPH_BASE + 0x00003800UL)
//#define SPI3_BASE             (APB1PERIPH_BASE + 0x00003C00UL)
//#define I2S3ext_BASE          (APB1PERIPH_BASE + 0x00004000UL)
//#define USART2_BASE           (APB1PERIPH_BASE + 0x00004400UL)
//#define USART3_BASE           (APB1PERIPH_BASE + 0x00004800UL)
//#define UART4_BASE            (APB1PERIPH_BASE + 0x00004C00UL)
//#define UART5_BASE            (APB1PERIPH_BASE + 0x00005000UL)
//#define I2C1_BASE             (APB1PERIPH_BASE + 0x00005400UL)
//#define I2C2_BASE             (APB1PERIPH_BASE + 0x00005800UL)
//#define USB_BASE              (APB1PERIPH_BASE + 0x00005C00UL) /*!< USB_IP Peripheral Registers base address */
//#define USB_PMAADDR           (APB1PERIPH_BASE + 0x00006000UL) /*!< USB_IP Packet Memory Area base address */
//#define CAN_BASE              (APB1PERIPH_BASE + 0x00006400UL)
//#define PWR_BASE              (APB1PERIPH_BASE + 0x00007000UL)
//#define DAC1_BASE             (APB1PERIPH_BASE + 0x00007400UL)
//#define DAC_BASE               DAC1_BASE

// AHB2 peripherails
//#define SYSCFG_BASE           (APB2PERIPH_BASE + 0x00000000UL)
//#define COMP1_BASE            (APB2PERIPH_BASE + 0x0000001CUL)
//#define COMP2_BASE            (APB2PERIPH_BASE + 0x00000020UL)
//#define COMP3_BASE            (APB2PERIPH_BASE + 0x00000024UL)
//#define COMP4_BASE            (APB2PERIPH_BASE + 0x00000028UL)
//#define COMP5_BASE            (APB2PERIPH_BASE + 0x0000002CUL)
//#define COMP6_BASE            (APB2PERIPH_BASE + 0x00000030UL)
//#define COMP7_BASE            (APB2PERIPH_BASE + 0x00000034UL)
//#define COMP_BASE             COMP1_BASE
//#define OPAMP1_BASE           (APB2PERIPH_BASE + 0x00000038UL)
//#define OPAMP2_BASE           (APB2PERIPH_BASE + 0x0000003CUL)
//#define OPAMP3_BASE           (APB2PERIPH_BASE + 0x00000040UL)
//#define OPAMP4_BASE           (APB2PERIPH_BASE + 0x00000044UL)
//#define OPAMP_BASE            OPAMP1_BASE
//#define EXTI_BASE             (APB2PERIPH_BASE + 0x00000400UL)
//#define TIM1_BASE             (APB2PERIPH_BASE + 0x00002C00UL)
//#define SPI1_BASE             (APB2PERIPH_BASE + 0x00003000UL)
//#define TIM8_BASE             (APB2PERIPH_BASE + 0x00003400UL)
//#define USART1_BASE           (APB2PERIPH_BASE + 0x00003800UL)
//#define TIM15_BASE            (APB2PERIPH_BASE + 0x00004000UL)
//#define TIM16_BASE            (APB2PERIPH_BASE + 0x00004400UL)
//#define TIM17_BASE            (APB2PERIPH_BASE + 0x00004800UL)

// AHB1 peripherals
//#define DMA1_BASE             (AHB1PERIPH_BASE + 0x00000000UL)
//#define DMA1_Channel1_BASE    (AHB1PERIPH_BASE + 0x00000008UL)
//#define DMA1_Channel2_BASE    (AHB1PERIPH_BASE + 0x0000001CUL)
//#define DMA1_Channel3_BASE    (AHB1PERIPH_BASE + 0x00000030UL)
//#define DMA1_Channel4_BASE    (AHB1PERIPH_BASE + 0x00000044UL)
//#define DMA1_Channel5_BASE    (AHB1PERIPH_BASE + 0x00000058UL)
//#define DMA1_Channel6_BASE    (AHB1PERIPH_BASE + 0x0000006CUL)
//#define DMA1_Channel7_BASE    (AHB1PERIPH_BASE + 0x00000080UL)
//#define DMA2_BASE             (AHB1PERIPH_BASE + 0x00000400UL)
//#define DMA2_Channel1_BASE    (AHB1PERIPH_BASE + 0x00000408UL)
//#define DMA2_Channel2_BASE    (AHB1PERIPH_BASE + 0x0000041CUL)
//#define DMA2_Channel3_BASE    (AHB1PERIPH_BASE + 0x00000430UL)
//#define DMA2_Channel4_BASE    (AHB1PERIPH_BASE + 0x00000444UL)
//#define DMA2_Channel5_BASE    (AHB1PERIPH_BASE + 0x00000458UL)
#define RCC_BASE              (AHB1PERIPH_BASE + 0x00001000UL)
//#define FLASH_R_BASE          (AHB1PERIPH_BASE + 0x00002000UL)
//#define OB_BASE               0x1FFFF800UL
//#define FLASHSIZE_BASE        0x1FFFF7CCUL
//#define UID_BASE              0x1FFFF7ACUL
//#define CRC_BASE              (AHB1PERIPH_BASE + 0x00003000UL)
//#define TSC_BASE              (AHB1PERIPH_BASE + 0x00004000UL)

// AHB2 peripherals
#define GPIOA_BASE            (AHB2PERIPH_BASE + 0x00000000UL)
#define GPIOB_BASE            (AHB2PERIPH_BASE + 0x00000400UL)
#define GPIOC_BASE            (AHB2PERIPH_BASE + 0x00000800UL)
#define GPIOD_BASE            (AHB2PERIPH_BASE + 0x00000C00UL)
#define GPIOF_BASE            (AHB2PERIPH_BASE + 0x00001400UL)

#define GD32_PERIPHGRP_GPIO_EXISTS 1
#define GD32_PERIPH_GPIOA_EXISTS 1
#define GD32_PERIPH_GPIOB_EXISTS 1
#define GD32_PERIPH_GPIOC_EXISTS 1
#define GD32_PERIPH_GPIOD_EXISTS 1
#define GD32_PERIPH_GPIOF_EXISTS 1
#define GD32_PERIPH_GPIO_REG_BRR_EXISTS 1
#define GD32_PERIPHGRP_RCC_EXISTS 1
#define GD32_PERIPH_RCC_EXISTS 1
#define GD32_PERIPHGRP_ADC_EXISTS 1
#define GD32_PERIPH_ADC1_EXISTS 1

// TODO:
//#define GD32_PERIPHGRP_TSC_EXISTS 1
//#define GD32_PERIPH_TSC_EXISTS 1
//#define GD32_PERIPHGRP_CRC_EXISTS 1
//#define GD32_PERIPH_CRC_EXISTS 1
//#define GD32_PERIPHGRP_FLASH_EXISTS 1
//#define GD32_PERIPH_FLASH_EXISTS 1
//#define GD32_PERIPHGRP_DMA_EXISTS 1
//#define GD32_PERIPH_DMA1_EXISTS 1
//#define GD32_PERIPH_DMA2_EXISTS 1
//#define GD32_PERIPHGRP_TIM_EXISTS 1
//#define GD32_PERIPH_TIM2_EXISTS 1
//#define GD32_PERIPH_TIM3_EXISTS 1
//#define GD32_PERIPH_TIM4_EXISTS 1
//#define GD32_PERIPH_TIM15_EXISTS 1
//#define GD32_PERIPH_TIM16_EXISTS 1
//#define GD32_PERIPH_TIM17_EXISTS 1
//#define GD32_PERIPH_TIM6_EXISTS 1
//#define GD32_PERIPH_TIM7_EXISTS 1
//#define GD32_PERIPH_TIM1_EXISTS 1
//#define GD32_PERIPH_TIM8_EXISTS 1
//#define GD32_PERIPHGRP_USART_EXISTS 1
//#define GD32_PERIPH_USART1_EXISTS 1
//#define GD32_PERIPH_USART2_EXISTS 1
//#define GD32_PERIPH_USART3_EXISTS 1
//#define GD32_PERIPH_UART4_EXISTS 1
//#define GD32_PERIPH_UART5_EXISTS 1
//#define GD32_PERIPHGRP_SPI_EXISTS 1
//#define GD32_PERIPH_SPI1_EXISTS 1
//#define GD32_PERIPHGRP_EXTI_EXISTS 1
//#define GD32_PERIPH_EXTI_EXISTS 1
//#define GD32_PERIPHGRP_PWR_EXISTS 1
//#define GD32_PERIPH_PWR_EXISTS 1
//#define GD32_PERIPHGRP_CAN_EXISTS 1
//#define GD32_PERIPH_CAN_EXISTS 1
//#define GD32_PERIPHGRP_USB_FS_EXISTS 1
//#define GD32_PERIPH_USB_FS_EXISTS 1
//#define GD32_PERIPHGRP_I2C_EXISTS 1
//#define GD32_PERIPH_I2C1_EXISTS 1
//#define GD32_PERIPHGRP_IWDG_EXISTS 1
//#define GD32_PERIPH_IWDG_EXISTS 1
//#define GD32_PERIPHGRP_WWDG_EXISTS 1
//#define GD32_PERIPH_WWDG_EXISTS 1
//#define GD32_PERIPHGRP_RTC_EXISTS 1
//#define GD32_PERIPH_RTC_EXISTS 1
//#define GD32_PERIPHGRP_DAC_EXISTS 1
//#define GD32_PERIPH_DAC1_EXISTS 1
//#define GD32_PERIPHGRP_DBGMCU_EXISTS 1
//#define GD32_PERIPH_DBGMCU_EXISTS 1
//#define GD32_PERIPHGRP_SYSCFG_EXISTS 1
//#define GD32_PERIPH_SYSCFG_EXISTS 1
//#define GD32_PERIPHGRP_FMC_EXISTS 1
//#define GD32_PERIPH_FMC_EXISTS 1
//#define GD32_PERIPHGRP_NVIC_EXISTS 1
//#define GD32_PERIPH_NVIC_EXISTS 1
//#define GD32_PERIPH_NVIC_STIR_EXISTS 1
//#define GD32_PERIPHGRP_FPU_EXISTS 1
//#define GD32_PERIPH_FPU_EXISTS 1
//#define GD32_PERIPH_FPU_CPACR_EXISTS 1
//#define GD32_PERIPHGRP_MPU_EXISTS 1
//#define GD32_PERIPH_MPU_EXISTS 1
//#define GD32_PERIPHGRP_STK_EXISTS 1
//#define GD32_PERIPH_STK_EXISTS 1
//#define GD32_PERIPHGRP_SCB_EXISTS 1
//#define GD32_PERIPH_SCB_EXISTS 1
//#define GD32_PERIPH_SCB_ACTRL_EXISTS 1

namespace igb {
namespace gd32 {

enum class PeriphGroupType : uint16_t {
  gpio = 0,
  rcc,
  //tsc,
  //dma,
  //tim,
  //usart,
  //spi,
  //exti,
  //i2c,
  //dac,
  //adc,
  //syscfg,
};

enum class PeriphType : uint16_t {
  gpioa = 0,
  gpiob,
  gpioc,
  gpiod,
  gpiof,
  rcc,
  adc1,
  i2c0,
  i2c1,
  i2s0,
  spi0,
  spi1,
  tim0,
  tim1,
  tim2,
  tim5,
  tim13,
  tim14,
  tim15,
  tim16,
  tsc,
  usart0,
  usart1,
  usb,
};

enum class GpioType : uint8_t {
  gpioa = 0,
  gpiob = 1,
  gpioc = 2,
  gpiod = 3,
  gpiof = 5,
};
constexpr static uint8_t to_idx(GpioType type) {
  switch (type) {
    case GpioType::gpioa:
      return 0;
      break;
    case GpioType::gpiob:
      return 1;
      break;
    case GpioType::gpioc:
      return 2;
      break;
    case GpioType::gpiod:
      return 3;
      break;
    case GpioType::gpiof:
      return 4;
      break;
  }
  return 0;
}

// TODO:
//enum class DmaType : uint8_t {
//  dma1 = 0,
//  dma2,
//};
//constexpr static uint8_t to_idx(DmaType type) {
//  switch (type) {
//    case DmaType::dma1:
//      return 0;
//      break;
//    case DmaType::dma2:
//      return 1;
//      break;
//  }
//  return 0;
//}
//enum class TimType : uint8_t {
//  tim1 = 0,
//  tim2,
//  tim3,
//  tim4,
//  tim6,
//  tim7,
//  tim8,
//  tim15,
//  tim16,
//  tim17,
//};
//constexpr static uint8_t to_idx(TimType type) {
//  switch (type) {
//    case TimType::tim1:
//      return 0;
//      break;
//    case TimType::tim2:
//      return 1;
//      break;
//    case TimType::tim3:
//      return 2;
//      break;
//    case TimType::tim4:
//      return 3;
//      break;
//    case TimType::tim6:
//      return 4;
//      break;
//    case TimType::tim7:
//      return 5;
//      break;
//    case TimType::tim8:
//      return 6;
//      break;
//    case TimType::tim15:
//      return 7;
//      break;
//    case TimType::tim16:
//      return 8;
//      break;
//    case TimType::tim17:
//      return 9;
//      break;
//  }
//  return 0;
//}
//enum class UsartType : uint8_t {
//  usart1 = 0,
//  usart2,
//  usart3,
//  uart4,
//  uart5,
//};
//constexpr static uint8_t to_idx(UsartType type) {
//  switch (type) {
//    case UsartType::usart1:
//      return 0;
//      break;
//    case UsartType::usart2:
//      return 1;
//      break;
//    case UsartType::usart3:
//      return 2;
//      break;
//    case UsartType::uart4:
//      return 3;
//      break;
//    case UsartType::uart5:
//      return 4;
//      break;
//  }
//  return 0;
//}
//enum class SpiType : uint8_t {
//  spi1 = 0,
//};
//constexpr static uint8_t to_idx(SpiType type) {
//  switch (type) {
//    case SpiType::spi1:
//      return 0;
//      break;
//  }
//  return 0;
//}
//enum class I2cType : uint8_t {
//  i2c1 = 0,
//};
//constexpr static uint8_t to_idx(I2cType type) {
//  switch (type) {
//    case I2cType::i2c1:
//      return 0;
//      break;
//  }
//  return 0;
//}
//enum class DacType : uint8_t {
//  dac1 = 0,
//};
//constexpr static uint8_t to_idx(DacType type) {
//  switch (type) {
//    case DacType::dac1:
//      return 0;
//      break;
//  }
//  return 0;
//}
enum class AdcType : uint8_t {
  adc1 = 0,
};
constexpr static uint8_t to_idx(AdcType type) {
  switch (type) {
    case AdcType::adc1:
      return 0;
      break;
  }
  return 0;
}

enum class GpioPinType : uint8_t {
  pa0 = (0 << 4) | 0,
  pa1 = (0 << 4) | 1,
  pa2 = (0 << 4) | 2,
  pa3 = (0 << 4) | 3,
  pa4 = (0 << 4) | 4,
  pa5 = (0 << 4) | 5,
  pa6 = (0 << 4) | 6,
  pa7 = (0 << 4) | 7,
  pa8 = (0 << 4) | 8,
  pa9 = (0 << 4) | 9,
  pa10 = (0 << 4) | 10,
  pa11 = (0 << 4) | 11,
  pa12 = (0 << 4) | 12,
  pa13 = (0 << 4) | 13,
  pa14 = (0 << 4) | 14,
  pa15 = (0 << 4) | 15,
  pb0 = (1 << 4) | 0,
  pb1 = (1 << 4) | 1,
  pb2 = (1 << 4) | 2,
  pb3 = (1 << 4) | 3,
  pb4 = (1 << 4) | 4,
  pb5 = (1 << 4) | 5,
  pb6 = (1 << 4) | 6,
  pb7 = (1 << 4) | 7,
  pb8 = (1 << 4) | 8,
  pb9 = (1 << 4) | 9,
  pb10 = (1 << 4) | 10,
  pb11 = (1 << 4) | 11,
  pb12 = (1 << 4) | 12,
  pb13 = (1 << 4) | 13,
  pb14 = (1 << 4) | 14,
  pb15 = (1 << 4) | 15,
  pc0 = (2 << 4) | 0,
  pc1 = (2 << 4) | 1,
  pc2 = (2 << 4) | 2,
  pc3 = (2 << 4) | 3,
  pc4 = (2 << 4) | 4,
  pc5 = (2 << 4) | 5,
  pc6 = (2 << 4) | 6,
  pc7 = (2 << 4) | 7,
  pc8 = (2 << 4) | 8,
  pc9 = (2 << 4) | 9,
  pc10 = (2 << 4) | 10,
  pc11 = (2 << 4) | 11,
  pc12 = (2 << 4) | 12,
  pc13 = (2 << 4) | 13,
  pc14 = (2 << 4) | 14,
  pc15 = (2 << 4) | 15,
  pd0 = (3 << 4) | 0,
  pd1 = (3 << 4) | 1,
  pd2 = (3 << 4) | 2,
  pd3 = (3 << 4) | 3,
  pd4 = (3 << 4) | 4,
  pd5 = (3 << 4) | 5,
  pd6 = (3 << 4) | 6,
  pd7 = (3 << 4) | 7,
  pd8 = (3 << 4) | 8,
  pd9 = (3 << 4) | 9,
  pd10 = (3 << 4) | 10,
  pd11 = (3 << 4) | 11,
  pd12 = (3 << 4) | 12,
  pd13 = (3 << 4) | 13,
  pd14 = (3 << 4) | 14,
  pd15 = (3 << 4) | 15,
  //pe0 = (4 << 4) | 0,
  //pe1 = (4 << 4) | 1,
  //pe2 = (4 << 4) | 2,
  //pe3 = (4 << 4) | 3,
  //pe4 = (4 << 4) | 4,
  //pe5 = (4 << 4) | 5,
  //pe6 = (4 << 4) | 6,
  //pe7 = (4 << 4) | 7,
  //pe8 = (4 << 4) | 8,
  //pe9 = (4 << 4) | 9,
  //pe10 = (4 << 4) | 10,
  //pe11 = (4 << 4) | 11,
  //pe12 = (4 << 4) | 12,
  //pe13 = (4 << 4) | 13,
  //pe14 = (4 << 4) | 14,
  //pe15 = (4 << 4) | 15,
  pf0 = (5 << 4) | 0,
  pf1 = (5 << 4) | 1,
  pf2 = (5 << 4) | 2,
  pf3 = (5 << 4) | 3,
  pf4 = (5 << 4) | 4,
  pf5 = (5 << 4) | 5,
  pf6 = (5 << 4) | 6,
  pf7 = (5 << 4) | 7,
  pf8 = (5 << 4) | 8,
  pf9 = (5 << 4) | 9,
  pf10 = (5 << 4) | 10,
  pf11 = (5 << 4) | 11,
  pf12 = (5 << 4) | 12,
  pf13 = (5 << 4) | 13,
  pf14 = (5 << 4) | 14,
  pf15 = (5 << 4) | 15,
};

constexpr static GpioType extract_gpio_type(GpioPinType pin_type) {
  switch (pin_type) {
    case GpioPinType::pa0:
    case GpioPinType::pa1:
    case GpioPinType::pa2:
    case GpioPinType::pa3:
    case GpioPinType::pa4:
    case GpioPinType::pa5:
    case GpioPinType::pa6:
    case GpioPinType::pa7:
    case GpioPinType::pa8:
    case GpioPinType::pa9:
    case GpioPinType::pa10:
    case GpioPinType::pa11:
    case GpioPinType::pa12:
    case GpioPinType::pa13:
    case GpioPinType::pa14:
    case GpioPinType::pa15:
      return GpioType::gpioa;
      [[fallthrough]];
    case GpioPinType::pb0:
    case GpioPinType::pb1:
    case GpioPinType::pb2:
    case GpioPinType::pb3:
    case GpioPinType::pb4:
    case GpioPinType::pb5:
    case GpioPinType::pb6:
    case GpioPinType::pb7:
    case GpioPinType::pb8:
    case GpioPinType::pb9:
    case GpioPinType::pb10:
    case GpioPinType::pb11:
    case GpioPinType::pb12:
    case GpioPinType::pb13:
    case GpioPinType::pb14:
    case GpioPinType::pb15:
      return GpioType::gpiob;
      [[fallthrough]];
    case GpioPinType::pc0:
    case GpioPinType::pc1:
    case GpioPinType::pc2:
    case GpioPinType::pc3:
    case GpioPinType::pc4:
    case GpioPinType::pc5:
    case GpioPinType::pc6:
    case GpioPinType::pc7:
    case GpioPinType::pc8:
    case GpioPinType::pc9:
    case GpioPinType::pc10:
    case GpioPinType::pc11:
    case GpioPinType::pc12:
    case GpioPinType::pc13:
    case GpioPinType::pc14:
    case GpioPinType::pc15:
      return GpioType::gpioc;
      [[fallthrough]];
    case GpioPinType::pd0:
    case GpioPinType::pd1:
    case GpioPinType::pd2:
    case GpioPinType::pd3:
    case GpioPinType::pd4:
    case GpioPinType::pd5:
    case GpioPinType::pd6:
    case GpioPinType::pd7:
    case GpioPinType::pd8:
    case GpioPinType::pd9:
    case GpioPinType::pd10:
    case GpioPinType::pd11:
    case GpioPinType::pd12:
    case GpioPinType::pd13:
    case GpioPinType::pd14:
    case GpioPinType::pd15:
      return GpioType::gpiod;
      [[fallthrough]];
    //case GpioPinType::pe0:
    //case GpioPinType::pe1:
    //case GpioPinType::pe2:
    //case GpioPinType::pe3:
    //case GpioPinType::pe4:
    //case GpioPinType::pe5:
    //case GpioPinType::pe6:
    //case GpioPinType::pe7:
    //case GpioPinType::pe8:
    //case GpioPinType::pe9:
    //case GpioPinType::pe10:
    //case GpioPinType::pe11:
    //case GpioPinType::pe12:
    //case GpioPinType::pe13:
    //case GpioPinType::pe14:
    //case GpioPinType::pe15:
    //  return GpioType::gpioe;
    //  [[fallthrough]];
    case GpioPinType::pf0:
    case GpioPinType::pf1:
    case GpioPinType::pf2:
    case GpioPinType::pf3:
    case GpioPinType::pf4:
    case GpioPinType::pf5:
    case GpioPinType::pf6:
    case GpioPinType::pf7:
    case GpioPinType::pf8:
    case GpioPinType::pf9:
    case GpioPinType::pf10:
    case GpioPinType::pf11:
    case GpioPinType::pf12:
    case GpioPinType::pf13:
    case GpioPinType::pf14:
    case GpioPinType::pf15:
      return GpioType::gpiof;
      [[fallthrough]];
    default:
      break;
  }
  return GpioType::gpioa; // never reach
}

constexpr static uint8_t extract_pin_idx(GpioPinType pin_type) {
  switch (pin_type) {
    case GpioPinType::pa0:
      return 0;
    case GpioPinType::pa1:
      return 1;
    case GpioPinType::pa2:
      return 2;
    case GpioPinType::pa3:
      return 3;
    case GpioPinType::pa4:
      return 4;
    case GpioPinType::pa5:
      return 5;
    case GpioPinType::pa6:
      return 6;
    case GpioPinType::pa7:
      return 7;
    case GpioPinType::pa8:
      return 8;
    case GpioPinType::pa9:
      return 9;
    case GpioPinType::pa10:
      return 10;
    case GpioPinType::pa11:
      return 11;
    case GpioPinType::pa12:
      return 12;
    case GpioPinType::pa13:
      return 13;
    case GpioPinType::pa14:
      return 14;
    case GpioPinType::pa15:
      return 15;
    case GpioPinType::pb0:
      return 0;
    case GpioPinType::pb1:
      return 1;
    case GpioPinType::pb2:
      return 2;
    case GpioPinType::pb3:
      return 3;
    case GpioPinType::pb4:
      return 4;
    case GpioPinType::pb5:
      return 5;
    case GpioPinType::pb6:
      return 6;
    case GpioPinType::pb7:
      return 7;
    case GpioPinType::pb8:
      return 8;
    case GpioPinType::pb9:
      return 9;
    case GpioPinType::pb10:
      return 10;
    case GpioPinType::pb11:
      return 11;
    case GpioPinType::pb12:
      return 12;
    case GpioPinType::pb13:
      return 13;
    case GpioPinType::pb14:
      return 14;
    case GpioPinType::pb15:
      return 15;
    case GpioPinType::pc0:
      return 0;
    case GpioPinType::pc1:
      return 1;
    case GpioPinType::pc2:
      return 2;
    case GpioPinType::pc3:
      return 3;
    case GpioPinType::pc4:
      return 4;
    case GpioPinType::pc5:
      return 5;
    case GpioPinType::pc6:
      return 6;
    case GpioPinType::pc7:
      return 7;
    case GpioPinType::pc8:
      return 8;
    case GpioPinType::pc9:
      return 9;
    case GpioPinType::pc10:
      return 10;
    case GpioPinType::pc11:
      return 11;
    case GpioPinType::pc12:
      return 12;
    case GpioPinType::pc13:
      return 13;
    case GpioPinType::pc14:
      return 14;
    case GpioPinType::pc15:
      return 15;
    case GpioPinType::pd0:
      return 0;
    case GpioPinType::pd1:
      return 1;
    case GpioPinType::pd2:
      return 2;
    case GpioPinType::pd3:
      return 3;
    case GpioPinType::pd4:
      return 4;
    case GpioPinType::pd5:
      return 5;
    case GpioPinType::pd6:
      return 6;
    case GpioPinType::pd7:
      return 7;
    case GpioPinType::pd8:
      return 8;
    case GpioPinType::pd9:
      return 9;
    case GpioPinType::pd10:
      return 10;
    case GpioPinType::pd11:
      return 11;
    case GpioPinType::pd12:
      return 12;
    case GpioPinType::pd13:
      return 13;
    case GpioPinType::pd14:
      return 14;
    case GpioPinType::pd15:
      return 15;
    //case GpioPinType::pe0:
    //  return 0;
    //case GpioPinType::pe1:
    //  return 1;
    //case GpioPinType::pe2:
    //  return 2;
    //case GpioPinType::pe3:
    //  return 3;
    //case GpioPinType::pe4:
    //  return 4;
    //case GpioPinType::pe5:
    //  return 5;
    //case GpioPinType::pe6:
    //  return 6;
    //case GpioPinType::pe7:
    //  return 7;
    //case GpioPinType::pe8:
    //  return 8;
    //case GpioPinType::pe9:
    //  return 9;
    //case GpioPinType::pe10:
    //  return 10;
    //case GpioPinType::pe11:
    //  return 11;
    //case GpioPinType::pe12:
    //  return 12;
    //case GpioPinType::pe13:
    //  return 13;
    //case GpioPinType::pe14:
    //  return 14;
    //case GpioPinType::pe15:
    //  return 15;
    case GpioPinType::pf0:
      return 0;
    case GpioPinType::pf1:
      return 1;
    case GpioPinType::pf2:
      return 2;
    case GpioPinType::pf3:
      return 3;
    case GpioPinType::pf4:
      return 4;
    case GpioPinType::pf5:
      return 5;
    case GpioPinType::pf6:
      return 6;
    case GpioPinType::pf7:
      return 7;
    case GpioPinType::pf8:
      return 8;
    case GpioPinType::pf9:
      return 9;
    case GpioPinType::pf10:
      return 10;
    case GpioPinType::pf11:
      return 11;
    case GpioPinType::pf12:
      return 12;
    case GpioPinType::pf13:
      return 13;
    case GpioPinType::pf14:
      return 14;
    case GpioPinType::pf15:
      return 15;
    default:
      break;
  }
  return 0; // never reach
}

enum class BusType : uint8_t {
  ahb = 0,
  apb1,
  apb2,
};

typedef struct {
  volatile uint32_t MODER;
  volatile uint32_t OTYPER;
  volatile uint32_t OSPEEDR;
  volatile uint32_t PUPDR;
  volatile uint32_t IDR;
  volatile uint32_t ODR;
  volatile uint32_t BSRR;
  volatile uint32_t LCKR;
  volatile uint32_t AFR[2];
  volatile uint32_t BRR;
  volatile uint32_t TG; // extended
  volatile uint32_t _reserved[3]; // 0x30-0x38
  volatile uint32_t OSPD1; // extended
} GPIO_TypeDef ;

typedef struct {
  volatile uint32_t CR;
  volatile uint32_t CFGR;
  volatile uint32_t CIR;
  volatile uint32_t APB2RSTR;
  volatile uint32_t APB1RSTR;
  volatile uint32_t AHBENR;
  volatile uint32_t APB2ENR;
  volatile uint32_t APB1ENR;
  volatile uint32_t BDCR;
  volatile uint32_t CSR;
  volatile uint32_t AHBRSTR;
  volatile uint32_t CFGR2;
  volatile uint32_t CFGR3;
  volatile uint32_t CTL1; // extended
} RCC_TypeDef;

typedef struct {
  volatile uint32_t STAT;           // 0x00
  volatile uint32_t CTL0;           // 0x04
  volatile uint32_t CTL1;           // 0x08
  volatile uint32_t SAMPT0;         // 0x0C
  volatile uint32_t SAMPT1;         // 0x10
  volatile uint32_t _reserved1[4];  // 0x14, 0x18, 0x1C, 0x20
  volatile uint32_t WDHT;           // 0x24
  volatile uint32_t WDLT;           // 0x28
  volatile uint32_t RSQ0;           // 0x2C
  volatile uint32_t RSQ1;           // 0x30
  volatile uint32_t RSQ2;           // 0x34
  volatile uint32_t _reserved2[5];  // 0x38, 0x3C, 0x40, 0x44, 0x48
  volatile uint32_t RDATA;          // 0x4C
  volatile uint32_t _reserved3[12]; // 0x50-0x7C
  volatile uint32_t OVSAMPCTL;      // 0x80
} ADC_TypeDef;

#define GPIOA_ ((GPIO_TypeDef*)GPIOA_BASE)
#define GPIOB_ ((GPIO_TypeDef*)GPIOB_BASE)
#define GPIOC_ ((GPIO_TypeDef*)GPIOC_BASE)
#define GPIOD_ ((GPIO_TypeDef*)GPIOD_BASE)
#define GPIOF_ ((GPIO_TypeDef*)GPIOF_BASE)

#define RCC_ ((RCC_TypeDef*)RCC_BASE)

const std::array<volatile uint32_t*, 3> GD32_BUS_TO_ENR_ADDRESS = {
  &(RCC_->AHBENR),
  &(RCC_->APB1ENR),
  &(RCC_->APB2ENR),
};

const std::array<volatile uint32_t*, 3> GD32_BUS_TO_RSTR_ADDRESS = {
  &(RCC_->AHBRSTR),
  &(RCC_->APB1RSTR),
  &(RCC_->APB2RSTR),
};

#include <igb_gd32/base/_info.hpp>

constexpr struct PeriphInfo {
  const std::array<GpioInfo, 5> gpio {
    GpioInfo {
      .periph_type = PeriphType::gpioa,
      .p_gpio = GPIOA_,
      .addr = GPIOA_BASE,
      .bus = PeriphBusInfo { BusType::ahb, (uint32_t)1 << 17},
    },
    GpioInfo {
      .periph_type = PeriphType::gpiob,
      .p_gpio = GPIOB_,
      .addr = GPIOB_BASE,
      .bus = PeriphBusInfo { BusType::ahb, (uint32_t)1 << 18},
    },
    GpioInfo {
      .periph_type = PeriphType::gpioc,
      .p_gpio = GPIOC_,
      .addr = GPIOC_BASE,
      .bus = PeriphBusInfo { BusType::ahb, (uint32_t)1 << 19},
    },
    GpioInfo {
      .periph_type = PeriphType::gpiod,
      .p_gpio = GPIOD_,
      .addr = GPIOD_BASE,
      .bus = PeriphBusInfo { BusType::ahb, (uint32_t)1 << 20},
    },
    GpioInfo {
      .periph_type = PeriphType::gpiof,
      .p_gpio = GPIOF_,
      .addr = GPIOF_BASE,
      .bus = PeriphBusInfo { BusType::ahb, (uint32_t)1 << 22},
    },
  };
  //const TscInfo tsc {
  //  .periph_type = PeriphType::tsc,
  //  .p_tsc = TSC,
  //  .addr = TSC_BASE,
  //  .irqn = EXTI2_TSC_IRQn,
  //  .bus = PeriphBusInfo { BusType::ahb, (uint32_t)1 << 24},
  //};
  const RccInfo rcc {
    .periph_type = PeriphType::rcc,
    .p_rcc = RCC_,
    .addr = RCC_BASE,
  };
  //const std::array<DmaInfo, 2> dma {
  //  DmaInfo {
  //    .periph_type = PeriphType::dma1,
  //    .p_dma = DMA1,
  //    .addr = DMA1_BASE,
  //    .bus = PeriphBusInfo { BusType::ahb, (uint32_t)1 << 0},
  //    .channels = {
  //
  //      DmaChannelInfo {
  //        .exists = true,
  //        .p_dma_channel = DMA1_Channel1,
  //        .addr = DMA1_Channel1_BASE,
  //        .irqn = DMA1_Channel1_IRQn
  //      },
  //
  //      DmaChannelInfo {
  //        .exists = true,
  //        .p_dma_channel = DMA1_Channel2,
  //        .addr = DMA1_Channel2_BASE,
  //        .irqn = DMA1_Channel2_IRQn
  //      },
  //
  //      DmaChannelInfo {
  //        .exists = true,
  //        .p_dma_channel = DMA1_Channel3,
  //        .addr = DMA1_Channel3_BASE,
  //        .irqn = DMA1_Channel3_IRQn
  //      },
  //
  //      DmaChannelInfo {
  //        .exists = true,
  //        .p_dma_channel = DMA1_Channel4,
  //        .addr = DMA1_Channel4_BASE,
  //        .irqn = DMA1_Channel4_IRQn
  //      },
  //
  //      DmaChannelInfo {
  //        .exists = true,
  //        .p_dma_channel = DMA1_Channel5,
  //        .addr = DMA1_Channel5_BASE,
  //        .irqn = DMA1_Channel5_IRQn
  //      },
  //
  //      DmaChannelInfo {
  //        .exists = true,
  //        .p_dma_channel = DMA1_Channel6,
  //        .addr = DMA1_Channel6_BASE,
  //        .irqn = DMA1_Channel6_IRQn
  //      },
  //
  //      DmaChannelInfo {
  //        .exists = true,
  //        .p_dma_channel = DMA1_Channel7,
  //        .addr = DMA1_Channel7_BASE,
  //        .irqn = DMA1_Channel7_IRQn
  //      },
  //
  //      DmaChannelInfo {
  //        .exists = false,
  //        .p_dma_channel = (DMA_Channel_TypeDef*)0,
  //        .addr = 0,
  //        .irqn = (IRQn_Type)0
  //      },
  //    },
  //  },
  //  DmaInfo {
  //    .periph_type = PeriphType::dma2,
  //    .p_dma = DMA2,
  //    .addr = DMA2_BASE,
  //    .bus = PeriphBusInfo { BusType::ahb, (uint32_t)1 << 1},
  //    .channels = {
  //
  //      DmaChannelInfo {
  //        .exists = true,
  //        .p_dma_channel = DMA2_Channel1,
  //        .addr = DMA2_Channel1_BASE,
  //        .irqn = DMA2_Channel1_IRQn
  //      },
  //
  //      DmaChannelInfo {
  //        .exists = true,
  //        .p_dma_channel = DMA2_Channel2,
  //        .addr = DMA2_Channel2_BASE,
  //        .irqn = DMA2_Channel2_IRQn
  //      },
  //
  //      DmaChannelInfo {
  //        .exists = true,
  //        .p_dma_channel = DMA2_Channel3,
  //        .addr = DMA2_Channel3_BASE,
  //        .irqn = DMA2_Channel3_IRQn
  //      },
  //
  //      DmaChannelInfo {
  //        .exists = true,
  //        .p_dma_channel = DMA2_Channel4,
  //        .addr = DMA2_Channel4_BASE,
  //        .irqn = DMA2_Channel4_IRQn
  //      },
  //
  //      DmaChannelInfo {
  //        .exists = true,
  //        .p_dma_channel = DMA2_Channel5,
  //        .addr = DMA2_Channel5_BASE,
  //        .irqn = DMA2_Channel5_IRQn
  //      },
  //
  //      DmaChannelInfo {
  //        .exists = false,
  //        .p_dma_channel = (DMA_Channel_TypeDef*)0,
  //        .addr = 0,
  //        .irqn = (IRQn_Type)0
  //      },
  //
  //      DmaChannelInfo {
  //        .exists = false,
  //        .p_dma_channel = (DMA_Channel_TypeDef*)0,
  //        .addr = 0,
  //        .irqn = (IRQn_Type)0
  //      },
  //
  //      DmaChannelInfo {
  //        .exists = false,
  //        .p_dma_channel = (DMA_Channel_TypeDef*)0,
  //        .addr = 0,
  //        .irqn = (IRQn_Type)0
  //      },
  //    },
  //  },
  //};
  //const std::array<TimInfo, 10> tim {
  //  TimInfo {
  //    .periph_type = PeriphType::tim1,
  //    .category = TimCategory::ADVANCED,
  //    .p_tim = TIM1,
  //    .addr = TIM1_BASE,
  //    .irqn = TIM1_UP_TIM16_IRQn,
  //    .bus = PeriphBusInfo { BusType::apb2, (uint32_t)1 << 11},
  //  },
  //  TimInfo {
  //    .periph_type = PeriphType::tim2,
  //    .category = TimCategory::GENERAL,
  //    .p_tim = TIM2,
  //    .addr = TIM2_BASE,
  //    .irqn = TIM2_IRQn,
  //    .bus = PeriphBusInfo { BusType::apb1, (uint32_t)1 << 0},
  //  },
  //  TimInfo {
  //    .periph_type = PeriphType::tim3,
  //    .category = TimCategory::GENERAL,
  //    .p_tim = TIM3,
  //    .addr = TIM3_BASE,
  //    .irqn = TIM3_IRQn,
  //    .bus = PeriphBusInfo { BusType::apb1, (uint32_t)1 << 1},
  //  },
  //  TimInfo {
  //    .periph_type = PeriphType::tim4,
  //    .category = TimCategory::GENERAL,
  //    .p_tim = TIM4,
  //    .addr = TIM4_BASE,
  //    .irqn = TIM4_IRQn,
  //    .bus = PeriphBusInfo { BusType::apb1, (uint32_t)1 << 2},
  //  },
  //  TimInfo {
  //    .periph_type = PeriphType::tim6,
  //    .category = TimCategory::BASIC,
  //    .p_tim = TIM6,
  //    .addr = TIM6_BASE,
  //    .irqn = TIM6_DAC_IRQn,
  //    .bus = PeriphBusInfo { BusType::apb1, (uint32_t)1 << 4},
  //  },
  //  TimInfo {
  //    .periph_type = PeriphType::tim7,
  //    .category = TimCategory::BASIC,
  //    .p_tim = TIM7,
  //    .addr = TIM7_BASE,
  //    .irqn = TIM7_IRQn,
  //    .bus = PeriphBusInfo { BusType::apb1, (uint32_t)1 << 5},
  //  },
  //  TimInfo {
  //    .periph_type = PeriphType::tim8,
  //    .category = TimCategory::ADVANCED,
  //    .p_tim = TIM8,
  //    .addr = TIM8_BASE,
  //    .irqn = TIM8_UP_IRQn,
  //    .bus = PeriphBusInfo { BusType::apb2, (uint32_t)1 << 13},
  //  },
  //  TimInfo {
  //    .periph_type = PeriphType::tim15,
  //    .category = TimCategory::GENERAL,
  //    .p_tim = TIM15,
  //    .addr = TIM15_BASE,
  //    .irqn = TIM1_BRK_TIM15_IRQn,
  //    .bus = PeriphBusInfo { BusType::apb2, (uint32_t)1 << 16},
  //  },
  //  TimInfo {
  //    .periph_type = PeriphType::tim16,
  //    .category = TimCategory::GENERAL,
  //    .p_tim = TIM16,
  //    .addr = TIM16_BASE,
  //    .irqn = TIM1_UP_TIM16_IRQn,
  //    .bus = PeriphBusInfo { BusType::apb2, (uint32_t)1 << 17},
  //  },
  //  TimInfo {
  //    .periph_type = PeriphType::tim17,
  //    .category = TimCategory::GENERAL,
  //    .p_tim = TIM17,
  //    .addr = TIM17_BASE,
  //    .irqn = TIM1_TRG_COM_TIM17_IRQn,
  //    .bus = PeriphBusInfo { BusType::apb2, (uint32_t)1 << 18},
  //  },
  //};
  //const std::array<UsartInfo, 5> usart {
  //  UsartInfo {
  //    .periph_type = PeriphType::usart1,
  //    .p_usart = USART1,
  //    .addr = USART1_BASE,
  //    .irqn = USART1_IRQn,
  //    .bus = PeriphBusInfo { BusType::apb2, (uint32_t)1 << 14},
  //  },
  //  UsartInfo {
  //    .periph_type = PeriphType::usart2,
  //    .p_usart = USART2,
  //    .addr = USART2_BASE,
  //    .irqn = USART2_IRQn,
  //    .bus = PeriphBusInfo { BusType::apb1, (uint32_t)1 << 17},
  //  },
  //  UsartInfo {
  //    .periph_type = PeriphType::usart3,
  //    .p_usart = USART3,
  //    .addr = USART3_BASE,
  //    .irqn = USART3_IRQn,
  //    .bus = PeriphBusInfo { BusType::apb1, (uint32_t)1 << 18},
  //  },
  //  UsartInfo {
  //    .periph_type = PeriphType::uart4,
  //    .p_usart = UART4,
  //    .addr = UART4_BASE,
  //    .irqn = UART4_IRQn,
  //    .bus = PeriphBusInfo { BusType::apb1, (uint32_t)1 << 19},
  //  },
  //  UsartInfo {
  //    .periph_type = PeriphType::uart5,
  //    .p_usart = UART5,
  //    .addr = UART5_BASE,
  //    .irqn = UART5_IRQn,
  //    .bus = PeriphBusInfo { BusType::apb1, (uint32_t)1 << 20},
  //  },
  //};
  //const std::array<SpiInfo, 1> spi {
  //  SpiInfo {
  //    .periph_type = PeriphType::spi1,
  //    .p_spi = SPI1,
  //    .addr = SPI1_BASE,
  //    .bus = PeriphBusInfo { BusType::apb2, (uint32_t)1 << 12},
  //  },
  //};
  //const ExtiInfo exti {
  //  .periph_type = PeriphType::exti,
  //  .p_exti = EXTI,
  //  .addr = EXTI_BASE,
  //  .line_irqns = {
  //    EXTI0_IRQn,
  //    EXTI1_IRQn,
  //    EXTI2_TSC_IRQn,
  //    EXTI3_IRQn,
  //    EXTI4_IRQn,
  //    EXTI9_5_IRQn,
  //    EXTI9_5_IRQn,
  //    EXTI9_5_IRQn,
  //    EXTI9_5_IRQn,
  //    EXTI9_5_IRQn,
  //    EXTI15_10_IRQn,
  //    EXTI15_10_IRQn,
  //    EXTI15_10_IRQn,
  //    EXTI15_10_IRQn,
  //    EXTI15_10_IRQn,
  //    EXTI15_10_IRQn,
  //  },
  //};
  //const std::array<I2cInfo, 1> i2c {
  //  I2cInfo {
  //    .periph_type = PeriphType::i2c1,
  //    .p_i2c = I2C1,
  //    .addr = I2C1_BASE,
  //    .bus = PeriphBusInfo { BusType::apb1, (uint32_t)1 << 21},
  //  },
  //};
  //const std::array<DacInfo, 1> dac {
  //  DacInfo {
  //    .periph_type = PeriphType::dac1,
  //    .p_dac = DAC1,
  //    .addr = DAC1_BASE,
  //    .irqn = TIM6_DAC_IRQn,
  //    .bus = PeriphBusInfo { BusType::apb1, (uint32_t)1 << 29},
  //  },
  //};
  const std::array<AdcInfo, 1> adc {
    AdcInfo {
      .periph_type = PeriphType::adc1,
      .p_adc = (ADC_TypeDef*)ADC_BASE,
      .addr = ADC_BASE,
      .irqn = ADC_CMP_IRQn,
      .bus = PeriphBusInfo { BusType::apb2, (uint32_t)1 << 9},
    },
  };
  //const SysCfgInfo syscfg {
  //  .periph_type = PeriphType::syscfg,
  //  .p_syscfg = SYSCFG,
  //  .addr = SYSCFG_BASE,
  //  .bus = PeriphBusInfo { BusType::apb2, (uint32_t)1 << 0},
  //};
} GD32_PERIPH_INFO;

enum class GpioAf : uint8_t {
  af0 = 0,
  af1,
  af2,
  af3,
  af4,
  af5,
  af6,
  af7,
};

constexpr static std::optional<GpioAf> get_af_idx(PeriphType periph_type, GpioPinType gpio_pin) {
  switch (gpio_pin) {
    case GpioPinType::pa0:
      switch (periph_type) {
        case PeriphType::usart0:
        case PeriphType::usart1:
          return GpioAf::af1;
        case PeriphType::tim1:
          return GpioAf::af2;
        case PeriphType::tsc:
          return GpioAf::af3;
        case PeriphType::i2c1:
          return GpioAf::af4;
        default:
          break;
      }
      break;
    case GpioPinType::pa1:
      switch (periph_type) {
        case PeriphType::usart0:
        case PeriphType::usart1:
          return GpioAf::af1;
        case PeriphType::tim1:
          return GpioAf::af2;
        case PeriphType::tsc:
          return GpioAf::af3;
        case PeriphType::i2c1:
          return GpioAf::af4;
        default:
          break;
      }
      break;
    case GpioPinType::pa2:
      switch (periph_type) {
        case PeriphType::tim14:
          return GpioAf::af0;
        case PeriphType::usart0:
        case PeriphType::usart1:
          return GpioAf::af1;
        case PeriphType::tim1:
          return GpioAf::af2;
        case PeriphType::tsc:
          return GpioAf::af3;
        default:
          break;
      }
      break;
    case GpioPinType::pa3:
      switch (periph_type) {
        case PeriphType::tim14:
          return GpioAf::af0;
        case PeriphType::usart0:
        case PeriphType::usart1:
          return GpioAf::af1;
        case PeriphType::tim1:
          return GpioAf::af2;
        case PeriphType::tsc:
          return GpioAf::af3;
        default:
          break;
      }
      break;
    case GpioPinType::pa4:
      switch (periph_type) {
        case PeriphType::spi0:
        case PeriphType::i2s0:
          return GpioAf::af0;
        case PeriphType::usart0:
        case PeriphType::usart1:
          return GpioAf::af1;
        case PeriphType::tsc:
          return GpioAf::af3;
        case PeriphType::tim13:
          return GpioAf::af4;
        case PeriphType::spi1:
          return GpioAf::af6;
        default:
          break;
      }
      break;
    case GpioPinType::pa5:
      switch (periph_type) {
        case PeriphType::spi0:
        case PeriphType::i2s0:
          return GpioAf::af0;
        case PeriphType::tim1:
          return GpioAf::af2;
        case PeriphType::tsc:
          return GpioAf::af3;
        default:
          break;
      }
      break;
    case GpioPinType::pa6:
      switch (periph_type) {
        case PeriphType::spi0:
        case PeriphType::i2s0:
          return GpioAf::af0;
        case PeriphType::tim2:
          return GpioAf::af1;
        case PeriphType::tim0:
          return GpioAf::af2;
        case PeriphType::tsc:
          return GpioAf::af3;
        case PeriphType::tim15:
          return GpioAf::af5;
        default:
          break;
      }
      break;
    case GpioPinType::pa7:
      switch (periph_type) {
        case PeriphType::spi0:
        case PeriphType::i2s0:
          return GpioAf::af0;
        case PeriphType::tim2:
          return GpioAf::af1;
        case PeriphType::tim0:
          return GpioAf::af2;
        case PeriphType::tsc:
          return GpioAf::af3;
        case PeriphType::tim13:
          return GpioAf::af4;
        case PeriphType::tim16:
          return GpioAf::af5;
        default:
          break;
      }
      break;
    case GpioPinType::pa8:
      switch (periph_type) {
        case PeriphType::usart0:
          return GpioAf::af1;
        case PeriphType::tim0:
          return GpioAf::af2;
        case PeriphType::usart1:
          return GpioAf::af4;
        case PeriphType::usb:
          return GpioAf::af5;
        default:
          break;
      }
      break;
    case GpioPinType::pa9:
      switch (periph_type) {
        case PeriphType::tim14:
          return GpioAf::af0;
        case PeriphType::usart0:
          return GpioAf::af1;
        case PeriphType::tim0:
          return GpioAf::af2;
        case PeriphType::tsc:
          return GpioAf::af3;
        case PeriphType::i2c0:
          return GpioAf::af4;
        case PeriphType::usb:
          return GpioAf::af5;
        default:
          break;
      }
      break;
    case GpioPinType::pa10:
      switch (periph_type) {
        case PeriphType::tim16:
          return GpioAf::af0;
        case PeriphType::usart0:
          return GpioAf::af1;
        case PeriphType::tim0:
          return GpioAf::af2;
        case PeriphType::tsc:
          return GpioAf::af3;
        case PeriphType::i2c0:
          return GpioAf::af4;
        case PeriphType::usb:
          return GpioAf::af5;
        default:
          break;
      }
      break;
    case GpioPinType::pa11:
      switch (periph_type) {
        case PeriphType::usart0:
          return GpioAf::af1;
        case PeriphType::tim0:
          return GpioAf::af2;
        case PeriphType::tsc:
          return GpioAf::af3;
        case PeriphType::spi1:
          return GpioAf::af6;
        default:
          break;
      }
      break;
    case GpioPinType::pa12:
      switch (periph_type) {
        case PeriphType::usart0:
          return GpioAf::af1;
        case PeriphType::tim0:
          return GpioAf::af2;
        case PeriphType::tsc:
          return GpioAf::af3;
        case PeriphType::spi1:
          return GpioAf::af6;
        default:
          break;
      }
      break;
    case GpioPinType::pa13:
      switch (periph_type) {
        case PeriphType::spi1:
          return GpioAf::af6;
        default:
          break;
      }
      break;
    case GpioPinType::pa14:
      switch (periph_type) {
        case PeriphType::usart0:
        case PeriphType::usart1:
          return GpioAf::af1;
        case PeriphType::spi1:
          return GpioAf::af6;
        default:
          break;
      }
      break;
    case GpioPinType::pa15:
      switch (periph_type) {
        case PeriphType::spi0:
        case PeriphType::i2s0:
          return GpioAf::af0;
        case PeriphType::usart0:
        case PeriphType::usart1:
          return GpioAf::af1;
        case PeriphType::tim1:
          return GpioAf::af2;
        case PeriphType::spi1:
          return GpioAf::af6;
        default:
          break;
      }
      break;
    case GpioPinType::pb0:
      switch (periph_type) {
        case PeriphType::tim2:
          return GpioAf::af1;
        case PeriphType::tim0:
          return GpioAf::af2;
        case PeriphType::tsc:
          return GpioAf::af3;
        case PeriphType::usart1:
          return GpioAf::af4;
        default:
          break;
      }
      break;
    case GpioPinType::pb1:
      switch (periph_type) {
        case PeriphType::tim13:
          return GpioAf::af0;
        case PeriphType::tim2:
          return GpioAf::af1;
        case PeriphType::tim0:
          return GpioAf::af2;
        case PeriphType::tsc:
          return GpioAf::af3;
        case PeriphType::spi1:
          return GpioAf::af6;
        default:
          break;
      }
      break;
    case GpioPinType::pb2:
      switch (periph_type) {
        case PeriphType::tsc:
          return GpioAf::af3;
        default:
          break;
      }
      break;
    case GpioPinType::pb3:
      switch (periph_type) {
        case PeriphType::spi0:
        case PeriphType::i2s0:
          return GpioAf::af0;
        case PeriphType::tim1:
          return GpioAf::af2;
        case PeriphType::tsc:
          return GpioAf::af3;
        default:
          break;
      }
      break;
    case GpioPinType::pb4:
      switch (periph_type) {
        case PeriphType::spi0:
        case PeriphType::i2s0:
          return GpioAf::af0;
        case PeriphType::tim2:
          return GpioAf::af1;
        case PeriphType::tsc:
          return GpioAf::af3;
        default:
          break;
      }
      break;
    case GpioPinType::pb5:
      switch (periph_type) {
        case PeriphType::spi0:
        case PeriphType::i2s0:
          return GpioAf::af0;
        case PeriphType::tim2:
          return GpioAf::af1;
        case PeriphType::tim15:
          return GpioAf::af2;
        case PeriphType::i2c0:
          return GpioAf::af3;
        default:
          break;
      }
      break;
    case GpioPinType::pb6:
      switch (periph_type) {
        case PeriphType::usart0:
          return GpioAf::af0;
        case PeriphType::i2c0:
          return GpioAf::af1;
        case PeriphType::tim15:
          return GpioAf::af2;
        case PeriphType::tsc:
          return GpioAf::af3;
        default:
          break;
      }
      break;
    case GpioPinType::pb7:
      switch (periph_type) {
        case PeriphType::usart0:
          return GpioAf::af0;
        case PeriphType::i2c0:
          return GpioAf::af1;
        case PeriphType::tim16:
          return GpioAf::af2;
        case PeriphType::tsc:
          return GpioAf::af3;
        default:
          break;
      }
      break;
    case GpioPinType::pb8:
      switch (periph_type) {
        case PeriphType::i2c0:
          return GpioAf::af1;
        case PeriphType::tim15:
          return GpioAf::af2;
        case PeriphType::tsc:
          return GpioAf::af3;
        default:
          break;
      }
      break;
    case GpioPinType::pb9:
      switch (periph_type) {
        case PeriphType::i2c0:
          return GpioAf::af1;
        case PeriphType::tim16:
          return GpioAf::af2;
        case PeriphType::i2s0:
          return GpioAf::af5;
        default:
          break;
      }
      break;
    case GpioPinType::pb10:
      switch (periph_type) {
        case PeriphType::i2c0:
        case PeriphType::i2c1:
          return GpioAf::af1;
        case PeriphType::tim1:
          return GpioAf::af2;
        case PeriphType::tsc:
          return GpioAf::af3;
        case PeriphType::spi1:
          return GpioAf::af6;
        default:
          break;
      }
      break;
    case GpioPinType::pb11:
      switch (periph_type) {
        case PeriphType::i2c0:
        case PeriphType::i2c1:
          return GpioAf::af1;
        case PeriphType::tim1:
          return GpioAf::af2;
        case PeriphType::tsc:
          return GpioAf::af3;
        case PeriphType::spi1:
          return GpioAf::af6;
        default:
          break;
      }
      break;
    case GpioPinType::pb12:
      switch (periph_type) {
        case PeriphType::spi0:
        case PeriphType::spi1:
          return GpioAf::af0;
        case PeriphType::tim0:
          return GpioAf::af2;
        case PeriphType::tsc:
          return GpioAf::af3;
        case PeriphType::i2c1:
          return GpioAf::af4;
        default:
          break;
      }
      break;
    case GpioPinType::pb13:
      switch (periph_type) {
        case PeriphType::spi0:
        case PeriphType::spi1:
          return GpioAf::af0;
        case PeriphType::tim0:
          return GpioAf::af2;
        case PeriphType::tsc:
          return GpioAf::af3;
        default:
          break;
      }
      break;
    case GpioPinType::pb14:
      switch (periph_type) {
        case PeriphType::spi0:
        case PeriphType::spi1:
          return GpioAf::af0;
        case PeriphType::tim14:
          return GpioAf::af1;
        case PeriphType::tim0:
          return GpioAf::af2;
        case PeriphType::tsc:
          return GpioAf::af3;
        default:
          break;
      }
      break;
    case GpioPinType::pb15:
      switch (periph_type) {
        case PeriphType::spi0:
        case PeriphType::spi1:
          return GpioAf::af0;
        case PeriphType::tim14:
          return GpioAf::af1;
        //case PeriphType::tim14: Collision!: TIMER14_CH1 & IMER14_CH0_ON
        //  return GpioAf::af3;
        case PeriphType::tim0:
          return GpioAf::af2;
        default:
          break;
      }
      break;
    case GpioPinType::pc0:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pc1:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pc2:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pc3:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pc4:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pc5:
      switch (periph_type) {
        case PeriphType::tsc:
          return GpioAf::af0;
        default:
          break;
      }
      break;
    case GpioPinType::pc6:
      switch (periph_type) {
        case PeriphType::tim2:
          return GpioAf::af0;
        case PeriphType::i2s0:
          return GpioAf::af2;
        default:
          break;
      }
      break;
    case GpioPinType::pc7:
      switch (periph_type) {
        case PeriphType::tim2:
          return GpioAf::af0;
        default:
          break;
      }
      break;
    case GpioPinType::pc8:
      switch (periph_type) {
        case PeriphType::tim2:
          return GpioAf::af0;
        default:
          break;
      }
      break;
    case GpioPinType::pc9:
      switch (periph_type) {
        case PeriphType::tim2:
          return GpioAf::af0;
        default:
          break;
      }
      break;
    case GpioPinType::pc10:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pc11:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pc12:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pc13:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pc14:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pc15:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pd0:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pd1:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pd2:
      switch (periph_type) {
        case PeriphType::tim2:
          return GpioAf::af0;
        default:
          break;
      }
      break;
    case GpioPinType::pd3:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pd4:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pd5:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pd6:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pd7:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pd8:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pd9:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pd10:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pd11:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pd12:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pd13:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pd14:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pd15:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pf0:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pf1:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pf2:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pf3:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pf4:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pf5:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pf6:
      switch (periph_type) {
        case PeriphType::i2c0:
        case PeriphType::i2c1:
          return GpioAf::af0;
        default:
          break;
      }
      break;
    case GpioPinType::pf7:
      switch (periph_type) {
        case PeriphType::i2c0:
        case PeriphType::i2c1:
          return GpioAf::af0;
        default:
          break;
      }
      break;
    case GpioPinType::pf8:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pf9:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pf10:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pf11:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pf12:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pf13:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pf14:
      switch (periph_type) {
        default:
          break;
      }
      break;
    case GpioPinType::pf15:
      switch (periph_type) {
        default:
          break;
      }
      break;
    default:
      break;
  }
  return std::nullopt;
}


template<typename T>
constexpr static std::optional<PeriphType> as_periph_type(T type) {
  return std::nullopt;
}

template<>
constexpr std::optional<PeriphType> as_periph_type(GpioType type) {
  switch (type) {
    case GpioType::gpioa:
      return PeriphType::gpioa;
    case GpioType::gpiob:
      return PeriphType::gpiob;
    case GpioType::gpioc:
      return PeriphType::gpioc;
    case GpioType::gpiod:
      return PeriphType::gpiod;
    case GpioType::gpiof:
      return PeriphType::gpiof;
  }
  return std::nullopt;
}

template<>
constexpr std::optional<PeriphType> as_periph_type(AdcType type) {
  switch (type) {
    case AdcType::adc1:
      return PeriphType::adc1;
  }
  return std::nullopt;
}

// TODO:
//template<>
//constexpr std::optional<PeriphType> as_periph_type(DmaType type) {
//  switch (type) {
//    case DmaType::dma1:
//      return PeriphType::dma1;
//    case DmaType::dma2:
//      return PeriphType::dma2;
//  }
//  return std::nullopt;
//}
//template<>
//constexpr std::optional<PeriphType> as_periph_type(TimType type) {
//  switch (type) {
//    case TimType::tim1:
//      return PeriphType::tim1;
//    case TimType::tim2:
//      return PeriphType::tim2;
//    case TimType::tim3:
//      return PeriphType::tim3;
//    case TimType::tim4:
//      return PeriphType::tim4;
//    case TimType::tim6:
//      return PeriphType::tim6;
//    case TimType::tim7:
//      return PeriphType::tim7;
//    case TimType::tim8:
//      return PeriphType::tim8;
//    case TimType::tim15:
//      return PeriphType::tim15;
//    case TimType::tim16:
//      return PeriphType::tim16;
//    case TimType::tim17:
//      return PeriphType::tim17;
//  }
//  return std::nullopt;
//}
//template<>
//constexpr std::optional<PeriphType> as_periph_type(UsartType type) {
//  switch (type) {
//    case UsartType::usart1:
//      return PeriphType::usart1;
//    case UsartType::usart2:
//      return PeriphType::usart2;
//    case UsartType::usart3:
//      return PeriphType::usart3;
//    case UsartType::uart4:
//      return PeriphType::uart4;
//    case UsartType::uart5:
//      return PeriphType::uart5;
//  }
//  return std::nullopt;
//}
//template<>
//constexpr std::optional<PeriphType> as_periph_type(SpiType type) {
//  switch (type) {
//    case SpiType::spi1:
//      return PeriphType::spi1;
//  }
//  return std::nullopt;
//}
//template<>
//constexpr std::optional<PeriphType> as_periph_type(I2cType type) {
//  switch (type) {
//    case I2cType::i2c1:
//      return PeriphType::i2c1;
//  }
//  return std::nullopt;
//}
//template<>
//constexpr std::optional<PeriphType> as_periph_type(DacType type) {
//  switch (type) {
//    case DacType::dac1:
//      return PeriphType::dac1;
//  }
//  return std::nullopt;
//}

}
}

#endif /* IGB_GD32_BASE_MCU_DEVICE_GD32F350_H */
