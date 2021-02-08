// modified by centrevillage
/**
  ******************************************************************************
  * @file    twi.h
  * @author  WI6LABS
  * @version V1.0.0
  * @date    01-August-2016
  * @brief   Header for twi module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#ifndef PERIPH_LIB_TWI_H
#define PERIPH_LIB_TWI_H

#include "_define_series_name.h"

/* Includes ------------------------------------------------------------------*/
//#include "stm32_def.h"
//#include "PeripheralPins.h"

#include <igb_stm32/base.hpp>
#include <igb_stm32/periph/gpio.hpp>
#include <igb_stm32/hal.hpp>

/* Exported types ------------------------------------------------------------*/
/* offsetof is a gcc built-in function, this is the manual implementation */
#define OFFSETOF(type, member) ((uint32_t) (&(((type *)(0))->member)))

#ifndef I2C_TIMEOUT_TICK
#define I2C_TIMEOUT_TICK        100
#endif

#define SLAVE_MODE_TRANSMIT     0
#define SLAVE_MODE_RECEIVE      1
#define SLAVE_MODE_LISTEN       2

/* Interrupt priority */
#ifndef I2C_IRQ_PRIO
#define I2C_IRQ_PRIO       2
#endif
#ifndef I2C_IRQ_SUBPRIO
#define I2C_IRQ_SUBPRIO    0
#endif

/* I2C Tx/Rx buffer size */
#if !defined(I2C_TXRX_BUFFER_SIZE)
#define I2C_TXRX_BUFFER_SIZE    32
#elif (I2C_TXRX_BUFFER_SIZE >= 256)
#error I2C buffer size cannot exceed 255
#endif

/* Redefinition of IRQ for F0/G0/L0 families */
#if defined(STM32F0xx) || defined(STM32G0xx) || defined(STM32L0xx)
#if defined(I2C1_BASE)
#define I2C1_EV_IRQn        I2C1_IRQn
#define I2C1_EV_IRQHandler  I2C1_IRQHandler
#endif // defined(I2C1_BASE)
#if defined(I2C2_BASE)
#define I2C2_EV_IRQn        I2C2_IRQn
#define I2C2_EV_IRQHandler  I2C2_IRQHandler
#endif // defined(I2C2_BASE)
/* Only for STM32L0xx */
#if defined(I2C3_BASE)
#define I2C3_EV_IRQn        I2C3_IRQn
#define I2C3_EV_IRQHandler  I2C3_IRQHandler
#endif // defined(I2C3_BASE)
/* Defined but no one has it */
#if defined(I2C4_BASE)
#define I2C4_EV_IRQn        I2C4_IRQn
#define I2C4_EV_IRQHandler  I2C4_IRQHandler
#endif // defined(I2C4_BASE)
#endif /* STM32F0 || STM32G0 || STM32L0 */

template<typename I2C_TYPE>
struct i2c_t {
  /*  The 1st 2 members I2CName i2c
     *  and I2C_HandleTypeDef handle should
     *  be kept as the first members of this struct
     *  to have get_i2c_obj() function work as expected
     */
  I2C_TypeDef  *i2c;
  I2C_HandleTypeDef handle;
  void *__this;
  I2C_TYPE igb_i2c;
  IRQn_Type irq;
#if !defined(STM32F0xx) && !defined(STM32G0xx) && !defined(STM32L0xx)
  IRQn_Type irqER;
#endif /* !STM32F0 && !STM32G0 && !STM32L0 */
  volatile int slaveRxNbData; // Number of accumulated bytes received in Slave mode
  void (*i2c_onSlaveReceive)(i2c_t<I2C_TYPE> *);
  void (*i2c_onSlaveTransmit)(i2c_t<I2C_TYPE> *);
  volatile uint8_t i2cTxRxBuffer[I2C_TXRX_BUFFER_SIZE];
  volatile uint8_t i2cTxRxBufferSize;
  volatile uint8_t slaveMode;
  uint8_t isMaster;
  uint8_t generalCall;
};

///@brief I2C state
typedef enum {
  I2C_OK = 0,
  I2C_DATA_TOO_LONG = 1,
  I2C_NACK_ADDR = 2,
  I2C_NACK_DATA = 3,
  I2C_ERROR = 4,
  I2C_TIMEOUT = 5,
  I2C_BUSY = 6
} i2c_status_e;

/* Exported functions ------------------------------------------------------- */
/**
  * @brief  Default init and setup GPIO and I2C peripheral
  * @param  obj : pointer to i2c_t structure
  * @retval none
  */
template<typename I2C_TYPE>
void i2c_init(i2c_t<I2C_TYPE> *obj) {
  i2c_custom_init(obj, 100000, I2C_ADDRESSINGMODE_7BIT, 0x33);
}

/**
  * @brief  Initialize and setup GPIO and I2C peripheral
  * @param  obj : pointer to i2c_t structure
  * @param  timing : one of the i2c_timing_e
  * @param  addressingMode : I2C_ADDRESSINGMODE_7BIT or I2C_ADDRESSINGMODE_10BIT
  * @param  ownAddress : device address
  * @retval none
  */
template<typename I2C_TYPE>
void i2c_custom_init(i2c_t<I2C_TYPE> *obj, uint32_t timing, uint32_t addressingMode,
                     uint32_t ownAddress) {
  if (obj != NULL) {


    I2C_HandleTypeDef *handle = &(obj->handle);

    obj->i2c = obj->igb_i2c.addr;
    //// Determine the I2C to use
    //I2C_TypeDef *i2c_sda = pinmap_peripheral(obj->sda, PinMap_I2C_SDA);
    //I2C_TypeDef *i2c_scl = pinmap_peripheral(obj->scl, PinMap_I2C_SCL);

    //Pins SDA/SCL must not be NP
    //if (i2c_sda == NP || i2c_scl == NP) {
    //  core_debug("ERROR: at least one I2C pin has no peripheral\n");
    //} else {

      //obj->i2c = pinmap_merge_peripheral(i2c_sda, i2c_scl);

      //if (obj->i2c == NP) {
      //  core_debug("ERROR: I2C pins mismatch\n");

      //} else {

#if defined I2C1_BASE
        // Enable I2C1 clock if not done
        if (obj->i2c == I2C1) {
          __HAL_RCC_I2C1_CLK_ENABLE();
          __HAL_RCC_I2C1_FORCE_RESET();
          __HAL_RCC_I2C1_RELEASE_RESET();

          obj->irq = I2C1_EV_IRQn;
#if !defined(STM32F0xx) && !defined(STM32G0xx) && !defined(STM32L0xx)
          obj->irqER = I2C1_ER_IRQn;
#endif /* !STM32F0xx && !STM32G0xx && !STM32L0xx */
        }
#endif // I2C1_BASE
#if defined I2C2_BASE
        // Enable I2C2 clock if not done
        if (obj->i2c == I2C2) {
          __HAL_RCC_I2C2_CLK_ENABLE();
          __HAL_RCC_I2C2_FORCE_RESET();
          __HAL_RCC_I2C2_RELEASE_RESET();
          obj->irq = I2C2_EV_IRQn;
#if !defined(STM32F0xx) && !defined(STM32G0xx) && !defined(STM32L0xx)
          obj->irqER = I2C2_ER_IRQn;
#endif /* !STM32F0xx && !STM32G0xx && !STM32L0xx */
        }
#endif // I2C2_BASE
#if defined I2C3_BASE
        // Enable I2C3 clock if not done
        if (obj->i2c == I2C3) {
          __HAL_RCC_I2C3_CLK_ENABLE();
          __HAL_RCC_I2C3_FORCE_RESET();
          __HAL_RCC_I2C3_RELEASE_RESET();
          obj->irq = I2C3_EV_IRQn;
#if !defined(STM32L0xx)
          obj->irqER = I2C3_ER_IRQn;
#endif /* !STM32L0xx */
        }
#endif // I2C3_BASE
#if defined I2C4_BASE
        // Enable I2C4 clock if not done
        if (obj->i2c == I2C4) {
          __HAL_RCC_I2C4_CLK_ENABLE();
          __HAL_RCC_I2C4_FORCE_RESET();
          __HAL_RCC_I2C4_RELEASE_RESET();
          obj->irq = I2C4_EV_IRQn;
          obj->irqER = I2C4_ER_IRQn;
        }
#endif // I2C4_BASE
#if defined I2C5_BASE
        // Enable I2C5 clock if not done
        if (obj->i2c == I2C5) {
          __HAL_RCC_I2C5_CLK_ENABLE();
          __HAL_RCC_I2C5_FORCE_RESET();
          __HAL_RCC_I2C5_RELEASE_RESET();
          obj->irq = I2C5_EV_IRQn;
          obj->irqER = I2C5_ER_IRQn;
        }
#endif // I2C5_BASE
#if defined I2C6_BASE
        // Enable I2C6 clock if not done
        if (obj->i2c == I2C6) {
          __HAL_RCC_I2C6_CLK_ENABLE();
          __HAL_RCC_I2C6_FORCE_RESET();
          __HAL_RCC_I2C6_RELEASE_RESET();
          obj->irq = I2C6_EV_IRQn;
          obj->irqER = I2C6_ER_IRQn;
        }
#endif // I2C6_BASE

        /* Configure I2C GPIO pins */
        //pinmap_pinout(obj->scl, PinMap_I2C_SCL);
        //pinmap_pinout(obj->sda, PinMap_I2C_SDA);
        obj->igb_i2c.prepareGpio(obj->igb_i2c.scl_pin);
        obj->igb_i2c.prepareGpio(obj->igb_i2c.sda_pin);

        handle->Instance             = obj->i2c;
#ifdef I2C_TIMING
        handle->Init.Timing          = i2c_getTiming(obj, timing);
#else
        handle->Init.ClockSpeed      = i2c_getTiming(obj, timing);
        /* Standard mode (sm) is up to 100kHz, then it's Fast mode (fm)     */
        /* In fast mode duty cyble bit must be set in CCR register          */
        if (timing > 100000) {
          handle->Init.DutyCycle       = I2C_DUTYCYCLE_16_9;
        } else {
          handle->Init.DutyCycle       = I2C_DUTYCYCLE_2;
        }
#endif
        handle->Init.OwnAddress1     = ownAddress;
        handle->Init.OwnAddress2     = 0;
        handle->Init.AddressingMode  = addressingMode;
        handle->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
        handle->Init.GeneralCallMode = (obj->generalCall == 0) ? I2C_GENERALCALL_DISABLE : I2C_GENERALCALL_ENABLE;
        handle->Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

        handle->State = HAL_I2C_STATE_RESET;

        HAL_NVIC_SetPriority(obj->irq, I2C_IRQ_PRIO, I2C_IRQ_SUBPRIO);
        HAL_NVIC_EnableIRQ(obj->irq);
#if !defined(STM32F0xx) && !defined(STM32G0xx) && !defined(STM32L0xx)
        HAL_NVIC_SetPriority(obj->irqER, I2C_IRQ_PRIO, I2C_IRQ_SUBPRIO);
        HAL_NVIC_EnableIRQ(obj->irqER);
#endif /* !STM32F0xx && !STM32G0xx && !STM32L0xx */

        /* Init the I2C */
        if (HAL_I2C_Init(handle) != HAL_OK) {
          /* Initialization Error */
          Error_Handler();
        }

        /* Initialize default values */
        obj->slaveRxNbData = 0;
        obj->slaveMode = SLAVE_MODE_LISTEN;
      //}
    //}
  }
}

/**
  * @brief  Initialize and setup GPIO and I2C peripheral
  * @param  obj : pointer to i2c_t structure
  * @retval none
  */
template<typename I2C_TYPE>
void i2c_deinit(i2c_t<I2C_TYPE> *obj) {
  HAL_NVIC_DisableIRQ(obj->irq);
#if !defined(STM32F0xx) && !defined(STM32G0xx) && !defined(STM32L0xx)
  HAL_NVIC_DisableIRQ(obj->irqER);
#endif /* !STM32F0xx && !STM32G0xx && !STM32L0xx */
  HAL_I2C_DeInit(&(obj->handle));
}

/**
  * @brief  Setup transmission speed. I2C must be configured before.
  * @param  obj : pointer to i2c_t structure
  * @param  frequency : i2c transmission speed
  * @retval none
  */
template<typename I2C_TYPE>
void i2c_setTiming(i2c_t<I2C_TYPE> *obj, uint32_t frequency) {
  uint32_t f = i2c_getTiming(obj, frequency);
  __HAL_I2C_DISABLE(&(obj->handle));

#ifdef I2C_TIMING
  obj->handle.Init.Timing = f;
#else
  obj->handle.Init.ClockSpeed = f;
  /* Standard mode (sm) is up to 100kHz, then it's Fast mode (fm)     */
  /* In fast mode duty cyble bit must be set in CCR register          */
  if (frequency > 100000) {
    obj->handle.Init.DutyCycle       = I2C_DUTYCYCLE_16_9;
  } else {
    obj->handle.Init.DutyCycle       = I2C_DUTYCYCLE_2;
  }
#endif
  HAL_I2C_Init(&(obj->handle));
  __HAL_I2C_ENABLE(&(obj->handle));
}

/**
  * @brief  I2C error callback.
  * @note   In master mode, the callback is not used because the error is reported
  *         to the Arduino API from i2c_master_write() and i2c_master_read().
  *         In slave mode, there is no mechanism in Arduino API to report an error
  *         so the error callback forces the slave to listen again.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
template<typename I2C_TYPE>
i2c_status_e i2c_master_write(i2c_t<I2C_TYPE> *obj, uint8_t dev_address, uint8_t *data, uint16_t size) {
  i2c_status_e ret = I2C_OK;
  uint32_t tickstart = HAL_GetTick();
  uint32_t delta = 0;
  uint32_t err = 0;

  /* When size is 0, this is usually an I2C scan / ping to check if device is there and ready */
  if (size == 0) {
    ret = i2c_IsDeviceReady(obj, dev_address, 1);
  } else {
#if defined(I2C_OTHER_FRAME)
    uint32_t XferOptions = obj->handle.XferOptions; // save XferOptions value, because handle can be modified by HAL, which cause issue in case of NACK from slave
#endif

#if defined(I2C_OTHER_FRAME)
    if (HAL_I2C_Master_Seq_Transmit_IT(&(obj->handle), dev_address, data, size, XferOptions) == HAL_OK) {
#else
    if (HAL_I2C_Master_Transmit_IT(&(obj->handle), dev_address, data, size) == HAL_OK) {
#endif
      // wait for transfer completion
      while ((HAL_I2C_GetState(&(obj->handle)) != HAL_I2C_STATE_READY) && (delta < I2C_TIMEOUT_TICK)) {
        delta = (HAL_GetTick() - tickstart);
        if (HAL_I2C_GetError(&(obj->handle)) != HAL_I2C_ERROR_NONE) {
          break;
        }
      }

      err = HAL_I2C_GetError(&(obj->handle));
      if ((delta >= I2C_TIMEOUT_TICK)
          || ((err & HAL_I2C_ERROR_TIMEOUT) == HAL_I2C_ERROR_TIMEOUT)) {
        ret = I2C_TIMEOUT;
      } else {
        if ((err & HAL_I2C_ERROR_AF) == HAL_I2C_ERROR_AF) {
          ret = I2C_NACK_DATA;
        } else if (err != HAL_I2C_ERROR_NONE) {
          ret = I2C_ERROR;
        }
      }
    }
  }
  return ret;
}

/**
  * @brief  Write bytes to master
  * @param  obj : pointer to i2c_t structure
  * @param  data: pointer to data to be write
  * @param  size: number of bytes to be write.
  * @retval status
  */
template<typename I2C_TYPE>
i2c_status_e i2c_slave_write_IT(i2c_t<I2C_TYPE> *obj, uint8_t *data, uint16_t size) {
  uint8_t i = 0;
  i2c_status_e ret = I2C_OK;

  // Protection to not override the TxBuffer
  if (size > I2C_TXRX_BUFFER_SIZE) {
    ret = I2C_DATA_TOO_LONG;
  } else {
    // Check the communication status
    for (i = 0; i < size; i++) {
      obj->i2cTxRxBuffer[obj->i2cTxRxBufferSize + i] = *(data + i);
    }

    obj->i2cTxRxBufferSize += size;
  }
  return ret;
}

template<typename I2C_TYPE>
i2c_status_e i2c_master_read(i2c_t<I2C_TYPE> *obj, uint8_t dev_address, uint8_t *data, uint16_t size)
{
  i2c_status_e ret = I2C_OK;
  uint32_t tickstart = HAL_GetTick();
  uint32_t delta = 0;
  uint32_t err = 0;

#if defined(I2C_OTHER_FRAME)
  uint32_t XferOptions = obj->handle.XferOptions; // save XferOptions value, because handle can be modified by HAL, which cause issue in case of NACK from slave
#endif

#if defined(I2C_OTHER_FRAME)
  if (HAL_I2C_Master_Seq_Receive_IT(&(obj->handle), dev_address, data, size, XferOptions) == HAL_OK) {
#else
  if (HAL_I2C_Master_Receive_IT(&(obj->handle), dev_address, data, size) == HAL_OK) {
#endif
    // wait for transfer completion
    while ((HAL_I2C_GetState(&(obj->handle)) != HAL_I2C_STATE_READY) && (delta < I2C_TIMEOUT_TICK)) {
      delta = (HAL_GetTick() - tickstart);
      if (HAL_I2C_GetError(&(obj->handle)) != HAL_I2C_ERROR_NONE) {
        break;
      }
    }

    err = HAL_I2C_GetError(&(obj->handle));
    if ((delta >= I2C_TIMEOUT_TICK)
        || ((err & HAL_I2C_ERROR_TIMEOUT) == HAL_I2C_ERROR_TIMEOUT)) {
      ret = I2C_TIMEOUT;
    } else {
      if ((err & HAL_I2C_ERROR_AF) == HAL_I2C_ERROR_AF) {
        ret = I2C_NACK_DATA;
      } else if (err != HAL_I2C_ERROR_NONE) {
        ret = I2C_ERROR;
      }
    }
  }
  return ret;
}

/**
  * @brief  Checks if target device is ready for communication
  * @param  obj : pointer to i2c_t structure
  * @param  devAddr: specifies the address of the device.
  * @param  trials : Number of trials.
  * @retval status
  */
template<typename I2C_TYPE>
i2c_status_e i2c_IsDeviceReady(i2c_t<I2C_TYPE> *obj, uint8_t devAddr, uint32_t trials) {
  i2c_status_e ret = I2C_OK;

  switch (HAL_I2C_IsDeviceReady(&(obj->handle), devAddr, trials, I2C_TIMEOUT_TICK)) {
    case HAL_OK:
      ret = I2C_OK;
      break;
    case HAL_TIMEOUT:
      ret = (obj->handle.State != HAL_I2C_STATE_READY) ? I2C_TIMEOUT : I2C_NACK_ADDR;
      break;
    case HAL_BUSY:
      ret = (obj->handle.State != HAL_I2C_STATE_READY) ? I2C_BUSY : I2C_NACK_ADDR;
      break;
    default:
      ret = (obj->handle.State != HAL_I2C_STATE_READY) ? I2C_ERROR : I2C_NACK_ADDR;
      break;
  }
  return ret;
}

/** @brief  sets function called before a slave read operation
  * @param  obj : pointer to i2c_t structure
  * @param  function: callback function to use
  * @retval None
  */
template<typename I2C_TYPE>
void i2c_attachSlaveRxEvent(i2c_t<I2C_TYPE> *obj, void (*function)(i2c_t<I2C_TYPE> *)) {
  if ((obj != NULL) && (function != NULL)) {
    obj->i2c_onSlaveReceive = function;
    HAL_I2C_EnableListen_IT(&(obj->handle));
  }
}

/** @brief  sets function called before a slave write operation
  * @param  obj : pointer to i2c_t structure
  * @param  function: callback function to use
  * @retval None
  */
template<typename I2C_TYPE>
void i2c_attachSlaveTxEvent(i2c_t<I2C_TYPE> *obj, void (*function)(i2c_t<I2C_TYPE> *)) {
  if ((obj != NULL) && (function != NULL)) {
    obj->i2c_onSlaveTransmit = function;
    HAL_I2C_EnableListen_IT(&(obj->handle));
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

#endif /* PERIPH_LIB_TWI_H */
