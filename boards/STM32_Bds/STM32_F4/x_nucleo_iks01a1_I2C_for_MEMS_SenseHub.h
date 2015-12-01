/**
  ******************************************************************************
  * @file    x_nucleo_iks01a1.h
  * @author  CL
  * @version V1.2.0
  * @date    28-January-2015
  * @brief   This file contains definitions for the x_nucleo_iks01a1.c 
  *          board specific functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __X_NUCLEO_IKS01A1_H
#define __X_NUCLEO_IKS01A1_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#ifdef STM32F401xE
#include "stm32f4xx_hal.h"
#endif
#ifdef STM32L053xx
#include "stm32l0xx_hal.h"
#endif
#include "../Components/Common/hum_temp.h"
#include "../Components/Common/imu_6axes.h"
#include "../Components/Common/magneto.h"
#include "../Components/Common/pressure.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup X_NUCLEO_IKS01A1
  * @{
  */

/** @defgroup X_NUCLEO_IKS01A1_Exported_Types X_NUCLEO_IKS01A1_Exported_Types
  * @{
  */
/**
 * @brief  Axes raw structure definition
 */
typedef struct {
    int32_t AXIS_X;
    int32_t AXIS_Y;
    int32_t AXIS_Z;
} AxesRaw_TypeDef;

/**
  * @}
  */

/** @defgroup X_NUCLEO_IKS01A1_Exported_Defines X_NUCLEO_IKS01A1_Exported_Defines
  * @{
  */

/* I2C clock speed configuration (in Hz) */
#ifndef NUCLEO_I2C_SHIELDS_SPEED
    #define NUCLEO_I2C_SHIELDS_SPEED                         400000
#endif /* I2C_ONBOARD_SENSORS_SPEED */

/* I2C peripheral configuration defines (control interface of the audio codec) */
#define NUCLEO_I2C_SHIELDS                            I2C1
#define NUCLEO_I2C_SHIELDS_CLK_ENABLE()               __I2C1_CLK_ENABLE()
#define NUCLEO_I2C_SHIELDS_SCL_SDA_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()
#define NUCLEO_I2C_SHIELDS_SCL_SDA_AF                 GPIO_AF4_I2C1
#define NUCLEO_I2C_SHIELDS_SCL_SDA_GPIO_PORT          GPIOB
#define NUCLEO_I2C_SHIELDS_SCL_PIN                    GPIO_PIN_8
#define NUCLEO_I2C_SHIELDS_SDA_PIN                    GPIO_PIN_9

#define NUCLEO_I2C_SHIELDS_FORCE_RESET()              __I2C1_FORCE_RESET()
#define NUCLEO_I2C_SHIELDS_RELEASE_RESET()            __I2C1_RELEASE_RESET()

/* I2C interrupt requests */
#ifdef STM32F401xE
#define NUCLEO_I2C_SHIELDS_EV_IRQn                    I2C1_EV_IRQn
#endif
#ifdef STM32L053xx
#define NUCLEO_I2C_SHIELDS_EV_IRQn                    I2C1_IRQn    
#endif    
#define NUCLEO_I2C_SHIELDS_ER_IRQn                    I2C1_ER_IRQn

/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the SPI communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */
#define NUCLEO_I2C_SHIELDS_TIMEOUT_MAX    0x1000 /*<! The value of the maximal timeout for BUS waiting loops */



/* User can use this section to tailor USARTx/UARTx instance used and associated
   resources */
/* Definition for USARTx clock resources */
#define USARTx                           USART2
#define USARTx_CLK_ENABLE()              __USART2_CLK_ENABLE();
#define DMAx_CLK_ENABLE()                __DMA1_CLK_ENABLE()
#define USARTx_RX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __USART2_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __USART2_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_2
#define USARTx_TX_GPIO_PORT              GPIOA
#ifdef STM32F401xE
#define USARTx_TX_AF                     GPIO_AF7_USART2
#endif
#ifdef STM32L053xx
#define USARTx_TX_AF                     GPIO_AF4_USART2
#endif
#define USARTx_RX_PIN                    GPIO_PIN_3
#define USARTx_RX_GPIO_PORT              GPIOA
#ifdef STM32F401xE
#define USARTx_RX_AF                     GPIO_AF7_USART2
#endif
#ifdef STM32L053xx
#define USARTx_RX_AF                     GPIO_AF4_USART2
#endif

/* Definition for USARTx's DMA */
#define USARTx_TX_DMA_CHANNEL            DMA_CHANNEL_4
#define USARTx_TX_DMA_STREAM             DMA1_Stream6
#define USARTx_RX_DMA_CHANNEL            DMA_CHANNEL_4
#define USARTx_RX_DMA_STREAM             DMA1_Stream5



/* Definition for interrupt Pins */
#define HUM_TEMP_DRDY_GPIO_PORT           GPIOB
#define HUM_TEMP_DRDY_GPIO_CLK_ENABLE()   __GPIOB_CLK_ENABLE()
#define HUM_TEMP_DRDY_GPIO_CLK_DISABLE()  __GPIOB_CLK_DISABLE()
#define HUM_TEMP_DRDY_PIN                 GPIO_PIN_10
#ifdef STM32F401xE
#define HUM_TEMP_DRDY_EXTI_IRQn           EXTI15_10_IRQn
#endif
#ifdef STM32L053xx
#define HUM_TEMP_DRDY_EXTI_IRQn           EXTI4_15_IRQn
#endif

#define IMU_6AXES_INT1_GPIO_PORT           GPIOB
#define IMU_6AXES_INT1_GPIO_CLK_ENABLE()   __GPIOB_CLK_ENABLE()
#define IMU_6AXES_INT1_GPIO_CLK_DISABLE()  __GPIOB_CLK_DISABLE()
#define IMU_6AXES_INT1_PIN                 GPIO_PIN_5
#ifdef STM32F401xE
#define IMU_6AXES_INT1_EXTI_IRQn           EXTI9_5_IRQn
#endif
#ifdef STM32L053xx
#define IMU_6AXES_INT1_EXTI_IRQn           EXTI4_15_IRQn
#endif

#define MAGNETO_DRDY_GPIO_PORT           GPIOC
#define MAGNETO_DRDY_GPIO_CLK_ENABLE()   __GPIOC_CLK_ENABLE()
#define MAGNETO_DRDY_GPIO_CLK_DISABLE()  __GPIOC_CLK_DISABLE()
#define MAGNETO_DRDY_PIN                 GPIO_PIN_0
#ifdef STM32F401xE
#define MAGNETO_DRDY_EXTI_IRQn           EXTI0_IRQn
#endif
#ifdef STM32L053xx
#define MAGNETO_DRDY_EXTI_IRQn           EXTI0_1_IRQn
#endif

#define MAGNETO_INT1_GPIO_PORT           GPIOC
#define MAGNETO_INT1_GPIO_CLK_ENABLE()   __GPIOC_CLK_ENABLE()
#define MAGNETO_INT1_GPIO_CLK_DISABLE()  __GPIOC_CLK_DISABLE()
#define MAGNETO_INT1_PIN                 GPIO_PIN_1
#ifdef STM32F401xE
#define MAGNETO_INT1_EXTI_IRQn           EXTI1_IRQn
#endif
#ifdef STM32L053xx
#define MAGNETO_INT1_EXTI_IRQn           EXTI0_1_IRQn
#endif

#define PRESSURE_INT1_GPIO_PORT           GPIOB
#define PRESSURE_INT1_GPIO_CLK_ENABLE()   __GPIOB_CLK_ENABLE()
#define PRESSURE_INT1_GPIO_CLK_DISABLE()  __GPIOB_CLK_DISABLE()
#define PRESSURE_INT1_PIN                 GPIO_PIN_4
#ifdef STM32F401xE
#define PRESSURE_INT1_EXTI_IRQn           EXTI4_IRQn
#endif
#ifdef STM32L053xx
#define PRESSURE_INT1_EXTI_IRQn           EXTI4_15_IRQn
#endif

// ready for use
#define USER_INT_GPIO_PORT           GPIOA
#define USER_INT_GPIO_CLK_ENABLE()   __GPIOA_CLK_ENABLE()
#define USER_INT_GPIO_CLK_DISABLE()  __GPIOA_CLK_DISABLE()
#define USER_INT_PIN                 GPIO_PIN_10
#ifdef STM32F401xE
#define USER_INT_EXTI_IRQn           EXTI15_10_IRQn
#endif
#ifdef STM32L053xx
#define USER_INT_EXTI_IRQn           EXTI4_15_IRQn
#endif

// ready for use
#define MEMS_INT1_GPIO_PORT           GPIOA
#define MEMS_INT1_GPIO_CLK_ENABLE()   __GPIOA_CLK_ENABLE()
#define MEMS_INT1_GPIO_CLK_DISABLE()  __GPIOA_CLK_DISABLE()
#define MEMS_INT1_PIN                 GPIO_PIN_4
#ifdef STM32F401xE
#define MEMS_INT1_EXTI_IRQn           EXTI4_IRQn
#endif
#ifdef STM32L053xx
#define MEMS_INT1_EXTI_IRQn           EXTI4_15_IRQn
#endif

// ready for use
#define MEMS_INT2_GPIO_PORT           GPIOB
#define MEMS_INT2_GPIO_CLK_ENABLE()   __GPIOB_CLK_ENABLE()
#define MEMS_INT2_GPIO_CLK_DISABLE()  __GPIOB_CLK_DISABLE()
#define MEMS_INT2_PIN                 GPIO_PIN_0
#ifdef STM32F401xE
#define MEMS_INT2_EXTI_IRQn           EXTI0_IRQn
#endif
#ifdef STM32L053xx
#define MEMS_INT2_EXTI_IRQn           EXTI0_1_IRQn
#endif

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
  }
#endif

#endif /* __X_NUCLEO_IKS01A1_H */



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
