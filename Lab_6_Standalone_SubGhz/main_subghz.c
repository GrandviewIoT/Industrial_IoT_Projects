// In future, convert this over to send a real message from rest of Sensors
// rather than just lighting and LED.


                            // CLIENT  vs  SERVER
// is specified in spirit1_appli.h

/******************************************************************************
*                                                                 STM32 ONLY
*                         main_subghz.c
*
*
* LoRa example of remote sensor access via 915 MHz comm. 
*
*
* Supported/Tested Boards
* -----------------------
*      ST's Spirit1 at 915 MHz
*
*
*  History:
*    09/12/15 - Created for Industrial IoT OpenSource project.  Duquaine
* -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -
*
* The MIT License (MIT)
*
* Copyright (c) 2014-2015 Wayne Duquaine / Grandview Systems
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
******************************************************************************/

#include "user_api.h"              // pull in high level User API defs
//#include "cube_hal.h"
#include "radio_shield_config.h"
#include "spirit1_appli.h"

#define  TX_BUFFER_SIZE   20
#define  RX_BUFFER_SIZE   96


  uint8_t  TxLength = TX_BUFFER_SIZE;
  uint8_t  RxLength = 0;
  uint8_t  aTransmitBuffer[TX_BUFFER_SIZE] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,\
                                                                16,17,18,19,20};
  uint8_t  aReceiveBuffer[RX_BUFFER_SIZE]  = {0x00};


#if defined(LPM_ENABLE)
void SystemPower_Config(void);
#endif
int  main (void);


/*******************************************************************************
*                              Main program.
*******************************************************************************/
int  main (void)
{
       // Reset all peripherals, Initialize Flash interface and the Systick.
    sys_Init (0, 0);              // start up system with default CPU clock rate

    HAL_EnableDBGStopMode();

    pin_Config (LED1, GPIO_OUTPUT, 0);         // Init D13 LED on Nucleo board

       //-------------------------------------------------------------
       // Initialize LEDs on the SubGHz comm module
       //-------------------------------------------------------------
    RadioShieldLedInit (RADIO_SHIELD_LED);

       //-------------------------------------------------------------
       // Initialize SubGHz GPIOs, SPI, ...
       //-------------------------------------------------------------
    HAL_Spirit1_Init();

       //-------------------------------------------------------------
       // Initialize P2P (point-to-point) mode on SubGHz comm module
       //-------------------------------------------------------------
    P2P_Init();

/// BSP_PB_Init (BUTTON_KEY, BUTTON_MODE_EXTI);   // Initialize PushButtons
    BSP_PB_Init (BUTTON_USER, BUTTON_MODE_EXTI);   // Initialize PushButtons

#if defined(LPM_ENABLE)
    SystemPower_Config();          // Configure the system Power for LPW
    Enter_LP_mode();
#endif

    while (1)
     {         //----------------------------------------------------------
               // Invoke Data communication MAIN Polling Loop.
               // The routine checks if PushButton was pressed, and if so,
               // sends a message to remote node.
               // Otherwise, it hangs out a Receive to check if a message
               // was received from the other node. If so, it reads
               // the message, and sends a reply/ack.
               //
               // The Data_Comm_On() routine is in spirit1_appli.c
               //----------------------------------------------------------
       Data_Comm_On (aTransmitBuffer, TxLength, aReceiveBuffer, RxLength);
     }
}



#if defined(LPM_ENABLE)

/*************************************************************************
  * @brief  System Power Configuration
  *         The system Power is configured as follow :
  *            + Regulator in LP mode
  *            + VREFINT OFF, with fast wakeup enabled
  *            + HSI as SysClk after Wake Up
  *            + No IWDG
  *            + Wakeup using EXTI Line (Key Button PC.13)
  * @param  None
  * @retval None
  */
void  SystemPower_Config (void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

       /* Enable Ultra LOW POWER mode */
  HAL_PWREx_EnableUltraLowPower();

       /* Enable the fast wake up from Ultra LOW POWER mode */
  HAL_PWREx_EnableFastWakeUp();

       /* Select HSI as system clock source after Wake Up from LOW POWER Stop mode */
  __HAL_RCC_WAKEUPSTOP_CLK_CONFIG (RCC_StopWakeUpClock_HSI);

  __GPIOA_CLK_ENABLE();     // Enable needed GPIO clocks
  __GPIOB_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();

      /* Configure GPIO Pins: PA0,PA1,PA2,PA3,PA4,PA5,PA8,PA9,PA11,PA12,PA13,\
      ** PA14,PA15 in Analog Input mode (floating input trigger OFF) */
      /* PA6: SDO, PA7: SDI, PA10 :SDn  are not put in analog mode  */
  GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 \
         | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_11 \
         | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init (GPIOA, &GPIO_InitStructure);

      /* Configure GPIO Pins: PB0,PB1,PB2,PB4,PB5,PB7,PB8,PB9,PB10,PB11,PB12,PB13,\
      ** PB14,PB15 in Analog Input mode (floating input trigger OFF) */
      /* PB6: CSn, PB3: SCLK are not put in analog mode  */

  GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_4 \
         | GPIO_PIN_5 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10  \
         | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init (GPIOB, &GPIO_InitStructure);

      /* Configure GPIO Pins: PC0,PC1,PC2,PC3,PC4,PC5,PC6,PC8,PC9,PC10,PC11,PC12,\
      ** PC13,PC14,PC15 in Analog Input mode (floating input trigger OFF) */
      /* PC7: GPIO3, PC13: Push Button are not put in analog mode  */
  GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 \
         | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_8 | GPIO_PIN_9 \
         | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init (GPIOC, &GPIO_InitStructure);

      /* Configure all GPIO port pins in Analog Input mode (floating input trigger OFF) */
  GPIO_InitStructure.Pin  = GPIO_PIN_All;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;

  HAL_GPIO_Init (GPIOD, &GPIO_InitStructure);
  HAL_GPIO_Init (GPIOH, &GPIO_InitStructure);

      /* Disable GPIOs clock */
  __GPIOA_CLK_DISABLE();
  __GPIOB_CLK_DISABLE();
  __GPIOC_CLK_DISABLE();
  __GPIOD_CLK_DISABLE();
  __GPIOH_CLK_DISABLE();

}
#endif

#ifdef USE_FULL_ASSERT

/**
* @brief Reports the name of the source file and the source line number
* where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

}

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
