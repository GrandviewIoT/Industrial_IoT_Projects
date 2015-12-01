
// WVD - See exposure in Vendor Code notes:  ??? !!!

/**
  ******************************************************************************
  * @file    stm32_bluenrg_ble.c
  * @author  CL
  * @version V1.0.0
  * @date    04-July-2014
  * @brief
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
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

/* Includes ------------------------------------------------------------------*/
#include "user_api.h"                 // MCU and Board specific parms/pins/etc

#include "stm32_bluenrg_ble.h"
#include "gp_timer.h"
#include "debug.h"

#define HEADER_SIZE         5
#define MAX_BUFFER_SIZE   255
#define TIMEOUT_DURATION   15

void  BlueNRG_GPIO_Init (SPI_HandleTypeDef* hspi);

    SPI_HandleTypeDef  SpiHandle;    // NOTE:  ST's hci routines depend upon
                                     //        this hardcoded  SpiHandle  label.
                                     //        They have direct externs to it.


/* Private function prototypes ---------------*/
static void us150Delay(void);
void set_irq_as_output(void);
void set_irq_as_input(void);
void  Hal_Write_Serial (const void* data1, const void* data2,
                        int32_t n_bytes1, int32_t n_bytes2);


#if NO_LONGER_CALLED
/**************************************************************************
*                           BlueNRG_GPIO_Init
*
* @brief  This function is used for low level initialization of the SPI
*         communication support with the BlueNRG Expansion Board.
*
* @param  Pointer to the handle of the STM32Cube HAL SPI interface.
* @retval None
*************************************************************************/
void  BlueNRG_GPIO_Init (SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef  GPIO_InitStruct;


//  09/13/15        ---- NO ONE CALLS THIS LOGIC ANYMORE !!! ----
//                   is now handled in mnet_connect_network()

         //---------------------------------------------------
         // Configure GPIOs for control of BLE NRG module
         //---------------------------------------------------

         //---------------------------------------------------
         // Configure RESET pin:  PA8 - Arduino D7 on Nucleo.
         //---------------------------------------------------
    pin_Config (BLE_BLUENRG_RESET, GPIO_OUTPUT, PIN_USE_PULLUP);
    pin_Low (BLE_BLUENRG_RESET);    // and set it LOW, to prevent spurius interrupts during config

         //-------------------------------------------------------------------
         // Configure SPI CS pin:  PA1 - Arduino A1 on Nucleo.
         //
         // Rest of SPI pins will be auto-configured on spi_Init call (below)
         //-------------------------------------------------------------------
    pin_Config (BLE_BLUENRG_SPI_CS, GPIO_OUTPUT, PIN_USE_PULLUP);
    pin_High (BLE_BLUENRG_SPI_CS);  // and set it HIGH, to De-Assert CS during config

         //---------------------------------------------------
         // Configure IRQ pin:  PA0 - Arduino A0 on Nucleo
         //---------------------------------------------------
    pin_Config_IRQ_Pin (BLE_BLUENRG_IRQ_PIN, GPIO_RUPT_MODE_RISING, 0, BLE_BLUENRG_IRQn, 4);

         // and setup associated NVIC EXTI entries
    HAL_NVIC_SetPriority (BLE_BLUENRG_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ (BLE_BLUENRG_IRQn);


#if WVD_SHUTOFF                              // 06/20/15 - debug L0 issues
  if (hspi->Instance == BNRG_SPI_INSTANCE)   // 0620/15 - L0 is skipping this ==> wrong instance ???
    {
        /* Enable GPIO Ports Clock */
    BNRG_SPI_RESET_CLK_ENABLE();
    BNRG_SPI_SCLK_CLK_ENABLE();
    BNRG_SPI_MISO_CLK_ENABLE();
    BNRG_SPI_MOSI_CLK_ENABLE();
    BNRG_SPI_CS_CLK_ENABLE();
    BNRG_SPI_IRQ_CLK_ENABLE();

        /* Enable SPI clock */
    BNRG_SPI_CLK_ENABLE();

        /* Reset */
    GPIO_InitStruct.Pin       = BNRG_SPI_RESET_PIN;
    GPIO_InitStruct.Mode      = BNRG_SPI_RESET_MODE;
    GPIO_InitStruct.Pull      = BNRG_SPI_RESET_PULL;
    GPIO_InitStruct.Speed     = BNRG_SPI_RESET_SPEED;
    GPIO_InitStruct.Alternate = BNRG_SPI_RESET_ALTERNATE;
    HAL_GPIO_Init (BNRG_SPI_RESET_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin (BNRG_SPI_RESET_PORT, BNRG_SPI_RESET_PIN, GPIO_PIN_RESET); // force RESET
	/* Added to avoid spurious interrupt from the BlueNRG */
#endif

        /* SCLK */
    GPIO_InitStruct.Pin       = BNRG_SPI_SCLK_PIN;
    GPIO_InitStruct.Mode      = BNRG_SPI_SCLK_MODE;
    GPIO_InitStruct.Pull      = BNRG_SPI_SCLK_PULL;
    GPIO_InitStruct.Speed     = BNRG_SPI_SCLK_SPEED;
    GPIO_InitStruct.Alternate = BNRG_SPI_SCLK_ALTERNATE;
    HAL_GPIO_Init(BNRG_SPI_SCLK_PORT, &GPIO_InitStruct);

        /* MISO */
    GPIO_InitStruct.Pin       = BNRG_SPI_MISO_PIN;
    GPIO_InitStruct.Mode      = BNRG_SPI_MISO_MODE;
    GPIO_InitStruct.Pull      = BNRG_SPI_MISO_PULL;
    GPIO_InitStruct.Speed     = BNRG_SPI_MISO_SPEED;
    GPIO_InitStruct.Alternate = BNRG_SPI_MISO_ALTERNATE;
    HAL_GPIO_Init(BNRG_SPI_MISO_PORT, &GPIO_InitStruct);

        /* MOSI */
    GPIO_InitStruct.Pin       = BNRG_SPI_MOSI_PIN;
    GPIO_InitStruct.Mode      = BNRG_SPI_MOSI_MODE;
    GPIO_InitStruct.Pull      = BNRG_SPI_MOSI_PULL;
    GPIO_InitStruct.Speed     = BNRG_SPI_MOSI_SPEED;
    GPIO_InitStruct.Alternate = BNRG_SPI_MOSI_ALTERNATE;
    HAL_GPIO_Init(BNRG_SPI_MOSI_PORT, &GPIO_InitStruct);

#if WVD_SHUTOFF                              // 06/20/15 - debug L0 issues
        /* NSS/CSN/CS */
    GPIO_InitStruct.Pin       = BNRG_SPI_CS_PIN;
    GPIO_InitStruct.Mode      = BNRG_SPI_CS_MODE;
    GPIO_InitStruct.Pull      = BNRG_SPI_CS_PULL;
    GPIO_InitStruct.Speed     = BNRG_SPI_CS_SPEED;
    GPIO_InitStruct.Alternate = BNRG_SPI_CS_ALTERNATE;
    HAL_GPIO_Init (BNRG_SPI_CS_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin (BNRG_SPI_CS_PORT, BNRG_SPI_CS_PIN, GPIO_PIN_SET);  // De-Assert BLE CS

        /* IRQ -- INPUT */
    GPIO_InitStruct.Pin       = BNRG_SPI_IRQ_PIN;
    GPIO_InitStruct.Mode      = BNRG_SPI_IRQ_MODE;
    GPIO_InitStruct.Pull      = BNRG_SPI_IRQ_PULL;
    GPIO_InitStruct.Speed     = BNRG_SPI_IRQ_SPEED;
#if defined(STM32F103xB)
#else
    GPIO_InitStruct.Alternate = BNRG_SPI_IRQ_ALTERNATE;
#endif
    HAL_GPIO_Init (BNRG_SPI_IRQ_PORT, &GPIO_InitStruct);

 #if L0_TESTING_MOVED     // 06/20/15
        /* Configure the NVIC for SPI */
    HAL_NVIC_SetPriority (BNRG_SPI_EXTI_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ (BNRG_SPI_EXTI_IRQn);
 #endif
   }
#endif                              // WVD_SHUTOFF

}
#endif                              // NO_LONGER_CALLED


#if NO_LONGER_CALLED
/************************************************************************
* @brief  Initialize the SPI communication support for the BlueNRG
*         Expansion Board.
*
* @param  None
* @retval None
*************************************************************************/
void  BNRG_SPI_Init (void)
{

//  09/13/15        ---- NO ONE CALLS THIS LOGIC ANYMORE !!! ----
//                   is now handled in mnet_connect_network()

  SpiHandle.Instance               = BNRG_SPI_INSTANCE;
  SpiHandle.Init.Mode              = BNRG_SPI_MODE;
  SpiHandle.Init.Direction         = BNRG_SPI_DIRECTION;
  SpiHandle.Init.DataSize          = BNRG_SPI_DATASIZE;
  SpiHandle.Init.CLKPolarity       = BNRG_SPI_CLKPOLARITY;
  SpiHandle.Init.CLKPhase          = BNRG_SPI_CLKPHASE;
  SpiHandle.Init.NSS               = BNRG_SPI_NSS;
  SpiHandle.Init.FirstBit          = BNRG_SPI_FIRSTBIT;
  SpiHandle.Init.TIMode            = BNRG_SPI_TIMODE;
  SpiHandle.Init.CRCPolynomial     = BNRG_SPI_CRCPOLYNOMIAL;
  SpiHandle.Init.BaudRatePrescaler = BNRG_SPI_BAUDRATEPRESCALER;
  SpiHandle.Init.CRCCalculation    = BNRG_SPI_CRCCALCULATION;

  HAL_SPI_Init (&SpiHandle);
}
#endif                              // NO_LONGER_CALLED


/*********************************************************************
* @brief  Reset the BlueNRG device.
*
* @param  None
* @retval None       caller:  mnet_connect_network()
**********************************************************************/
void  BlueNRG_RST (void)
{
       // THUIS LOGIC IS STILL CALLED - 09/13/15
////HAL_GPIO_WritePin (BNRG_SPI_RESET_PORT, BNRG_SPI_RESET_PIN, GPIO_PIN_RESET);
    pin_Low (BLE_BLUENRG_RESET);
    sys_Delay_Millis (5);

////HAL_GPIO_WritePin (BNRG_SPI_RESET_PORT, BNRG_SPI_RESET_PIN, GPIO_PIN_SET);
    pin_High(BLE_BLUENRG_RESET);
    sys_Delay_Millis (5);
}


#if NEVER_CALLED
/*************************************************************************
* @brief  Activate internal bootloader using GPIO RESET pin.
*
* @param  None
* @retval None
************************************************************************/
void  BlueNRG_HW_Bootloader (void)
{
    set_irq_as_output();

    BlueNRG_RST();

    set_irq_as_input();
}
#endif            // not seeing any place in code where this is called


/************************************************************************
* @brief  Reports if the BlueNRG has data for the host micro.
*
* @param  None
* @retval 1 if data are present, 0 otherwise
*
* Called by:  HCI_Isr(),  HCI_Process() in hci.c
************************************************************************/
// FIXME: find a better way to handle this return value (bool type? TRUE and FALSE)
uint8_t  BlueNRG_DataPresent (void)
{
////if (HAL_GPIO_ReadPin(BNRG_SPI_EXTI_PORT, BNRG_SPI_EXTI_PIN) == GPIO_PIN_SET)
    if (pin_Read(BLE_BLUENRG_IRQ_PIN))
       return 1;
       else return 0;
}                              /* end BlueNRG_DataPresent() */


/*************************************************************************
* @brief  Reads from BlueNRG SPI buffer and stores data into local buffer.
*
* @param  hspi     : Handle of the STM32Cube HAL SPI interface
* @param  buffer   : Buffer where data from SPI are stored
* @param  buff_size: Buffer size
* @retval int32_t  : Number of read bytes
*
* Called by:  HCI_Isr(),  HCI_Process() in hci.c
*             HAL_GPIO_EXTI_Callback()  in bluenrg_interface.c
*
* _All_ those routines call this using a hardcoded pointer to &SpiHandle.
*  caller: HAL_GPIO_EXTI_Callback
*************************************************************************/
int32_t  BlueNRG_SPI_Read_All (SPI_HandleTypeDef *hspi, uint8_t *buffer,
                               uint8_t buff_size)
{
  uint16_t          byte_count;
  uint8_t           len = 0;
  uint8_t           char_ff = 0xff;
  volatile uint8_t  read_char;

  uint8_t   header_master [HEADER_SIZE] = { 0x0b, 0x00, 0x00, 0x00, 0x00 };
  uint8_t   header_slave [HEADER_SIZE];

     /* ASSERT CS */
////HAL_GPIO_WritePin (BNRG_SPI_CS_PORT, BNRG_SPI_CS_PIN, GPIO_PIN_RESET);
    ASSERT_CS_BlueNRG();           // Ensure CS is set to ACTIVE

     /* Read the header */
////HAL_SPI_TransmitReceive (hspi, header_master, header_slave, HEADER_SIZE, TIMEOUT_DURATION);
    spi_Write_Read (BLE_BLUENRG_SPI_ID, header_master, header_slave,
                    HEADER_SIZE, 0);

  if (header_slave[0] == 0x02)          // F4_46 keeps returning 0x03, 0x7D, 0x00, 0x07, 0x00 in header_slave
     {
           /* device is ready to send back data it has received */
      byte_count = (header_slave[4]<<8) | header_slave[3];

      if (byte_count > 0)
        {
             /* avoid reading more data than total size of the buffer */
         if (byte_count > buff_size)
            {
              byte_count = buff_size;
            }

         for (len = 0; len < byte_count; len++)
           {
////         HAL_SPI_TransmitReceive (hspi, &char_ff, (uint8_t*) &read_char,
////                                  1, TIMEOUT_DURATION);
             spi_Write_Read (BLE_BLUENRG_SPI_ID,
                             &char_ff, (uint8_t*) &read_char,
                             1, 0);

             buffer[len] = read_char;
           }
       }
    }

      /* DEASSERT CS line */
////HAL_GPIO_WritePin (BNRG_SPI_CS_PORT, BNRG_SPI_CS_PIN, GPIO_PIN_SET);
    DEASSERT_CS_BlueNRG();                     // Ensure CS is set to NOT active

// VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
// WVD  ??? !!!  CROSS_PLATFORM EXPOSURE - esp for FASTER DEVICES  !!! ???
// VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
     //----------------------------------------------------------------------
     // Add a small delay to give time to the BlueNRG to set the IRQ pin low
     // to avoid a useless SPI read at the end of the transaction
     //----------------------------------------------------------------------
  for (volatile int i = 0; i < 2; i++)
    __NOP();

#ifdef PRINT_CSV_FORMAT
  if (len > 0)
   {
    PRINT_CSV ("00:00:00.000");
    for (int i=0; i<len; i++)
        {
          PRINT_CSV (" %02x", buffer[i]);
        }
    PRINT_CSV ("\n");
  }
#endif

  return len;
}


/************************************************************************
* @brief  Write HCI data to a SPI serial interface.
*
* @param  data1   :  1st buffer
* @param  data2   :  2nd buffer
* @param  n_bytes1: number of bytes in 1st buffer
* @param  n_bytes2: number of bytes in 2nd buffer
* @retval None
*
* Called by:  HCI_Isr()  routine in hci.c
************************************************************************/
void  Hal_Write_Serial (const void* data1, const void* data2,
                        int32_t n_bytes1, int32_t n_bytes2)
{
  struct timer   t;
  int            rc;

  Timer_Set (&t, CLOCK_SECOND/10);

#ifdef PRINT_CSV_FORMAT
  PRINT_CSV ("00:00:00.000");
  for (int i=0; i<n_bytes1; i++)
     {
       PRINT_CSV (" %02x", ((uint8_t *)data1)[i]);
     }
  for (int i=0; i<n_bytes2; i++)
     {
       PRINT_CSV (" %02x", ((uint8_t *)data2)[i]);
     }
  PRINT_CSV ("\n");
#endif

  while (1)
    {
      rc = BlueNRG_SPI_Write (&SpiHandle, (uint8_t*) data1, (uint8_t*) data2,
                              n_bytes1, n_bytes2);
      if (rc == 0)
         break;
      if (Timer_Expired(&t))
         {
            break;
         }
    }
}


/*******************************************************************************
* @brief  Writes data from local buffer to SPI.
*
* @param  hspi     : Handle of the STM32Cube HAL SPI interface
* @param  data1    : First data buffer to be written
* @param  data2    : Second data buffer to be written
* @param  Nb_bytes1: Size of first data buffer to be written
* @param  Nb_bytes2: Size of second data buffer to be written
* @retval Number of read bytes
*
* Called by:  Hal_Write_Serial
*
*
* _All_ those routines call this using a hardcoded pointer to &SpiHandle.
* CALLERs:  aci_write_hal_config -> hci_send_cmd -> hci_write -> Hal_Write_Serial()
*           aci_gatt_init  -> hci_send_cmd -> hci_write -> Hal_Write_Serial()
*           aci_gap_init  -> hci_send_cmd -> hci_write -> Hal_Write_Serial()
*******************************************************************************/
int32_t  BlueNRG_SPI_Write (SPI_HandleTypeDef *hspi, uint8_t* data1,
                            uint8_t* data2, uint8_t Nb_bytes1, uint8_t Nb_bytes2)
{
    int32_t  result = 0;

    int32_t  spi_fix_enabled = 0;

#ifdef ENABLE_SPI_FIX
    spi_fix_enabled = 1;
#endif //ENABLE_SPI_FIX

  unsigned char  header_master[HEADER_SIZE] = {0x0a, 0x00, 0x00, 0x00, 0x00};
  unsigned char  header_slave[HEADER_SIZE]  = {0xaa, 0x00, 0x00, 0x00, 0x00};

  unsigned char  read_char_buf [MAX_BUFFER_SIZE];

    Disable_SPI_IRQ();

      /*
      ** If the SPI_FIX is enabled, the IRQ is set in Output mode, then it is pulled
      ** high and, after a delay of at least 112us, the CS line is asserted and the
      ** header transmit/receive operations are started.
      ** After these transmit/receive operations the IRQ is reset in input mode.
      */
  if (spi_fix_enabled)
     {
       set_irq_as_output();

// VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
// WVD  ??? !!!  CROSS_PLATFORM EXPOSURE - esp for FASTER DEVICES  !!! ???
// VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
             /* Wait to Assert CS line until after at least 112us */
       us150Delay();
     }

       /* ASSERT CS line */
////HAL_GPIO_WritePin (BNRG_SPI_CS_PORT, BNRG_SPI_CS_PIN, GPIO_PIN_RESET);
    ASSERT_CS_BlueNRG();           // Ensure CS is set to ACTIVE

      /* Exchange header with BlueNRG */
////HAL_SPI_TransmitReceive (hspi, header_master, header_slave, HEADER_SIZE, TIMEOUT_DURATION); // WORKS
    spi_Write_Read (BLE_BLUENRG_SPI_ID, header_master, header_slave,
                    HEADER_SIZE, 0);

  if (spi_fix_enabled)
     {
       set_irq_as_input();
     }

  if (header_slave[0] == 0x02)
     {
             /* SPI is ready */
       if (header_slave[1] >= (Nb_bytes1+Nb_bytes2))
          {
                  /*  ensure Buffer is big enough */
           if (Nb_bytes1 > 0)
              {
////            HAL_SPI_TransmitReceive (hspi, data1, read_char_buf, Nb_bytes1, TIMEOUT_DURATION);
                spi_Write_Read (BLE_BLUENRG_SPI_ID,
                                data1, read_char_buf,
                                Nb_bytes1, 0);

              }
           if (Nb_bytes2 > 0)
              {
////            HAL_SPI_TransmitReceive (hspi, data2, read_char_buf, Nb_bytes2, TIMEOUT_DURATION);
                spi_Write_Read (BLE_BLUENRG_SPI_ID,
                                data2, read_char_buf,
                                Nb_bytes2, 0);
              }
     }
    else {
              /* Buffer is too small */
           result = -2;
         }
    }
   else {
              /* SPI is not ready */
            result = -1;
        }

              /* Release CS line */
////HAL_GPIO_WritePin (BNRG_SPI_CS_PORT, BNRG_SPI_CS_PIN, GPIO_PIN_SET);
    DEASSERT_CS_BlueNRG();                     // Ensure CS is set to NOT active

    Enable_SPI_IRQ();

    return result;
}


/************************************************************************
* @brief  Set the IRQ line into Output mode, for special sequences.
*
* @param  None
* @retval None
************************************************************************/
void  set_irq_as_output()
{

#if ORIGINAL_LOGIC
  GPIO_InitTypeDef  GPIO_InitStructure;

      /* Pull IRQ high */
  GPIO_InitStructure.Pin   = BNRG_SPI_IRQ_PIN;
  GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Speed = BNRG_SPI_IRQ_SPEED;
  GPIO_InitStructure.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init (BNRG_SPI_IRQ_PORT, &GPIO_InitStructure);

  HAL_GPIO_WritePin (BNRG_SPI_IRQ_PORT, BNRG_SPI_IRQ_PIN, GPIO_PIN_SET);
#endif

        // Convert IRQ from input to output, then set it high.
    pin_Config (BLE_BLUENRG_IRQ_PIN, GPIO_OUTPUT, 0);
    pin_High (BLE_BLUENRG_IRQ_PIN);
}


/************************************************************************
* @brief  Reset the IRQ back into input mode.
*
* @param  None
* @retval None
************************************************************************/
void  set_irq_as_input ()
{

#if ORIGINAL_LOGIC
  GPIO_InitTypeDef  GPIO_InitStructure;

      /* IRQ input */
  GPIO_InitStructure.Pin       = BNRG_SPI_IRQ_PIN;
  GPIO_InitStructure.Mode      = BNRG_SPI_IRQ_MODE;
#if defined(STM32F103xB)
#else
  GPIO_InitStructure.Alternate = BNRG_SPI_IRQ_ALTERNATE;
#endif
  GPIO_InitStructure.Pull      = GPIO_PULLDOWN;
  GPIO_InitStructure.Speed     = BNRG_SPI_IRQ_SPEED;
  HAL_GPIO_Init (BNRG_SPI_IRQ_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Pull      = BNRG_SPI_IRQ_PULL;
  HAL_GPIO_Init (BNRG_SPI_IRQ_PORT, &GPIO_InitStructure);
#endif
         //---------------------------------------------------
         // Put IRQ pin into EXTI input mode, rising edge.
         // Note: we do NOT re-initialize the NVIC parms.
         //
         // re-Configure IRQ pin:  PA0 - Arduino A0 on Nucleo
         //---------------------------------------------------
    pin_Config (BLE_BLUENRG_IRQ_PIN, GPIO_INPUT, GPIO_RUPT_MODE_RISING);
}


/************************************************************************
* @brief  Utility function for delay
*
* @param  None
* @retval None
* NOTE: TODO: implement with clock-independent function.
************************************************************************/
static void   us150Delay()
{

// VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
// WVD  ??? !!!  CROSS_PLATFORM EXPOSURE - esp for FASTER DEVICES  !!! ???
// VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV


#if SYSCLK_FREQ == 4000000
      for (volatile int i = 0; i < 35; i++)
         __NOP();
#elif SYSCLK_FREQ == 32000000
      for (volatile int i = 0; i < 420; i++)           // L0  (original ST code)
         __NOP();
#elif SYSCLK_FREQ == 48000000                          // WVD ADD for F0
      for (volatile int i = 0; i < 642; i++)           // WVD ADD for F0
         __NOP();
#elif SYSCLK_FREQ == 72000000                          // WVD ADD for F3_34
      for (volatile int i = 0; i < 964; i++)           // WVD ADD for F3_34
         __NOP();
#elif SYSCLK_FREQ == 84000000
      for (volatile int i = 0; i < 1125; i++)          // F4  (original ST code)
         __NOP();
#else
#error Implement delay function.
#endif
}


/************************************************************************
* @brief  Enable SPI IRQ.
*
* @param  None
* @retval None
************************************************************************/
void  Enable_SPI_IRQ (void)
{
    HAL_NVIC_EnableIRQ (BNRG_SPI_EXTI_IRQn);
}


/************************************************************************
* @brief  Disable SPI IRQ.
*
* @param  None
* @retval None
*************************************************************************/
void  Disable_SPI_IRQ (void)
{
    HAL_NVIC_DisableIRQ (BNRG_SPI_EXTI_IRQn);
}


/************************************************************************
* @brief  Clear Pending SPI IRQ.
*
* @param  None
* @retval None
************************************************************************/
void  Clear_SPI_IRQ (void)
{
    HAL_NVIC_ClearPendingIRQ (BNRG_SPI_EXTI_IRQn);
}


/************************************************************************
* @brief  Clear EXTI (External Interrupt) line for SPI IRQ.
*
* @param  None
* @retval None
************************************************************************/
void  Clear_SPI_EXTI_Flag (void)
{
    __HAL_GPIO_EXTI_CLEAR_IT (BNRG_SPI_EXTI_PIN);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
