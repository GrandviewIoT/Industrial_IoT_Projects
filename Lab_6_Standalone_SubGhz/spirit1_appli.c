
// 09/11/15
// this should be easily extenible to a SDLC-like Master/Multi-point network,
// aka Hub and spokes
// Have one node tagged a master that all the downstream clients talk to.
// It acts as the hub to SAF requests upstream to WiFi / GSM network.
//     ==> JSON needs a client id, so that any requests can be tagged as they
//         go up, and any optional cmds going down can be roiuted down to client.

/******************************************************************************
* @file                               spirit1_appli.c
* @author  Central Labs
* @version V1.1.0
* @date    14-Aug-2014
* @brief   user file to configure Spirit1 transceiver.
*
@verbatim
===============================================================================
##### How to use this driver #####
===============================================================================
[..]
This file is generated automatically by STM32CubeMX and eventually modified
by the user

@endverbatim
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
******************************************************************************/

#include "user_api.h"

//#include "cube_hal.h"
#include "spirit1_appli.h"
#include "MCU_Interface.h"

    uint32_t  appli_send_buf_called = 0;      // DEBUG COUNTERS - App Calls
    uint32_t  appli_send_buf_completed = 0;
    uint32_t  appli_rcv_buf_called  = 0;
    uint32_t  appli_rcv_buf_timeout = 0;
    uint32_t  appli_rcv_buf_data    = 0;

    uint32_t  spirit_irq_count     = 0;       // DEBUG COUNTERS - I/O ISRs
    uint32_t  spirit_IO_WAIT_complete  = 0;
    uint32_t  spirit_process_IRQ_count = 0;
    uint32_t  spirit_got_TXdone    = 0;
    uint32_t  spirit_got_RXdone    = 0;
    uint32_t  spirit_got_RXdiscard = 0;


/******************************************************************************
*                     RadioDriver_t structure  fitting
*/
RadioDriver_t spirit_cb =
{
  .Init = Spirit1InterfaceInit,
  .GpioIrq       = Spirit1GpioIrqInit,
  .RadioInit      = Spirit1RadioInit,
  .SetRadioPower  = Spirit1SetPower,
  .PacketConfig   = Spirit1PacketConfig,
  .SetPayloadLen  = Spirit1SetPayloadlength,
  .SetDestinationAddress = Spirit1SetDestinationAddress,
  .EnableTxIrq    = Spirit1EnableTxIrq,
  .EnableRxIrq    = Spirit1EnableRxIrq,
  .DisableIrq     = Spirit1DisableIrq,
  .SetRxTimeout   = Spirit1SetRxTimeout,
  .EnableSQI      = Spirit1EnableSQI,
  .SetRssiThreshold = Spirit1SetRssiTH,
  .ClearIrqStatus = Spirit1ClearIRQ,
  .StartRx        = Spirit1StartRx,
  .StartTx        = Spirit1StartTx,
  .GetRxPacket    = Spirit1GetRxPacket
};

/******************************************************************************
*                      MCULowPowerMode_t structure fitting
*/
MCULowPowerMode_t MCU_LPM_cb =
{
  .McuStopMode    = MCU_Enter_StopMode,
  .McuStandbyMode = MCU_Enter_StandbyMode,
  .McuSleepMode   = MCU_Enter_SleepMode
};

/******************************************************************************
*                      RadioLowPowerMode_t structure fitting
*/
RadioLowPowerMode_t Radio_LPM_cb =
{
  .RadioShutDown = RadioPowerOFF,
  .RadioStandBy  = RadioStandBy,
  .RadioSleep    = RadioSleep,
  .RadioPowerON  = RadioPowerON
};

/******************************************************************************
*                      GPIO structure fitting
*/
SGpioInit xGpioIRQ = {
  SPIRIT_GPIO_3,
  SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP,
  SPIRIT_GPIO_DIG_OUT_IRQ
};

/******************************************************************************
*                      Radio structure fitting
*/
SRadioInit xRadioInit = {
  XTAL_OFFSET_PPM,
  BASE_FREQUENCY,
  CHANNEL_SPACE,
  CHANNEL_NUMBER,
  MODULATION_SELECT,
  DATARATE,
  FREQ_DEVIATION,
  BANDWIDTH
};


#if defined(USE_STack_PROTOCOL)
/******************************************************************************
*                      Packet Basic structure fitting           STACK  PROTOCOL
*/
PktStackInit xStackInit = {
  PREAMBLE_LENGTH,
  SYNC_LENGTH,
  SYNC_WORD,
  LENGTH_TYPE,
  LENGTH_WIDTH,
  CRC_MODE,
  CONTROL_LENGTH,
  EN_FEC,
  EN_WHITENING
};

/*****************************************************************************
*                      LLP structure fitting
*/
PktStackLlpInit xStackLLPInit = {
  EN_AUTOACK,
  EN_PIGGYBACKING,
  MAX_RETRANSMISSIONS
};

/******************************************************************************
*                      Address structure fitting
*/
PktStackAddressesInit xAddressInit = {
  EN_FILT_MY_ADDRESS,
  MY_ADDRESS,
  EN_FILT_MULTICAST_ADDRESS,
  MULTICAST_ADDRESS,
  EN_FILT_BROADCAST_ADDRESS,
  BROADCAST_ADDRESS
};

#elif defined(USE_BASIC_PROTOCOL)

/******************************************************************************
*                      Packet Basic structure fitting           BASIC  PROTOCOL
*/
PktBasicInit xBasicInit = {
  PREAMBLE_LENGTH,
  SYNC_LENGTH,
  SYNC_WORD,
  LENGTH_TYPE,
  LENGTH_WIDTH,
  CRC_MODE,
  CONTROL_LENGTH,
  EN_ADDRESS,
  EN_FEC,
  EN_WHITENING
};


/******************************************************************************
*                       Address structure fitting
*/
PktBasicAddressesInit xAddressInit = {
  EN_FILT_MY_ADDRESS,
  MY_ADDRESS,
  EN_FILT_MULTICAST_ADDRESS,
  MULTICAST_ADDRESS,
  EN_FILT_BROADCAST_ADDRESS,
  BROADCAST_ADDRESS
};
#endif


#define TIME_UP       0x01

RadioDriver_t         *pRadioDriver;
MCULowPowerMode_t     *pMCU_LPM_Comm;
RadioLowPowerMode_t   *pRadio_LPM_Comm;

                int   spirit_io_semaphore  = 0;  // I/O Semaphore  Duq
           uint16_t   Spirit_IRQ_Signalled = 0;  // 1 = got an IRQ GPIO_3 interrupt from Spirit Module

                                                 // Flags declarations
volatile FlagStatus   xRxDoneFlag  = RESET, xTxDoneFlag = RESET, cmdFlag  = RESET;
volatile FlagStatus   xStartRx     = RESET, rx_timeout  = RESET, exitTime = RESET;
volatile FlagStatus   datasendFlag = RESET,  wakeupFlag = RESET;
volatile FlagStatus   PushButtonStatusWakeup = RESET;
volatile FlagStatus   PushButtonStatusData   = RESET;
                                         /* IRQ status struct declaration */
static __IO uint32_t  KEYStatusData   = 0x00;
uint8_t               TxFrameBuff [MAX_BUFFER_LEN] = {0x00};
uint16_t              exitCounter     = 0;
uint16_t              txCounter       = 0;
uint16_t              wakeupCounter   = 0;
uint16_t              dataSendCounter = 0x00;
SpiritIrqs            xIrqStatus;
AppliFrame_t          xTxFrame,  xRxFrame;


/* Function prototypes -----------------------------------------------*/

void HAL_Spirit1_Init(void);
void Data_Comm_On(uint8_t *pTxBuff, uint8_t cTxlen, uint8_t* pRxBuff, uint8_t cRxlen);
void Enter_LP_mode(void);
void Exit_LP_mode(void);
void MCU_Enter_StopMode(void);
void MCU_Enter_StandbyMode(void);
void MCU_Enter_SleepMode(void);
void RadioPowerON(void);
void RadioPowerOFF(void);
void RadioStandBy(void);
void RadioSleep(void);
void AppliSendBuff(AppliFrame_t *xTxFrame, uint8_t cTxlen);
void AppliReceiveBuff(uint8_t *RxFrameBuff, uint8_t cRxlen);
void P2P_Init(void);
void STackProtocolInit(void);
void BasicProtocolInit(void);
void P2PInterruptHandler(void);
void Set_KeyStatus(FlagStatus val);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_SYSTICK_Callback(void);
void process_Spirit_IRQ_request(void);         // WVD ADD


/******************************************************************************
* @brief  Initializes RF Transceiver's HAL.
* @param  None
* @retval None.
*/
void  HAL_Spirit1_Init (void)
{
  pRadioDriver = &spirit_cb;
  pRadioDriver->Init();
}


/******************************************************************************
*                                    MAIN  LOOP
*
*                                   Data_Comm_On
*
* @brief  SPIRIT1 Data Transfer Routine.
* @param  uint8_t *pTxBuff = Pointer to aTransmitBuffer
*         uint8_t cTxlen = length of aTransmitBuffer
*         uint8_t* pRxBuff = Pointer to aReceiveBuffer
*         uint8_t cRxlen= length of aReceiveBuffer
* @retval None.
********************************************************************************/

void  Data_Comm_On (uint8_t *pTxBuff, uint8_t cTxlen, uint8_t* pRxBuff, uint8_t cRxlen)
{

#if defined(RF_STANDBY)
  if (wakeupFlag)
   {
    AppliReceiveBuff (pRxBuff, cRxlen);      // Check if any message was received

    if (datasendFlag)
      {
        datasendFlag      = RESET;
        xTxFrame.Cmd      = LED_TOGGLE;      // SEND A TOGGLE CMD to Remote Node
        xTxFrame.CmdLen   = 0x01;
        xTxFrame.Cmdtag   = txCounter++;
        xTxFrame.CmdType  = APPLI_CMD;
        xTxFrame.DataBuff = pTxBuff;
        xTxFrame.DataLen  = cTxlen;

        AppliSendBuff (&xTxFrame, xTxFrame.DataLen);  // Send Request

        AppliReceiveBuff (pRxBuff, cRxlen);           // Receive ACK Reply
      }

    if (cmdFlag)
       {
         cmdFlag           = RESET;
         xTxFrame.Cmd      = ACK_OK;                  // SEND ACK to RCVD MSG
         xTxFrame.CmdLen   = 0x01;
         xTxFrame.Cmdtag   = xRxFrame.Cmdtag;
         xTxFrame.CmdType  = APPLI_CMD;
         xTxFrame.DataBuff = pTxBuff;
         xTxFrame.DataLen  = cTxlen;

         AppliSendBuff (&xTxFrame, xTxFrame.DataLen); // Send it

         sys_Delay_Millis (DELAY_TX_LED_GLOW);

         RadioShieldLedOff (RADIO_SHIELD_LED);        // Then go sleep
         pin_Low (LED1);

         wakeupFlag = RESET;
         Enter_LP_mode();
       }
   }
  else if (datasendFlag == SET  &&  wakeupFlag == RESET)
          {
            datasendFlag = RESET;
            Enter_LP_mode();              // Nothing to do - go sleep till get Interrupt
          }
#else

     //----------------------------------------------------------------------
     //  Check if we got an interrupt from the Spirit Module via GPIO_3 EXTI.
     //  If so, process it, to ssee what condition occurred (rcvd frame, ...).
     // Interrupt may be due to rcvd frame, ack rcvd, or receive timeout.
     //----------------------------------------------------------------------
// if (Spirit_IRQ_Signalled)       // WVD ADD
//     { Spirit_IRQ_Signalled = 0;  // turn off flag to denote we processed it
//       process_Spirit_IRQ_request();
//     }

     //---------------------------------------------------------------------
     //  See if any frame rcvd from remote partner
     //---------------------------------------------------------------------
  AppliReceiveBuff (pRxBuff, cRxlen);

     //---------------------------------------------------------------------
     //  See if local user pressed the PB1 push button to send a messzage
     //---------------------------------------------------------------------
  if (KEYStatusData)
     {
       KEYStatusData     = RESET;
       xTxFrame.Cmd      = LED_TOGGLE;      // SEND A TOGGLE CMD to Remote Node
       xTxFrame.CmdLen   = 0x01;
       xTxFrame.Cmdtag   = txCounter++;
       xTxFrame.CmdType  = APPLI_CMD;
       xTxFrame.DataBuff = pTxBuff;
       xTxFrame.DataLen  = cTxlen;
             //-------------------------------------
             // Send a request to our partner
             //-------------------------------------
       AppliSendBuff (&xTxFrame, xTxFrame.DataLen);   // Send Request to partner

             //-------------------------------------
             // Receive a ACK ior NAK from partner
             //-------------------------------------
       AppliReceiveBuff (pRxBuff, cRxlen);            // Receive ACK Reply
     }

             //-----------------------------------------------------------------
             // See if we need to send an ACK ior a prev msg received from Partner
             //------------------------------------------------------------------
  if (cmdFlag)
     {
       cmdFlag           = RESET;
       xTxFrame.Cmd      = ACK_OK;                   // SEND ACK to RCVD MSG
       xTxFrame.CmdLen   = 0x01;
       xTxFrame.Cmdtag   = xRxFrame.Cmdtag;
       xTxFrame.CmdType  = APPLI_CMD;
       xTxFrame.DataBuff = pTxBuff;
       xTxFrame.DataLen  = cTxlen;

       AppliSendBuff (&xTxFrame, xTxFrame.DataLen);  // Send it

       sys_Delay_Millis (DELAY_TX_LED_GLOW);

       RadioShieldLedOff (RADIO_SHIELD_LED);
       pin_Low (LED1);
#if defined(LPM_ENABLE)
       Enter_LP_mode();
#endif
     }
         //----------------------------------------------------
         // Double-check to see if a new interrupt intervened
         //----------------------------------------------------
//  if (Spirit_IRQ_Signalled)       // WVD ADD
//     { Spirit_IRQ_Signalled = 0;  // turn off the flag to denote we processed it
//       process_Spirit_IRQ_request();
//     }

#endif
}


/******************************************************************************
*                                 AppliSendBuff
*
* @brief  This function handles the point-to-point packet transmission
* @param  AppliFrame_t *xTxFrame = Pointer to AppliFrame_t structure
*         uint8_t cTxlen = Length of aTransmitBuffer
* @retval None
*******************************************************************************/
void  AppliSendBuff (AppliFrame_t *xTxFrame, uint8_t cTxlen)
{
  uint8_t xIndex    = 0;
  uint8_t trxLength = 0;
  pRadioDriver      = &spirit_cb;

appli_send_buf_called++;    // WVD DEBUG

  TxFrameBuff[0] = xTxFrame->Cmd;      // Setup header
  TxFrameBuff[1] = xTxFrame->CmdLen;
  TxFrameBuff[2] = xTxFrame->Cmdtag;
  TxFrameBuff[3] = xTxFrame->CmdType;
  TxFrameBuff[4] = xTxFrame->DataLen;

  for (; xIndex < cTxlen; xIndex++)    // Move in Data / Command portion
     {
       TxFrameBuff[xIndex+5] =  xTxFrame->DataBuff[xIndex];
     }

  trxLength = (xIndex+5);

      /* Spirit IRQs enable */
  pRadioDriver->DisableIrq();          // Re-config Spirit chip IRQs
  pRadioDriver->EnableTxIrq();

      /* payload length config */
  pRadioDriver->SetPayloadLen (trxLength);

      /* rx timeout config */
  pRadioDriver->SetRxTimeout (RECEIVE_TIMEOUT);

      /* IRQ registers blanking */
  pRadioDriver->ClearIrqStatus();

      /* destination address */
  pRadioDriver->SetDestinationAddress (DESTINATION_ADDRESS);
      /* send the TX command */
  pRadioDriver->StartTx (TxFrameBuff, trxLength);

appli_send_buf_completed++;    // WVD DEBUG
}


/******************************************************************************
*                                    AppliReceiveBuff
*
* @brief  This function handles the point-to-point packet reception
* @param  uint8_t *RxFrameBuff = Pointer to ReceiveBuffer
*         uint8_t cRxlen = length of ReceiveBuffer
* @retval None
*******************************************************************************/
void  AppliReceiveBuff (uint8_t *RxFrameBuff, uint8_t cRxlen)
{
  uint8_t   xIndex = 0;
  uint8_t   ledToggleCtr = 0;
  /*float   rRSSIValue = 0;*/

appli_rcv_buf_called++;    // WVD DEBUG

  cmdFlag      = RESET;
  exitTime     = SET;
  exitCounter  = TIME_TO_EXIT_RX;
  pRadioDriver = &spirit_cb;

      /* Spirit IRQs enable */
  pRadioDriver->DisableIrq();        // WVD why is this sequence needed ???  08/08/15
  pRadioDriver->EnableRxIrq();       // It is executed with all CPU rupts off !
                                     // Perhaps just temp disable to SPI rupts ?
                                     // Or ignore multiple push buttons if I/O in progress ???
      /* payload length config */
  pRadioDriver->SetPayloadLen (PAYLOAD_LEN);
      /* rx timeout config */
  pRadioDriver->SetRxTimeout (RECEIVE_TIMEOUT);

      /* IRQ registers blanking */
  pRadioDriver->ClearIrqStatus();

  pRadioDriver->StartRx();      // Issue an RX command to go into receive mode

      //-------------------------------------------------------------------
      //                              KEY
      //
      // Wait for data received from remote node or timeout period expired.
      //
      // App spends most of its time waiting in this tight loop for data
      // to arrive, or timeout to expire.
      //-------------------------------------------------------------------
  while ((RESET == xRxDoneFlag) && (RESET == rx_timeout) && (SET == exitTime))
    {
      spirit_io_wait (0);
//    if (Spirit_IRQ_Signalled)       // WVD ADD
//       { Spirit_IRQ_Signalled = 0;  // turn off flag to denote we processed it
//         process_Spirit_IRQ_request();
//       }
    }

  if ((rx_timeout == SET) || (exitTime == RESET))
     {       // handle TIMEOUT
appli_rcv_buf_timeout++;    // WVD DEBUG
       rx_timeout = RESET;
       pin_Toggle (LED1);
     }
    else if (xRxDoneFlag)
     {       // Handle DATA RECEIVED
appli_rcv_buf_data++;       // WVD DEBUG
       xRxDoneFlag = RESET;

       pRadioDriver->GetRxPacket (RxFrameBuff,&cRxlen);
       /*rRSSIValue = Spirit1GetRssiTH();*/

       xRxFrame.Cmd     = RxFrameBuff[0];           // READ in HEADER
       xRxFrame.CmdLen  = RxFrameBuff[1];
       xRxFrame.Cmdtag  = RxFrameBuff[2];
       xRxFrame.CmdType = RxFrameBuff[3];
       xRxFrame.DataLen = RxFrameBuff[4];

       for (xIndex = 5; xIndex < cRxlen; xIndex++)  // READ in Data / Cmd
         {
           xRxFrame.DataBuff[xIndex] = RxFrameBuff[xIndex];
         }

           //------------------------------------------------------
           //                 DECODE  Cmd/Ack  Rcvd
           //------------------------------------------------------
       if (xRxFrame.Cmd == LED_TOGGLE)
          {
            RadioShieldLedOn (RADIO_SHIELD_LED);
            cmdFlag = SET;
          }

       if (xRxFrame.Cmd == ACK_OK)
          {    //-------------------------------------------------------
               //  toggle the LED to denote a data handshake succeeded
               //-------------------------------------------------------
            for ( ; ledToggleCtr < 5; ledToggleCtr++)
              {
                RadioShieldLedToggle (RADIO_SHIELD_LED);
                sys_Delay_Millis (DELAY_RX_LED_TOGGLE);
              }
            RadioShieldLedOff (RADIO_SHIELD_LED);
            pin_Low (LED1);

#if defined(LPM_ENABLE)
#if defined(RF_STANDBY)/*||defined(RF_SLEEP)*/
            wakeupFlag = RESET;
#endif
            Enter_LP_mode();
#endif
          }
      }
}


/*******************************************************************************
*                                    P2P_Init
*
* @brief  This function initializes the protocol for point-to-point
* communication
* @param  None
* @retval None
*******************************************************************************/
void  P2P_Init (void)
{
  pRadioDriver = &spirit_cb;

     /* Spirit IRQ config */
  pRadioDriver->GpioIrq (&xGpioIRQ);

      /* Spirit Radio config */
  pRadioDriver->RadioInit (&xRadioInit);

      /* Spirit Radio set power */
  pRadioDriver->SetRadioPower (POWER_INDEX, POWER_DBM);

      /* Spirit Packet config */
  pRadioDriver->PacketConfig();

  pRadioDriver->EnableSQI();

  pRadioDriver->SetRssiThreshold (RSSI_THRESHOLD);
}


/*******************************************************************************
*                                   STackProtocolInit
*
* @brief  This function initializes the STack Packet handler of spirit1
* @param  None
* @retval None
*******************************************************************************/
void  STackProtocolInit (void)
{
#if defined(USE_STack_PROTOCOL)
      /* Spirit Packet config */
  SpiritPktStackInit (&xStackInit);
  SpiritPktStackAddressesInit (&xAddressInit);
  SpiritPktStackLlpInit (&xStackLLPInit);

      /* require ack from the receiver */
  SpiritPktStackRequireAck (S_ENABLE);

  if (EN_FILT_SOURCE_ADDRESS)
     {
       SpiritPktStackFilterOnSourceAddress (S_ENABLE);
       SpiritPktStackSetRxSourceMask (SOURCE_ADDR_MASK);
       SpiritPktStackSetSourceReferenceAddress (SOURCE_ADDR_REF);
     }
    else
     {
       SpiritPktStackFilterOnSourceAddress (S_DISABLE);
     }

#endif
}


/******************************************************************************
*                                    BasicProtocolInit
*
* @brief  This function initializes the BASIC Packet handler of spirit1
* @param  None
* @retval None
*******************************************************************************/
void  BasicProtocolInit (void)
{
#if defined(USE_BASIC_PROTOCOL)
  /* Spirit Packet config */
  SpiritPktBasicInit(&xBasicInit);
  SpiritPktBasicAddressesInit(&xAddressInit);
#endif
}


/******************************************************************************
* @brief  This routine will put the radio and mcu into LPM
* @param  None
* @retval None
*/
void  Enter_LP_mode (void)
{

  pMCU_LPM_Comm   = &MCU_LPM_cb;
  pRadio_LPM_Comm = &Radio_LPM_cb;

#if defined(MCU_STOP_MODE)&&defined(RF_SHUTDOWN)
  {
    pRadio_LPM_Comm->RadioShutDown();
    pMCU_LPM_Comm->McuStopMode();
  }
#elif defined(MCU_STOP_MODE)&&defined(RF_STANDBY)
  {
    pRadio_LPM_Comm->RadioStandBy();
    pMCU_LPM_Comm->McuStopMode();
  }
#elif defined(MCU_STOP_MODE)&&defined(RF_SLEEP)
  {
    pRadio_LPM_Comm->RadioSleep();
    pMCU_LPM_Comm->McuStopMode();
  }
#elif defined(MCU_STANDBY_MODE)&&defined(RF_SHUTDOWN)
  {
    pRadio_LPM_Comm->RadioShutDown();
    pMCU_LPM_Comm->McuStandbyMode();
  }
#elif defined(MCU_STANDBY_MODE)&&defined(RF_STANDBY)
  {
    pRadio_LPM_Comm->RadioStandBy();
    pMCU_LPM_Comm->McuStandbyMode();
  }
#elif defined(MCU_STANDBY_MODE)&&defined(RF_SLEEP)
  {
    pRadio_LPM_Comm->RadioSleep();
    pMCU_LPM_Comm->McuStandbyMode();
  }
#elif defined(MCU_SLEEP_MODE)&&defined(RF_SHUTDOWN)
  {
    pRadio_LPM_Comm->RadioShutDown();
    pMCU_LPM_Comm->McuSleepMode();
  }
#elif defined(MCU_SLEEP_MODE)&&defined(RF_STANDBY)
  {
    pRadio_LPM_Comm->RadioStandBy();
    pMCU_LPM_Comm->McuSleepMode();
  }
#elif defined(MCU_SLEEP_MODE)&&defined(RF_SLEEP)
  {
    pRadio_LPM_Comm->RadioSleep();
    pMCU_LPM_Comm->McuSleepMode();
  }
#elif defined(MCU_STOP_MODE)
  pMCU_LPM_Comm->McuStopMode();

#elif defined(MCU_STANDBY_MODE)
  pMCU_LPM_Comm->McuStandbyMode();

#else
  pMCU_LPM_Comm->McuSleepMode();
#endif
}


/******************************************************************************
* @brief  This routine wake-up the mcu and radio from LPM
* @param  None
* @retval None
*/
void  Exit_LP_mode (void)
{
  pRadio_LPM_Comm = &Radio_LPM_cb;
  pRadio_LPM_Comm->RadioPowerON();
}


/******************************************************************************
* @brief  This routine puts the MCU into LOW POWER stop mode
* @param  None
* @retval None
*/
void  MCU_Enter_StopMode (void)
{
    HAL_PWR_EnterSTOPMode (PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);  /* Infinite loop */
}

/******************************************************************************
* @brief  This routine puts the MCU into LOW POWER standby mode
* @param  None
* @retval None
*/
void  MCU_Enter_StandbyMode (void)
{
    HAL_PWR_EnterSTANDBYMode();      /* Infinite loop */
}


/******************************************************************************
* @brief  This routine puts the MCU into LOW POWER sleep mode
* @param  None
* @retval None
*/
void MCU_Enter_SleepMode(void)
{
        /* Suspend Tick increment to prevent wakeup by Systick interrupt.
        ** Otherwise the Systick interrupt will wake up the device within 1ms
        ** (HAL time base) */
    HAL_SuspendTick();

    HAL_PWR_EnterSLEEPMode (PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);  /* Infinite loop */
}


/******************************************************************************
* @brief  This function will turn on the radio and waits till it enters the Ready state.
* @param  Param:None.
* @retval None
*
*/
void  RadioPowerON (void)
{
  SpiritCmdStrobeReady();

  do {        /* Delay for state transition */
          for (volatile uint8_t i=0; i!=0xFF; i++)
             ;

              /* Read the MC_STATUS register */
          SpiritRefreshStatus();
     } while (g_xStatus.MC_STATE!=MC_STATE_READY);
}


/******************************************************************************
* @brief  This function will Shut Down the radio.
* @param  Param:None.
* @retval None
*
*/
void  RadioPowerOFF (void)
{
  SpiritEnterShutdown();
}


/******************************************************************************
* @brief  This function will put the radio in standby state.
* @param  None.
* @retval None
*
*/
void  RadioStandBy (void)
{
  SpiritCmdStrobeStandby();

#if 0
  do{
    /* Delay for state transition */
    for(volatile uint8_t i=0; i!=0xFF; i++);

    /* Reads the MC_STATUS register */
    SpiritRefreshStatus();
  }
  while(g_xStatus.MC_STATE!=MC_STATE_STANDBY);
#endif
}


/******************************************************************************
* @brief  This function will put the radio in sleep state.
* @param  None.
* @retval None
*
*/
void  RadioSleep (void)
{
  SpiritCmdStrobeSleep();

#if 0
  do{
    /* Delay for state transition */
    for(volatile uint8_t i=0; i!=0xFF; i++);

    /* Reads the MC_STATUS register */
    SpiritRefreshStatus();
  }
  while(g_xStatus.MC_STATE!=MC_STATE_SLEEP);
#endif
}


/******************************************************************************
* @brief  This routine updates the respective status for a Pushbutton key press.
* @param  None
* @retval None
*/
void  Set_KeyStatus (FlagStatus val)
{
  if (val==SET)
     {
       KEYStatusData = 1;
     }
    else KEYStatusData = 0;
}



/******************************************************************************
*                            process_Spirit_IRQ_request
*
* Detailed processing after a Spirit GPIO_3 EXTI Interrupt was posted. Go
* query the Spirit, and see what it needs to tell us: frame rcvd, TX complete, ...
*
* This code orginally was in the P2PInterruptHandler ISR, and required
* all CPU interrupts be turned off while sending a SPI request to Spirit
* from within the ISR.
*
* Putting SPI calls into an Interrupt ISR is poor design, as is having to
* turn off all CPU interrupts while sending a SPI request/reply.
* Forces a crappy polling model.
*
* The ISR now just sets a flag to indicate we need to service the Spirit,
* which we do on a deferred basis in here.
******************************************************************************/

void  process_Spirit_IRQ_request (void)
{
spirit_process_IRQ_count++;

       //----------------------------------------------------------
       // Read in Spirit's IRQ register to see the reason for the
       // GPIO_3 IRQ interrupt to us
       //----------------------------------------------------------
  SpiritIrqGetStatus (&xIrqStatus);

       //-----------------------------------------------
       //     Check the SPIRIT TX_DATA_SENT IRQ flag
       //-----------------------------------------------
  if (xIrqStatus.IRQ_TX_DATA_SENT || xIrqStatus.IRQ_MAX_RE_TX_REACH)
     {
       xTxDoneFlag = SET; // Denote we got a TX complete interrupt from Spirit.
                          // Note this flag is waited on by the Spirit1StartTx()
                          // routine, to determine when the TX was completed.
                          // That sequence is initiated by AppliSendBuff().
spirit_got_TXdone++;
     }

       //-----------------------------------------------
       //    Check the SPIRIT RX_DATA_READY IRQ flag
       //-----------------------------------------------
    else if (xIrqStatus.IRQ_RX_DATA_READY)
     {
       xRxDoneFlag = SET;   // Denote we got a RX received interrupt from Spirit
                            // Note this flag is looked at by AppliReceiveBuff()
                            // routine, to determine if a RX frame was received.
                            // That sequence is initiated by main loop / pass
spirit_got_RXdone++;
     }

       //-----------------------------------------------
       //    Check the SPIRIT RX_DATA_DISCARD IRQ flag
       //-----------------------------------------------
    else if (xIrqStatus.IRQ_RX_DATA_DISC)
     {     // RX_Discard is used to clear the FIFO buffer for the next rcv.
           // RX command - to ensure the device will be ready for next reception
spirit_got_RXdiscard++;
       if (xIrqStatus.IRQ_RX_TIMEOUT)       // Stays stuck in this loop after Rcv Frame 08/09/15
          {                                 // Must I do SpiritDisable/EnableIRQ sequence to clear it ?
            SpiritCmdStrobeFlushRxFifo();
            rx_timeout = SET;
          }
     }
}


//*****************************************************************************
//*****************************************************************************
//                            IRQ   ISR   HANDLERs
//*****************************************************************************
//*****************************************************************************

//*****************************************************************************
//                         NEW LOGIC TO ELIMINATE IRQ RACES
//
// The original ST logic tried to eliminate races by brute force. It
// Would shut off interrupts for long periods of time, and issue SPI I/O
// out of the EXTI ISR (P2PInterruptHandler), with interrupts shut off
// the whole time as well.
//
// This new logic tries to correct those issues by going to a SEMAPHORE
// based approach, even when using Bare Metal
//
// Called by:
//    - AppliReceiveBuff - waiting to receive data or receive timeout
//    - Spirit1StartTx   - waiting to get ACK reply from Spirit for a transmit
//*****************************************************************************

void  spirit_io_wait (int flags)
{

redo_wait:
    IO_SEMAPHORE_WAIT (&spirit_io_semaphore);  // go into LPM or RTOS proc switch --> See CC3100 LOGIC

spirit_IO_WAIT_complete++;

    if (Spirit_IRQ_Signalled != 1)
       goto redo_wait;              // still no joy with IRQ reply from Spirit1

      //----------------------------------------------------------------------
      // We got an IRQ request from Spirit1. Read it's status register to see
      // if it was TX or RX I/O complete, or a Read Timeout.
      // If so, process accordingly
      //----------------------------------------------------------------------
      //------------------------------------------------------------------
      // Read in Spirit's IRQ register to see the reason for the
      // GPIO_3 IRQ interrupt to us.
      //------------------------------------------------------------------
spirit_process_IRQ_count++;

       //----------------------------------------------------------
       // Read in Spirit's IRQ register to see the reason for the
       // GPIO_3 IRQ interrupt to us
       //----------------------------------------------------------
  SpiritIrqGetStatus (&xIrqStatus);

       //-----------------------------------------------
       //     Check the SPIRIT TX_DATA_SENT IRQ flag
       //-----------------------------------------------
  if (xIrqStatus.IRQ_TX_DATA_SENT || xIrqStatus.IRQ_MAX_RE_TX_REACH)
     {
       xTxDoneFlag = SET; // Denote we got a TX complete interrupt from Spirit.
                          // Note this flag is waited on by the Spirit1StartTx()
                          // routine, to determine when the TX was completed.
                          // That sequence is initiated by AppliSendBuff().
spirit_got_TXdone++;
     }

       //-----------------------------------------------
       //    Check the SPIRIT RX_DATA_READY IRQ flag
       //-----------------------------------------------
    else if (xIrqStatus.IRQ_RX_DATA_READY)
     {
       xRxDoneFlag = SET;   // Denote we got a RX received interrupt from Spirit
                            // Note this flag is looked at by AppliReceiveBuff()
                            // routine, to determine if a RX frame was received.
                            // That sequence is initiated by main loop / pass
spirit_got_RXdone++;
     }

       //-----------------------------------------------
       //    Check the SPIRIT RX_DATA_DISCARD IRQ flag
       //-----------------------------------------------
    else if (xIrqStatus.IRQ_RX_DATA_DISC)
     {     // RX_Discard is used to clear the FIFO buffer for the next rcv.
           // RX command - to ensure the device will be ready for next reception
spirit_got_RXdiscard++;
       if (xIrqStatus.IRQ_RX_TIMEOUT)       // Stays stuck in this loop after Rcv Frame 08/09/15
          {                                 // Must I do SpiritDisable/EnableIRQ sequence to clear it ?
            SpiritCmdStrobeFlushRxFifo();
            rx_timeout = SET;
          }
     }

}


/******************************************************************************
*                                GPIO_3 IRQ ISR
*
*                              P2PInterruptHandler
*
* @brief  This function handles SubGhz board's GPIO_3 External interrupt request.
*
*         In this application it is used to manage the
*         Spirit IRQ, that is notified by the Spirit GPIO_3 pin.
*
* @param  None
* @retval None
******************************************************************************/
void  P2PInterruptHandler (void)
{
spirit_irq_count++;

                                 // NEW LOGIC
    Spirit_IRQ_Signalled = 1;    // Set flag to tell main() loop that the
                                 // Spirit board needs service. It will be
                                 // processed in the background, on next loop
                                 // thru the bare-metal inifinte while loop.

    IO_SEMAPHORE_RELEASE (&spirit_io_semaphore); // Signal I/O operation complete

       // the caller: EXTI7_xxx() issues the Clear Interrupt flag

#if defined(ORIGINAL_CODE)
      //------------------------------------------------------------------
      // Read in Spirit's IRQ register to see the reason for the
      // GPIO_3 IRQ interrupt to us
      //------------------------------------------------------------------
  SpiritIrqGetStatus (&xIrqStatus);

      /* Check the SPIRIT TX_DATA_SENT IRQ flag */
  if (xIrqStatus.IRQ_TX_DATA_SENT || xIrqStatus.IRQ_MAX_RE_TX_REACH)
     {
       xTxDoneFlag = SET;
     }
                /* Check the SPIRIT RX_DATA_READY IRQ flag */
    else if (xIrqStatus.IRQ_RX_DATA_READY)
     {
       xRxDoneFlag = SET;
     }
                /* Check the SPIRIT RX_DATA_DISC IRQ flag */
    else if(xIrqStatus.IRQ_RX_DATA_DISC)
     {
           /* RX command - to ensure the device will be ready for the next reception */
       if (xIrqStatus.IRQ_RX_TIMEOUT)
          {
            SpiritCmdStrobeFlushRxFifo();
            rx_timeout = SET;
          }
     }
#endif

}


/******************************************************************************
*                                SYSTICK   ISR   Handler
*
* @brief  SYSTICK callback.
* @param  None
* @retval None
*/
void  HAL_SYSTICK_Callback (void)
{
  if (exitTime)
     {
            /* Decreament the counter to check when 3 seconds has been elapsed */
       exitCounter--;
            /* 3 seconds has been elapsed*/
       if (exitCounter <= TIME_UP)
          {
            exitTime = RESET;
          }
     }

#if defined(RF_STANDBY)
           /* Check if Push Button pressed for wakeup or to send data*/
  if (PushButtonStatusWakeup)
     {
           /* Decreament the counter to check when 5 seconds has been elapsed */
       wakeupCounter--;

           /* 5 seconds has been elapsed*/
       if (wakeupCounter<=TIME_UP)
          {
                /* Perform wakeup opeartion */
            wakeupFlag = SET;
            Exit_LP_mode();
            pin_Toggle (LED1);
            PushButtonStatusWakeup = RESET;
            PushButtonStatusData   = SET;
          }
     }
    else if (PushButtonStatusData)
     {
       dataSendCounter--;
       if (dataSendCounter<=TIME_UP)
          {
            datasendFlag = SET;
            PushButtonStatusWakeup = RESET;
            PushButtonStatusData   = RESET;
          }
     }
#endif
}


/******************************************************************************
*                                EXTI   ISR   Handler
*
* @brief GPIO EXTI callback
* @param uint16_t GPIO_Pin
* @retval None
*/
void  HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{

#if defined(MCU_STOP_MODE)     /*if MCU is in stop mode*/

        /* Clear LOW POWER Wake Up Flag */
  __HAL_PWR_CLEAR_FLAG (PWR_FLAG_WU);

        /* Configures system clock after wake-up from LOW POWER STOP: enable HSE,
        ** PLL and select PLL as system clock source (HSE and PLL are disabled in
        ** LOW_POWER STOP mode) */
    SystemClockConfig_STOP();
#endif

#if defined(MCU_SLEEP_MODE)
        /* Resume Tick interrupt if disabled prior to LOW POWER sleep mode entry */
    HAL_ResumeTick();
#endif

        /* Initialize LEDs */
    RadioShieldLedInit (RADIO_SHIELD_LED);
    pin_Config (LED1, GPIO_OUTPUT, 0);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
