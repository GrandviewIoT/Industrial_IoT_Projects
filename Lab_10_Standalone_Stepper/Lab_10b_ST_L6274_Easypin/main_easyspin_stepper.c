
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                               main_easyspin_stepper.c
//
//
// Program to checkout and confirm MCU clock init at 8 MHz and 16 MHz and to
// properly setup 1 ms timers when clocks are running at those diferent rates.
//
//
// History:
//   04/30/15 - Created to support St "EasySpin" shield/board. Duquaine
// -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -
//
// The MIT License (MIT)
//
// Copyright (c) 2014-2015 Wayne Duquaine / Grandview Systems
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//******************************************************************************

#include "user_api.h"                // pull in high level User API defs
                                     //       and any key DriverLib/HAL_Lib defs

#include "device_config_common.h"    // pull in device defs

#include <string.h>                  // Include defs for any strxxx() / memxxx()

#include "easyspin.h"                // pull in Easy Spin API defs

#define TIMER_PRESCALER           (1024)      // from EasySpin code


void  EXTI15_10_IRQHandler (void);
void  TIM3_IRQHandler (void);
void  EasySpin_FlagInterruptHandler(void);
void  EasySpin_StepClockHandler (int motor);


    volatile uint16_t  gLastError;   // latest reported error from EasySpin

    int        total_motor_passes = 10;    // number of back and forth passes
                                           // the motor should perform
    int        i;

    uint16_t   get_status_reg_val     = 0;

    uint32_t   IRQ_Flag_rupt_seen     = 0;
    uint32_t   exti_hal_callback_seen = 0;
    uint32_t   MyFlag_rupt_seen       = 0;
    uint32_t   PWM_1_TIM3_rupt_seen   = 0;
    uint32_t   pwm_hal_callback_seen  = 0;

    int        debug_test = 0;          // 1 = loop on SPI debug test. 0 = no

    int        ret_code    = 0;
    char       uart_cmd_buf [40];       // holds any user input via UART
    char       uart_test1  = 1;
    long       pwm_period  = 0;
    long       mg_PWMclock = 0;
    long       mg_PWMperiod = 0;
    long       mg_Sysclock = 0;
    long       nmap_SysClkTicks = 0;

    uint32_t   pwm_duty_value = 0;
    uint16_t   adc_cur_value  = 0;
    uint16_t   adc_value      = 0;

#define TARGET_STEPPER_PPS         512     // typically 512-1024 pulse steps/sec
#define BASE_PWM_FREQUENCY   TARGET_STEPPER_PPS

#define PWM_20K_FREQUENCY   20000

int  configure_stepper_motor (void);       // function prototypes
void MyFlagInterruptHandler (void);
void Error_Handler (uint16_t error);
void timer_period_completed_callback (void *callback_parm, int interrupt_flags);


  uint32_t           uwPeriod = 0;

/*******************************************************************************
*                              MAIN               Application's entry point
*******************************************************************************/

int  main (int argc, char **argv)
{
    sys_Init (0, 0);                    // Turn off WDT, init MCU clocks, ...

    pin_Config (LED1, GPIO_OUTPUT, 0);  // Setup LED1 for output to show activity


       //--------------------------------------------------------------------
       // Configure the L6474 Stepper Ctl chip and the motor attached to it.
       // This includes downloading configuration parameters into the L6474.
       //--------------------------------------------------------------------
extern  volatile uint8_t   numberOfDevices;   // used by EasySpin
numberOfDevices = 1;    // save max # devices for EasySpin logic

//  ret_code = configure_stepper_motor();



     /* Attach the function Error_Handler (defined below) to the error Handler*/
EasySpin_AttachErrorHandler (Error_Handler);

     /* Attach the function MyFlagInterruptHandler (defined below) to the EXTI IRQ flag interrupt */
EasySpin_AttachFlagInterrupt (MyFlagInterruptHandler);

     //-----------------------------------------
     //    re-Init of the EasysSpin library
     //-----------------------------------------
EasySpin_Begin (1);



       //---------------------------------------------------------------
       //     Read inexistent register to test MyFlagInterruptHandler
       //---------------------------------------------------------------
       /* Try to read an inexistent register.  */
       /* The flag interrupt should be raised */
       /* and the MyFlagInterruptHandler function called */
//  EasySpin_CmdGetParam (0, (easySPIN_Registers_t) 0x1F);
    sys_Delay_Millis (500);

     /* Move device 0 of 16000 steps in the FORWARD direction */
EasySpin_Move (0, FORWARD, 16000);
     /* Wait for the motor of device 0 ends moving */
EasySpin_WaitWhileActive(0)  ;

HAL_Delay (1000);


    while (1)
      {
         //---------------------------------------------------------------------
         //                        MOTION   CONTROL
         //
         // Perform a repeated sequence of steps, as if we were doing DIY
         // printing or some other form of positioning/machining sequence.
         //---------------------------------------------------------------------
            //------------------------------------------------------------------
            // Setup an initial speed profile and go into full-step mode so that
            // we can quickly move the stepper to the desired starting position
            // for machinng something.
            //------------------------------------------------------------------
          /* Select full step mode for the motor */
        EasySpin_SelectStepMode (0, easySPIN_STEP_SEL_1);
          /* Set speed and acceleration to be consistent with full step mode */
        EasySpin_SetMaxSpeed (0,100);
        EasySpin_SetMinSpeed (0,50);
        EasySpin_SetAcceleration (0,10);
        EasySpin_SetDeceleration (0,10);

          /* Move the motor to go position 200 */
        EasySpin_GoTo (0,200);
          /* Wait for the motor to stop moving */
        EasySpin_WaitWhileActive (0)  ;

          /* Set the current position of motor to be its "Home" position */
        EasySpin_SetHome (0);
        pin_Toggle (LED1);                    // show activity - toggle LED

            //------------------------------------------------------------------
            // Now go into 1/16 step mode and set an associated speed profile
            // so we can machine something
            //------------------------------------------------------------------
          /* Set L6474 to drive motor in 1/16 microstepping mode */
        EasySpin_SelectStepMode (0,easySPIN_STEP_SEL_1_16);
          /* Update speed, acceleration, deceleration for 1/16 microstep mode*/
        EasySpin_SetMaxSpeed (0,1600);
        EasySpin_SetMinSpeed (0,800);
        EasySpin_SetAcceleration (0,160);
        EasySpin_SetDeceleration (0,160);
        pin_Toggle (LED1);                    // show activity - toggle LED

            //------------------------------------------------------------------
            // Now sweep the motor back and forth several passes as if
            // machining or printing something.
            //------------------------------------------------------------------
         for (i = 0;  i < total_motor_passes;  i++)
           {
                 /* Request motor to step FORWARD to position 6400 */
             EasySpin_GoTo (0,6400);
                 /* Wait for motor to stop moving */
             EasySpin_WaitWhileActive (0);

                 /* Request motor to step BACKWARD to -6400 position */
             EasySpin_GoTo (0,6400);
                 /* Wait for motor to stop moving */
             EasySpin_WaitWhileActive (0);

             pin_Toggle (LED1);        // show activity - toggle after each pass
           }

          /* In preparation for a new sequence, request Motor to go to Home */
        EasySpin_GoHome (0);
        EasySpin_WaitWhileActive (0)  ;

        board_delay_ms (1000);      // pause between sequences
      }
}



//******************************************************************************
//   Configure Stepper Motor
//
//             Configure the stepp er motor we will be using:
//                - Configure control pins:  DIR,
//                - Configure SPI uses to talk to EasySpin chip
//                - Setup interrupt handlers to catch IRQ requests from EasySpin
//******************************************************************************
int  configure_stepper_motor (void)
{
    int   rc;
    TIM_HandleTypeDef  *tim_handle;              // CHEAP HACK - FIX
      TIM_MasterConfigTypeDef  sMasterConfig;    // TEMP HACK


// --- begin --- The following is equivalent to Easyspin_Begin() code
       //---------------------------------------------------------------------
       // Configure easySPIN - DIR pin for device 1 - D7
       //---------------------------------------------------------------------
    pin_Config (L6474_DIR_1_PIN, GPIO_OUTPUT, 0);
    pin_High (L6474_DIR_1_PIN);            // set default direction = Forward
                                           //    1 = forward,  0 = Backward

       //---------------------------------------------------------------------
       // Configure easySPIN - STBY/RESET pin  -  D8
       //---------------------------------------------------------------------
    pin_Config (L6474_RESET_PIN, GPIO_OUTPUT, 0);
    pin_Low (L6474_RESET_PIN);                     // turn on RESET to L6474

     //--------------------------------------------------------------------------
     // Config easySPIN - Interrupt IRQ Flag pin and associated EXTI in NVIC - D2
     //
     // Note that this configures everything, but leaves the interrupt disabled.
     // This is done in order to complete any other needed config operations,
     // before turning on interrupts.
     // A subsequent call to pin_Enable_IRQ() will enable the NVIC interrupt.
     //--------------------------------------------------------------------------
    pin_Config_IRQ_Pin (L6474_IRQ_PIN, GPIO_RUPT_MODE_FALLING, PIN_USE_PULLUP,
                        L6474_EXTI_IRQ_NUM, 5);

// BUG in above code not setting IRQn right ?
        /* Set Priority of External Line Interrupt used for the Flag interrupt*/
     HAL_NVIC_SetPriority (L6474_EXTI_IRQ_NUM, 5, 0);     // EXTI15_10_IRQn
        /* Enable the External Line Interrupt used for the Flag interrupt */
     HAL_NVIC_EnableIRQ (L6474_EXTI_IRQ_NUM);

       //-----------------------------------------------------------------------
       //                         SPI  Init
       // Config easySPIN  SPI  interface and  CS  pin.            CS pin is D10
       // Uses SPI 1 (PA5 / PA6 / PA7) on most Nucleos.
       //-----------------------------------------------------------------------

// easyspin uses: SPI_BAUDRATEPRESCALER_32  -  FIX THIS

    pin_Config (L6474_CS_PIN, GPIO_OUTPUT, 0);           // configure Chip Select
    pin_High (L6474_CS_PIN);                             //   and DE-Assert CS
    rc = spi_Init (L6474_SPI_MODULE, SPI_MASTER, L6474_SPI_MODE,
                         L6474_SPI_BAUD, 0);

       //-----------------------------------------------------------------
       //                         PWM  Init
       // Configure the PWM module to be used to drive the Stepper motor
       //
       //   Uses PC7 (Arduino D9) = TIM3_CH2
       //   PC7 has Timer3 Ch2 for _all_ chips _except_ F3-02 and L0-53
       //-----------------------------------------------------------------
       // SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.

// easy_spin uses Init.ClockDivision = TIM_CLOCKDIVISION_DIV1   !!! ??? FIX THIS !!! ???  WVD

/// uwPeriod = (SystemCoreClock / 20000 ) - 1;       // set period = 20 K Hz - WVD LOGIC
    uwPeriod = 0;                                    // at startup, period is set to 0
    if (rc == HAL_OK)
       rc = pwm_Init (L6474_PWM_1_MODULE, uwPeriod,
                      0);                     // actual rate on scope = 40 KHz !
    timer_Set_Prescalar(L6474_PWM_1_MODULE, (TIMER_PRESCALER - 1), 0);  // ensure pwm_Init turned on clocks

//pHTim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

       //-------------------------------------
       // Configure PWM channel to be used
       //-------------------------------------
    if (rc == HAL_OK)
       rc = pwm_Config_Channel (L6474_PWM_1_MODULE, L6474_PWM_1_CHANNEL,
                                0, TIMER_PIN_POLARITY_HIGH);   // 0 % duty cycle
//  sConfigOC.OCMode     = TIM_OCMODE_PWM1;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;


// ??? !!!  WVD  ??? !!! RESOLVE THIS
    tim_handle = (TIM_HandleTypeDef*) board_timerpwm_get_handle (L6474_PWM_1_MODULE); // !!! CHEAP HACK - WVD FIX THIS !!! ???
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization (tim_handle, &sMasterConfig);


       //---------------------------------------------------------
       // Setup the callback handler for the PWM/Timer Interrupt
       //---------------------------------------------------------
    tim_handle = (TIM_HandleTypeDef*) board_timerpwm_get_handle (L6474_PWM_1_MODULE); // !!! CHEAP HACK - WVD FIX THIS !!! ???
    timer_Set_Callback (L6474_PWM_1_MODULE,
                        timer_period_completed_callback,
                        tim_handle);      // !!! CHEAP HACK - WVD FIX THIS !!! ???

       //-------------------------------------------------------------------
       // Enable the PWM module and its associated channels, and
       // have it issue a PWM/Timer Interrupt at end of each period rollover.
       //--------------------------------------------------------------------
    if (rc == HAL_OK)
       rc = pwm_Enable (L6474_PWM_1_MODULE, TIMER_PERIOD_INTERRUPT_ENABLED);
//     rc = HAL_TIM_PWM_Start_IT (&hTimPwm1, easySPIN_CHAN_TIMER_PWM1);  --> see easyspin.c EasySpin_Pwm1SetFreq()

       //---------------------------------------------------
       // Enable the NVIC for EXTI interrupts from the L6474
       //---------------------------------------------------
    pin_Enable_IRQ (L6474_IRQ_PIN, L6474_EXTI_IRQ_NUM, 1);

       //-------------------------------------------------------------
       // Take L6474 out of reset, so it can start up
       //-------------------------------------------------------------
    pin_High (L6474_RESET_PIN);

       //---------------------------------------------------------------
       // Attach the function MyFlagInterruptHandler (defined below) to
       // EasySpin's IRQ flag interrupt
       //---------------------------------------------------------------
    EasySpin_AttachFlagInterrupt (MyFlagInterruptHandler);

       //---------------------------------------------------------------
       // Attach the function Error_Handler (defined below) to
       // the error Handler for EasySpin
       //---------------------------------------------------------------
    EasySpin_AttachErrorHandler (Error_Handler);

       //-------------------------------------------------------------
       // Take L6474 out of reset, so it can start up
       //-------------------------------------------------------------
    pin_High (L6474_RESET_PIN);

  /* Let a delay after reset */
  HAL_Delay(1);

       //--------------------------------------------------------------------
       // Set all registers and context variables to the predefined values
       // from the easyspin_target_config.h file.
       // The easySPin registers are set with the predefined values from
       // file ~\motion_cube\Libraries\Easyspin\Inc\easyspin_target_config.h
       //--------------------------------------------------------------------
//while (debug_test)                           // WVD loop for SPI debug testing
  EasySpin_SetDeviceParamsToPredefinedValues();

  /* Disable easySPIN powerstage */
  for (uint32_t i = 0; i < 1; i++)
   {
     EasySpin_CmdDisable (i);
               /* Get Status, which will clear flags after start up */
     EasySpin_CmdGetStatus (i);
   }

// --- end --- The above is equivalent to Easyspin_Begin() code

    return (ret_code);
}


/*******************************************************************************
  * @brief  This function is the User handler for the IRQ flag interrupt.
  *
  *         This gets invoked by the HAL EXTI IRQ Handler user callback logic.
  *
  * @param  None
  * @retval None          ----- STM32 CODE -----
  *
  * When make this communicatable (JSON or MODBUS), encode and send any erro conditions detected below  ??? !!! WVD
*******************************************************************************/
void  MyFlagInterruptHandler (void)
{
  uint16_t  statusRegister;

MyFlag_rupt_seen++;                      // DEBUG

     /* Get the value of status register via the easySpin command GET_STATUS */
  statusRegister = EasySpin_CmdGetStatus (0);

get_status_reg_val = statusRegister;     // DEBUG

     /* Check HIZ flag: if set, power brigdes are disabled */
  if ((statusRegister & easySPIN_STATUS_HIZ) == easySPIN_STATUS_HIZ)
     {
        // HIZ state
     }

     /* Check direction bit */
  if ((statusRegister & easySPIN_STATUS_DIR) == easySPIN_STATUS_DIR)
     {
          // Forward direction is set
     }
    else
     {
          // Backward direction is set
     }

     /* Check NOTPERF_CMD flag: if set, the command received by SPI can't be performed */
     /* This often occures when a command is sent to the easySpin */
     /* while it is in HIZ state */
  if ((statusRegister & easySPIN_STATUS_NOTPERF_CMD) == easySPIN_STATUS_NOTPERF_CMD)
     {
          // Command received by SPI can't be performed
     }

     /* Check WRONG_CMD flag: if set, the command does not exist */
  if ((statusRegister & easySPIN_STATUS_WRONG_CMD) == easySPIN_STATUS_WRONG_CMD)
     {
          // command received by SPI does not exist
     }

     /* Check UVLO flag: if not set, there is an undervoltage lock-out */
  if ((statusRegister & easySPIN_STATUS_UVLO) == 0)
     {
          // undervoltage lock-out
     }

     /* Check TH_WRN flag: if not set, the thermal warning threshold is reached */
  if ((statusRegister & easySPIN_STATUS_TH_WRN) == 0)
     {
          // thermal warning threshold is reached
     }

     /* Check TH_SHD flag: if not set, the thermal shut down threshold is reached */
  if ((statusRegister & easySPIN_STATUS_TH_SD) == 0)
     {
          // thermal shut down threshold is reached *
     }

      /* Check OCD  flag: if not set, there is an overcurrent detection */
  if ((statusRegister & easySPIN_STATUS_OCD) == 0)
     {
          // overcurrent detection
     }
}


/*******************************************************************************
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None    -     ----- STM32 CODE -----
  */
static void  Error_Handler (uint16_t error)
{
     /* Backup error number */
  gLastError = error;

     /* Infinite loop */
// while(1)                                    // ??? WVD FIX ??? !!!
     {
     }
}


/*******************************************************************************
  * @brief External Line EXTI Callback from HAL
  *
  *        This is called by the HAL_GPIO_EXTI_IRQHandler() and
  *        is the hook used to call back into user code. It is a weak extern.
  *
  * @param[in] GPIO_Pin pin number
  * @retval None
******************************************************************************/

void  HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{

exti_hal_callback_seen++;

  if (GPIO_Pin == easySPIN_FLAG_PIN)
     {
       EasySpin_FlagInterruptHandler();   // it is for our PA10 pin, so call our handler
     }
 }


#if THIS_IS_IN_STM32_IT_C_MODULE
/******************************************************************************
  * @brief  This function handles interrupt for External lines 10 to 15.
  *
  *         This handles the IRQ interrupt from the L7464 chip.
  *         We use HAL's HAL_GPIO_EXTI_IRQHandler() to invoke a user callback.
  *         The user callback is named HAL_GPIO_EXTI_Callback().
  *
  * @param  None
  * @retval None
******************************************************************************/
void  EXTI15_10_IRQHandler (void)
{
IRQ_Flag_rupt_seen++;

    HAL_GPIO_EXTI_IRQHandler (GPIO_PIN_10);    // aka PA10 used by L6474 IRQ
                                               // This will invoke the callback above.
}
#endif



#if THIS_IS_IN_BOARD_TIMERS_C
            // BUT NEEDS TO BE MOVED TO RUPT HANDLER
            // How does this link up with the EasySpin stuff.
            // Or is the real missing component the EXT_IRQ handler for rupts from L474 ??
/*******************************************************************************
  * @brief  This function handles TIM3 interrupt request.
  *
  *         TIM3 is the PWM Timer used by the Stepper to drive the Motor.
  *         We catch TIM3's interrupt every time the period expires.
  *         We then use HAL_TIM_IRQHandler() to invoke the user callback
  *         for the Timer period expire/rollover.
  *         The user callback is named HAL_TIM_PWM_PulseFinishedCallback()
  *
  * @param  None
  * @retval None
*******************************************************************************/
void  TIM3_IRQHandler (void)
{
    TIM_HandleTypeDef  *hdltimer;

PWM_1_TIM3_rupt_seen++;

    hdltimer = board_timerpwm_get_handle (L6474_PWM_1_MODULE);
    HAL_TIM_IRQHandler (hdltimer);   // this will invoke the callback below
}
#endif




/******************************************************************************
  * @brief PWM Callback
  *
  *        This callback is invoked by HAL's HAL_TIM_IRQHandler() whenever
  *        a timer period expire/rollover interrupt occurs.
  *        It is the hook used to call back into user code. It is a weak extern.
  *        This logic is used to ramp the PWM speed up or down, based on speed
  *        profile ramp settings.
  *
  * @param[in] htim PWM handle pointer
  * @retval None
******************************************************************************/

//void  HAL_TIM_PWM_PulseFinishedCallback (TIM_HandleTypeDef *htim)  // ORIGINAL HAL callback version
void  timer_period_completed_callback (void *callback_parm, int interrupt_flags)
{
    TIM_HandleTypeDef  *htim;

pwm_hal_callback_seen++;

  htim = (TIM_HandleTypeDef*) callback_parm;  // co-erce the void to proper type

//  if ((htim->Instance == easySPIN_TIMER_PWM1)               // ORIGINAL CODE PWM1 popped
//    && (htim->Channel == easySPIN_HAL_ACT_CHAN_TIMER_PWM1)) // ORIGINAL CODE - on a period pop, channel is irrelevant !!!
  if ((htim->Instance == TIM3)                                // PWM1 popped
//  && (htim->Channel == TIMER_CHANNEL_2))      // is being set to CLEARED unless go thru HAL IRQ handler which yields: HAL_TIM_ACTIVE_CHANNEL_2
     )
     {
       if (EasySpin_GetDeviceState(0) != INACTIVE)
          {
            EasySpin_StepClockHandler (0);
          }
     }

  if ((htim->Instance == easySPIN_TIMER_PWM2)                    // PWM2 popped
    && (htim->Channel == easySPIN_HAL_ACT_CHAN_TIMER_PWM2))
     {
       if (EasySpin_GetDeviceState(1) != INACTIVE)
          {
            EasySpin_StepClockHandler (1);
          }
     }

  if ((htim->Instance == easySPIN_TIMER_PWM3)                    // PWM3 popped
    && (htim->Channel == easySPIN_HAL_ACT_CHAN_TIMER_PWM3))
     {
       HAL_GPIO_TogglePin (easySPIN_PWM_3_PORT, easySPIN_PWM_3_PIN);
       if (EasySpin_GetDeviceState(2) != INACTIVE)
          {
            EasySpin_StepClockHandler (2);
          }
     }
}


//***********************************************************************************
//                              TEMP   TEST    HACK
//***********************************************************************************

/*******************************************************************************
  *                                   SPI
  *
  * @brief SPI MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param[in] hspi SPI handle pointer
  * @retval None
  */
void  HAL_SPI_MspInit (SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  if (hspi->Instance == SPIx)
   {
       /*##-1- Enable peripherals and GPIO Clocks ##############################*/
       /* Enable GPIO TX/RX clock */
    SPIx_SCK_GPIO_CLK_ENABLE();
    SPIx_MISO_GPIO_CLK_ENABLE();
    SPIx_MOSI_GPIO_CLK_ENABLE();
       /* Enable SPI clock */
    SPIx_CLK_ENABLE();

       /*##-2- Configure peripheral GPIO #######################################*/
       /* SPI SCK GPIO pin configuration  */
    GPIO_InitStruct.Pin       = SPIx_SCK_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_MEDIUM;
    GPIO_InitStruct.Alternate = SPIx_SCK_AF;

    HAL_GPIO_Init (SPIx_SCK_GPIO_PORT, &GPIO_InitStruct);

       /* SPI MISO GPIO pin configuration  */
    GPIO_InitStruct.Pin       = SPIx_MISO_PIN;
    GPIO_InitStruct.Alternate = SPIx_MISO_AF;

    HAL_GPIO_Init (SPIx_MISO_GPIO_PORT, &GPIO_InitStruct);

       /* SPI MOSI GPIO pin configuration  */
    GPIO_InitStruct.Pin       = SPIx_MOSI_PIN;
    GPIO_InitStruct.Alternate = SPIx_MOSI_AF;

    HAL_GPIO_Init (SPIx_MOSI_GPIO_PORT, &GPIO_InitStruct);
   }
}


/*******************************************************************************
  *                                 PWM
  *
  * @brief PWM MSP Initialization
  * @param[in] htim_pwm PWM handle pointer
  * @retval None
  */
void  HAL_TIM_PWM_MspInit (TIM_HandleTypeDef* htim_pwm)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  if (htim_pwm->Instance == easySPIN_TIMER_PWM1)
   {
       /* Peripheral clock enable */
    __easySPIN_PWM1_CLCK_ENABLE();

       /* GPIO configuration */
    GPIO_InitStruct.Pin       = easySPIN_PWM_1_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = easySPIN_AFx_TIMx_PWM1;
    HAL_GPIO_Init (easySPIN_PWM_1_PORT, &GPIO_InitStruct);

       /* Set Interrupt Group Priority of Timer3 Interrupt*/
    HAL_NVIC_SetPriority(easySPIN_PWM1_IRQn, 4, 0);

       /* Enable the timer3 global Interrupt */
    HAL_NVIC_EnableIRQ(easySPIN_PWM1_IRQn);
   }

  else if (htim_pwm->Instance == easySPIN_TIMER_PWM2)
   {
       /* Peripheral clock enable */
    __easySPIN_PWM2_CLCK_ENABLE();

       /* GPIO configuration */
    GPIO_InitStruct.Pin       = easySPIN_PWM_2_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = easySPIN_AFx_TIMx_PWM2;
    HAL_GPIO_Init (easySPIN_PWM_2_PORT, &GPIO_InitStruct);

       /* Set Interrupt Group Priority of Timer2 Interrupt*/
    HAL_NVIC_SetPriority(easySPIN_PWM2_IRQn, 4, 0);

       /* Enable the timer2 global Interrupt */
    HAL_NVIC_EnableIRQ(easySPIN_PWM2_IRQn);

   }

  else if (htim_pwm->Instance == easySPIN_TIMER_PWM3)
   {
       /* Peripheral clock enable */
    __easySPIN_PWM3_CLCK_ENABLE();

       /* GPIO configuration */
    GPIO_InitStruct.Pin   = easySPIN_PWM_3_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init (easySPIN_PWM_3_PORT, &GPIO_InitStruct);

       /* Set Interrupt Group Priority of Timer4 Interrupt*/
    HAL_NVIC_SetPriority(easySPIN_PWM3_IRQn, 3, 0);

       /* Enable the timer4 global Interrupt */
    HAL_NVIC_EnableIRQ(easySPIN_PWM3_IRQn);
   }
}


/*******************************************************************************
  * @brief PWM Callback
  *
  *        This callback is invoked by HAL's HAL_TIM_IRQHandler() whenever
  *        a timer period expire/rollover interrupt occurs.
  *        It is the hook used to call back into user code. It is a weak extern.
  *        This logic is used to ramp the PWM speed up or down, based on speed
  *        profile ramp settings.
  *
  * @param[in] htim PWM handle pointer
  * @retval None
  */
// The following is never bing called:
void  HAL_TIM_PWM_PulseFinishedCallback (TIM_HandleTypeDef *htim)
{
  if ((htim->Instance == easySPIN_TIMER_PWM1) && (htim->Channel == easySPIN_HAL_ACT_CHAN_TIMER_PWM1))
    {
      if (EasySpin_GetDeviceState(0) != INACTIVE)
         {
           EasySpin_StepClockHandler(0);
         }
    }

  if ((htim->Instance == easySPIN_TIMER_PWM2) && (htim->Channel == easySPIN_HAL_ACT_CHAN_TIMER_PWM2))
    {
      if (EasySpin_GetDeviceState(1) != INACTIVE)
         {
           EasySpin_StepClockHandler(1);
         }
   }

  if ((htim->Instance == easySPIN_TIMER_PWM3) && (htim->Channel == easySPIN_HAL_ACT_CHAN_TIMER_PWM3))
    {
      HAL_GPIO_TogglePin(easySPIN_PWM_3_PORT, easySPIN_PWM_3_PIN);
      if (EasySpin_GetDeviceState(2) != INACTIVE)
         {
           EasySpin_StepClockHandler(2);
         }
    }
}


/*******************************************************************************
  * @brief External Line Callback
  *
  *        This is called by the HAL_GPIO_EXTI_IRQHandler() and
  *        is the hook used to call back into user code. It is a weak extern.
  *
  * @param[in] GPIO_Pin pin number
  * @retval None
  */
void  ORIG__HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin);

void  ORIG__HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
  if (GPIO_Pin == easySPIN_FLAG_PIN)
     {
       EasySpin_FlagInterruptHandler();   // rupt is for our PA10 pin, so call our handler
     }
 }
