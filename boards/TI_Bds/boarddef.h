// Using CC3200 to directly drive the Motor BPs is probably a NON-STARTER

// Bottom line:
// - Stepper will work with everything, but will require CC3200 special tweaking
// - BLDC wil _ONLY_ work with CC2000.  Tough - that's TI's goof-up.
//   You can only do so many un-natural acts. It MIGHT work with MSP430-5529.TBD
// - DRV8846 not justified at this time.      Put effort into STM32 instead.
//   12V BDC motors can only gen 83 oz max torque.  Need 24V motors for 200+ oz
// - Consider RTU version of Stellaris ACIM.
// - SensorHub is only viable as an RTU solution.

//******************************************************************************
//
//                                boarddef.h                            TI MCUs
//
//
// Top level module for Board dependent definitions for Tiva, C2000, and MSP430
//
// Contains APIs for common routines implemented for every board, and invokes
// #include for the appropriate board_NNNNN.h file, based on MCU type. 
//
// History:
//   xx/xx/xx - Created.
//   12/14/14 - Renamed from board.h to boarddef.h, so we do not accidentally
//              pick up some of the board.h defs that TI has in their projects.
//
//
// MCU Hardware to be supported:
// ------------------------------
//    Launchpad  Tiva 123G     =  256 K Flash  /  32 K SRAM  (+ 2K EEPROM)
//    Launchpad  Tiva 1294XL   = 1024 K Flash  / 256 K SRAM  (+ 6K EEPROM)
//
//    Launchpad  C2000 F28027  =   32 K Flash  /   6 K SRAM  (standard)
//    Launchpad  C2000 F28027F =   32 K Flash  /   6 K SRAM  (FOC enabled)
//
//    Launchpad  MSP432 P401R  =  256 K Flash  /  64 K SRAM  (ARM Cortex-M4)
//
//    Launchpad  MSP430 5529   =  128 K Flash  /   8 K SRAM (+2K USB RAM Buffer)
//    Launchpad  MSP430 FR5969 =   64 K Flash  /   2 K SRAM
//
//    Launchpad  CC3200 WiFi   =   n/a  Flash  / 256 K SRAM (+ onbd SPI EEPROM)
//
//    Launchpad  MSP430 FR4133 = 15.3 K Flash  /   2 K SRAM    -- MAYBE --
//    Launchpad  MSP430 G2552  =   16 K Flash  / 0.5 K SRAM    -- MAYBE --
//
// History:
//   12/01/14 - Created as a generic high-level include, to select correct defs.
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

#ifndef  __BOARD_COMMON_H__
#define  __BOARD_COMMON_H__

#include "project_config_parms.h"

              //------------------------------------------------------
              //   Common Definitions (APIs, Consts) for All Boards
              //------------------------------------------------------
#include <stdint.h>               // contains uint32_t / uint16_t  etc  defs
#include <stdbool.h>
#include <string.h>               // strcpy/strlen  memcpy/memset  etc
#include <errno.h>

typedef  void (*P_EVENT_HANDLER)(void* pValue);
typedef  void (*TMR_CB_EVENT_HANDLER)(void *pValue, int rupt_flags);
typedef  void (*ADC_CB_EVENT_HANDLER)(void *pValue, uint16_t *channel_results,
                int num_channels, int flags);   // flags can contain ADC Id

#define  PIN_HIGH             0xFF
#define  PIN_LOW      ( ! PIN_HIGH)



              //*********************************************************
              //   Generic board level APIs used for Launchpad support
              //
              //  These board_xxx calls are used by the code to provide
              //  "low level" cross platform APIs for key GPIO, ADC, PWM,
              //  and Timer functions.
              //*********************************************************

                  //-----------------------
                  // generic APIs
                  //-----------------------
int   setup (void);
void  main_loop (void);

                  //-----------------------
                  //  Startup & Delay APIs
                  //-----------------------
void  board_init (long mcu_clock_rate);         // Function Prototypes
void  board_busy_wait_usec (long usec_delay);   // Delay of 1 us
void  board_delay_ms (long msec_delay);         // Delay of 1 ms
void  board_disable_global_interrupts (void);
void  board_enable_global_interrupts (void);
void  board_error_handler (void);
long  board_frequency_to_period_ticks (long frequency);

                  //-----------------
                  //  ADC APIs
                  //-----------------
int  board_adc_init (int adc_module_id, uint32_t clock_rate,
                     int trigger_type, int flags);
int  board_adc_config_channel (int adc_module_id, int channel_num,
                               int sequencer, int step_num,
                               int last,  int flags);
int  board_adc_check_conversions_done (int adc_module_id, int sequencer);
int  board_adc_disable (int adc_module_id, int sequencer);
int  board_adc_enable (int adc_module_id, int sequencer);
int  board_adc_set_callback (int adc_module_id, ADC_CB_EVENT_HANDLER callback_function, void *callback_parm);
int  board_adc_get_results (int adc_module_id, int sequencer,
                            uint16_t  channel_results[]);
int  board_adc_user_trigger_start (int adc_module_id, int sequencer);

                  //-----------------
                  //  DAC APIs
                  //-----------------
int  board_dac_init (int dac_module_id, uint32_t clock_rate, int flags);
int  board_dac_config_channel (int dac_module_id, int channel_num,
                               int trigger_type,
                               long sps_frequency, int flags);
int  board_dac_check_conversions_done (int dac_module_id, int channel_id);
int  board_dac_clear_conversions_done (int dac_module_id, int channel_num);
int  board_dac_enable_channel (int dac_module_id,  int channel_id, int flags);
int  board_dac_disable_channel (int dac_module_id, int channel_id, int flags);
int  board_dac_gen_sample_table (int wave_type, short *table_buf, int num_steps);
int  board_dac_set_sample_table (int module_id, int channel, short *table_buf, int num_steps);


                  //-----------------
                  //  GPIO APIs
                  //-----------------
void  board_gpio_init (void);
void  board_gpio_pin_config (uint32_t gpio_port, unsigned long pin, int dir,
                             int pull);
uint16_t  board_gpio_read_pins (int gpio_port_id, uint16_t pins_mask, int flags);
void  board_irq_pin_config (uint32_t gpio_port,  unsigned long pin,
                            int rise_fall, int pullup, uint32_t irq_vector_num,
                            unsigned long priority);
void  board_irq_pin_enable (uint32_t gpio_port,  unsigned long pin,
                            uint32_t irq_vector_num, int clear_pending);
void  board_spi_dma_init(void);
uint32_t board_spi_init (int spi_id,  int spi_mode,
                         int baud_rate_scalar, int use_dma);

                  //-----------------
                  //  I2C APIs
                  //-----------------
int  board_i2c_init (int i2c_module, long baud_rate, int i2c_ms_mode, int flags);
int  board_i2c_check_io_complete (int i2c_module, int flags);
int  board_i2c_nack (int i2c_module);
int  board_i2c_receive (int i2c_module, uint8_t *rcv_buf, int max_length, int flags);
int  board_i2c_send (int i2c_module, uint8_t *send_buf, int data_length, int flags);
int  board_i2c_send_receive (int i2c_module, uint8_t *send_buf, int send_length, 
                             uint8_t *rcv_buf, int max_rcv_length, int flags);
int  board_i2c_set_slave_address (int i2c_module, short slave_addr);
int  board_i2c_stop (int i2c_module);


                  //-----------------------------
                  //  Sysclocks / Systicks APIs
                  //-----------------------------
void  board_stop_WDT (void);
void  board_system_clock_config (long  mcu_clock_hz);
long  board_system_clock_get_frequency (void);
long  board_sys_IO_clock_get_frequency (void);
void  board_systick_timer_config (void);
unsigned long  board_systick_timer_get_value (void);
void  SysTick_ISR_Handler (void);


                  //-----------------------
                  //   Timer / PWM   APIs
                  //-----------------------
int  board_timerpwm_init (int module_id, int counter_type, long period_value,
                          int timer_clock_source, int flags);
int  board_timerpwm_check_completed (int module_id, int check_mask, int reset_flags);
int  board_timerpwm_config_channel (int module_id, int channel_id, long initial_duty, int mode, int flags);
int  board_timerpwm_config_channel_pair (int module_id, int channelA_id, int channelB_id, int flags);
int  board_timerpwm_config_trigger_mode (int module_id, int trig_type, int flags);
int  board_timerpwm_disable (int module_id, int flags);
int  board_timerpwm_enable_CCR_input (int module_id, int channel_id, long initial_duty, int flags);
int  board_timerpwm_enable_CCR_output (int module_id, int channel_id,
                           long initial_duty, int action_flags, int extended_flags);
int  board_timerpwm_enable (int module_id, int interrupt_flags);
long board_timerpwm_get_current_value (int module_id);
long board_timerpwm_get_CCR_capture_value (int module_id, int CCR_channel_num);
long board_timerpwm_get_duty_cycle (int modgen_id, int chan_id);
long board_timerpwm_get_period (int module_id);
int  board_timerpwm_reset_CCR_output (int module_id, int chan_id, int flags);
int  board_timerpwm_set_callback (int module_id, TMR_CB_EVENT_HANDLER callback_function, void *callback_parm);
int  board_timerpwm_set_duty_cycle (int modgen_id, int chan_num, long duty_cycle, int flags);
int  board_timerpwm_set_period (int module_id, long new_period_value, int flags);
int  board_timerpwm_set_dead_time (int module_id, int rising_edge, int falling_edge);
int  board_timerpwm_set_phase (int module_id, int channel_id, long phase_offset);
int  board_timerpwm_set_channel_output (int module_id, int channel_id, int output_mode, int flags);



                  //-----------------
                  //  UART APIs
                  //-----------------
void  board_uart_init (void);
char  board_uart_get_char (void);
int   board_uart_rx_data_check (void);
int   board_uart_read_string (char *read_buf, int buf_max_length);
void  board_uart_write_char (char outchar);
void  board_uart_write_string (char *outstr);


                  //-------------------------
                  //  UNIQUE-ID / CRC  APIs
                  //-------------------------
void  board_unique_crcid_init (unsigned long seed, int flags);
unsigned long  board_unique_crcid_compute (void *in_buf, int in_buf_length,
                                           int flags_32_8);

                  //-------------------------------
                  //  VTIMER (Virtual Timer) APIs
                  //-------------------------------
int  board_vtimer_start (int vtimer_id, uint32_t timer_duration_millis,
                         P_EVENT_HANDLER callback_function,  void *callback_parm);
int  board_vtimer_completed (int vtimer_id);
int  board_vtimer_reset (int vtimer_id);
void board_vtimer_check_expiration (uint32_t gsystick_millisecs);


                  // possible future

void  interval_timer_init (void);
void  interval_timer_disable (void);
void  interval_timer_enable (void);
void  timer_pwm_init (long flags);
void  timer_config_pwm_mode (long flags);
void  timer_config_period_duty_cycle (long period, long duty_cycle);
void  timer_disable_pwm (void);
void  timer_enable_pwm (void);

              // Extended APIs used for Launchpad support
void  gpio_force_all_control_pins_low (void);
void  gpio_quiesce_all_control_pins (void);
void  timer_force_pwm_pin_low (short pin_num);
void  timer_set_pin_for_pwm (short pin_num);
void  timer_set_pin_for_gpio (short pin_num);

              // Device Boosterpack specific APIs
//unsigned short SPI_DRV8711_ReadWrite (unsigned char dataHi, 
//                                      unsigned char dataLo);


//******************************************************************************
//                    GLOBAL  CONTANTS  USED  WITH  BOARD  APIs
//******************************************************************************

              //-------------------------------
              //   Internal Error Codes
              //-------------------------------
// TBD


              //------------------------------------------------------
              //------------------------------------------------------
              //------------------------------------------------------
              //           Pull in MCU and Board Specific Defs
              //------------------------------------------------------
              //------------------------------------------------------
              //------------------------------------------------------
#if defined(__MSP430F5529__) || defined(__MSP430FR6989__) || defined(__MSP430FR5969__) || defined(__MSP430FR4133__) || defined(__MSP430G2553__)

#define  MSP430_MCU                // generic that denotes this is a MSP430 MCU

#include "msp430.h"

#if ! defined(__MSP430G2553__)
#include "driverlib.h"             // key Driverlib includes
#endif

#include "msp430/board_MSP430.h"   // Board specific defs

#endif                             // MSP430       (16 bit)



#if defined(__MSP432P401R__) || defined(TARGET_IS_MSP432P4XX)

#define  MSP432_MCU                // generic that denotes this is a MSP432 ARM MCU

#include <msp.h>
#include <msp432.h>
#include "driverlib.h"             // key Driverlib includes

#include "msp432/board_MSP432.h"   // Board specific defs

#endif                             // MSP432       (ARM 32 bit)



#if defined(PART_TM4C123GH6PM)

#define  TIVA_MCU                  // generic that denotes this is a Tiva MCU

#include "tiva/board_Tiva_123g.h"  // Board specific defs

#endif                             // Tiva



#if defined(cc3200)

#define  CC3200_MCU                // generic that denotes this is a CC3200 MCU

#include "cc3200/board_CC3200.h"   // Board specific defs

#endif                             // CC3200



#if defined(C2000) || defined(F28027)

#define  C2000_MCU                 // generic that denotes this is a C2000 MCU

#include "c2000/board_C2000.h"

#endif                             // C2000


   //-----------------------------------------------------------------
   // supply defs that not included as part of Wiznet W5200/W5500
   //-----------------------------------------------------------------
#if defined(USES_W5200)  ||  defined(USES_W5500)

struct timeval
 {
     unsigned long  tv_sec;
     unsigned long  tv_usec;
 };

    // Need to add special logic to W5200 Handler to emulate FD_SET and SELECT
    // by setting IR2/IMR regs to interrupt on recv of disconnect situs


#define FD_SET                              WIZ_FD_SET
#define FD_CLR                              WIZ_FD_CLR
#define FD_ISSET                            WIZ_FD_ISSET
#define FD_ZERO                             WIZ_FD_ZERO
#define fd_set                              WIZ_FdSet_t

typedef struct WizFdSet_t                      /* Select socket array */
{
   uint32_t     fd_array;    /* Bit map of SOCKET Descriptors (Max 8 sockets) */
///uint32t      fd_array[(SL_FD_SETSIZE + 31)/32]; /* Bit map of SOCKET Descriptors */
} WIZ_FdSet_t;


/*
    Select's WIZ_FdSet_t SET function
    Sets current socket descriptor on WIZ_FdSet_t container
*/
void  WIZ_FD_SET (short fd, WIZ_FdSet_t *fdset);

/*
    Select's WIZ_FdSet_t CLR function
    Clears current socket descriptor on WIZ_FdSet_t container
*/
void  WIZ_FD_CLR (short fd, WIZ_FdSet_t *fdset);


/*
    Select's WIZ_FdSet_t ISSET function
    Checks if current socket descriptor is set (TRUE/FALSE)

    Returns TRUE if set, FALSE if unset
*/
short  WIZ_FD_ISSET (short fd, WIZ_FdSet_t *fdset);

/*
    Select's WIZ_FdSet_t ZERO function
    Clears all socket descriptors from WIZ_FdSet_t
*/
void  WIZ_FD_ZERO (WIZ_FdSet_t *fdset);
//#define  WIZ_FD_ZERO(WIZ_FdSet_t *fdset)  (*fdset = 0)

#define  fd_set   WIZ_FdSet_t

#endif                       //  defined(USES_W5200) || defined(USES_W5500)  



//
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//              Pin mappings for Booster packs to Launchpads
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//
// ----------------   Note conflicts on SPI-CS and IRQ with DRV8711's pin usage
// | CC3100  Defs |   Note conflicts on IRQ, RX, TX with DRV8301's pin usage
// ----------------
//   Hardware    Connector MSP430  MSP430  MSP430  MSP430   Tiva        C2000
//   Function      Pin LP   5529   FR5969  FR4133   G2553   TM4-123XL   28027F
//   --------      ------  ------  ------  ------  ------   ---------   ------
//   RX  USB CDC     --     P 4.5   P 2.1   P 1.1   P 1.1   PB0         GPIO28
//   TX  USB CDC     --     P 4.4   P 2.0   P 1.0   P 1.2   PB1         GPIO29
//  ENABLE/nHIB  X  J1-5    P 1.6   P 4.3   P 2.7   P 1.3   PE4/AIN9    GPIO34
//   SCLK           J1-7    P 3.2   P 2.2   P 5.1   P 1.5   PB4/SSI2Clk GPIO18
//
//   IRQ    in   X  J2-2    P 2.0   P 1.2   P 1.7   P 2.6XI PB2         GPIO19
//   SPI - CS    X  J2-3    P 2.2   P 3.0   P 1.6   P 2.7XO PE0         GPIO12
//   RESET          J2-5    RST     RST     RST     RST     RESET       RESET
//   MOSI/SDO       J2-6    P 3.0   P 1.6   P 5.2   P 1.7   PB7/SSI2Tx  GP16/32
//   MISO/SDI       J2-7    P 3.1   P 1.7   P 5.3   P 1.6   PB6/SSI2Rx  GP17/33
//
//  CC3100 Notes:
//     - For the SPI lines, the CC3100 always acts like a slave.
//     - For run state, ENABLE/nHIB must be HIGH.
//     - For run state, CS Assert has the pin go HIGH. De-Assert is LOW  VERIFY ???
//     - For requesting service, IRQ goes LOW.   (Trigger off off pin "falling")
//
//
// ----------------
// | DRV8711 Defs | Stepper - works with 20 or 40 pin LP
// ----------------
//   Hardware    Connector MSP430  MSP430  MSP430  MSP430   Tiva        C2000
//   Function      Pin LP   5529   FR5969  FR4133   G2553   TM4-123XL   28027F
//   --------      ------  ------  ------  ------  ------   ---------   ------
//   Potentiometer  J1-2    P 6.5   P 4.2   P 8.1   P 1.0   PB5/AIN11   ADCINA6
//   nSLEEP/ENABLE  J1-6    P 6.6   P 2.4   P 8.0   P 1.4   PE5         AIO4
//   SCLK           J1-7    P 3.2   P 2.2   P 5.1   P 1.5   PB4/SSI2Clk GPIO18
//   RESET          J1-8    P 2.7   P 3.4   P 2.5   P 2.0   PA5         AIO2
//   STEP/AIN1      J1-9    P 4.2   P 3.5   P 8.2   P 2.1   PA6         AIO10
//   DIR/AIN2       J1-10   P 4.1   P 3.6   P 8.3   P 2.2   PA7         AIO12
//
//   nSTALL      X  J2-2    P 2.0   P 1.2   P 1.7   P 2.6XI PB2         GPIO19
//   nFAULT      X  J2-3    P 2.2   P 3.0   P 1.6   P 2.7XO PE0         GPIO12
//   MOSI/SDO       J2-6    P 3.0   P 1.6   P 5.2   P 1.7   PB7/SSI2Tx  GP16/32
//   MISO/SDI       J2-7    P 3.1   P 1.7   P 5.3   P 1.6   PB6/SSI2Rx  GP17/33
//   BIN1           J2-8    P 2.6   P 1.5   P 1.3   P 2.5   PA4         GPIO6
//   BIN2           J2-9    P 2.3   P 1.4   P 1.4   P 2.4   PA3         GPIO7
//   SPI - CS       J2-10   P 8.1   P 1.3   P 1.5   P 2.3   PA2         AIO14
//
//  DRV8711 Notes:
//     - For run state, nSLEEP/ENABLE must be HIGH.
//     - For run state, RESET must be LOW.
//     - For run state, nSTALL input of LOW denotes a stall.
//     - For run state, nFAULT input of LOW denotes a fault.
//     - For run state, CS Assert has the pin go HIGH. De-Assert is LOW.
//     - Interval Timer should pop every 32 milli-seconds for Motion updates.
//     - Single Pulse width = minimum of 62.5 micro-secs.
//     - PWM range is typically 512-1024 pulses/second for Steppers.
//
//  Free pins left on 20-pin Launchpads after DRV8711 applied:
//   Function      Pin LP   5529   FR5969  FR4133   G2553   TM4-123XL    28027F
//   --------      ------  ------  ------  ------  ------   ---------    ------
//   Free #1   RX   J1-3    P 3.4   P 2.6   P 1.1    P 1.1  PB0/USB0ID   GPIO28  <---
//   Free #2   TX   J1-4    P 3.3   P 2.5   P 1.0    P 1.2  PB1/USB0VBus GPIO29  <---
//   Free #3 Button J1-5    P 1.8   P 4.3   P 2.7    P 1.3  PE4/AIN9     GPIO34
//   Free #3  Test  J2-4    P 7.4    N/C    P 5.0  TEST/TCK PF0/Switch2   N/C
//
//
// ----------------
// | DRV8301 Defs | BLDC - only works with 40 pin LP
// ----------------
//   Hardware    Connector MSP430  MSP430  MSP430  MSP430   Tiva        C2000
//   Function      Pin LP   5529   FR5969  FR4133   G2553   TM4-123XL   28027F
//   --------      ------  ------  ------  ------  ------   ---------   ------
//   FAULT  (RX)    J1-3    P 3.4   P 2.6   P 1.1   P 1.1   PB0         GPIO28
//   OCTW   (TX)    J1-4    P 3.3   P 2.5   P 1.0   P 1.2   PB1         GPIO29
//   SCLK           J1-7    P 3.2   P 2.2   P 5.1   P 1.5   PB4/SSI2Clk GPIO18
//
//   SPI - CS    X  J2-2    P 2.0   P 1.2   P 1.7   P 2.6XI PB2         GPIO19
//   MOSI/SDO       J2-6    P 3.0   P 1.6   P 5.2   P 1.7   PB7/SSI2Tx  GP16/32
//   MISO/SDI       J2-7    P 3.1   P 1.7   P 5.3   P 1.6   PB6/SSI2Rx  GP17/33
//   EN-GATE        J2-8    P 2.6   P 1.5   P 1.3   P 2.5   PA4         GPIO6
//   DC-CAL         J2-9    P 2.3   P 1.4   P 1.4   P 2.4   PA3         GPIO7
//
//   DC Bus-V-FB    J5-3   A0/P6.0    -      -        -     AIN7        ADCINA7
//   VA-FB          J5-4   A1/P6.1    -      -        -     AIN6        ADCINA3
//   VB-FB          J5-5   A2/P6.2    -      -        -     AIN5        ADCINA1
//   VC-FB          J5-6   A3/P6.3    -      -        -     AIN4        ADCINA0
//   IA-FB          J5-7   A4/P6.4    -      -        -     AIN2        ADCINB1
//   IB-FB          J5-8  A12/P7.0    -      -        -     AIN1        ADCINB3
//   IC-FB          J5-9    P 3.6     -      -        -     AIN0        ADCINB7
//                                                             
//   PWM A Hi       J6-1  TA2.2/P2.5  -      -        -     M1PWM6      EPWM1A
//   PWM A Lo       J6-2  TA2.1/P2.4  -      -        -     M1PWM7      EPWM1B
//   PWM B Hi       J6-3  TA0.4/P1.5  -      -        -     T3CCP1 !    EPWM2A
//   PWM B Lo       J6-4  TA0.3/P1.4  -      -        -     M0PWM6      EPWM2B
//   PWM C Hi       J6-5  TA0.2/P1.3  -      -        -     M0PWM7      EPWM3A
//   PWM C Lo       J6-6  TA0.1/P1.2  -      -        -     WT1CCP0 !   EPWM3B
//
//  DRV8301 Notes:
//     - For run state, xxxxxx must be HIGH.
//     - DC Bus and Motor Voltage feedbacks are scaled for 6V to 24V,
//       i.e. 0 on ADC represents zzzz and max on ADC represents 26.314 V.
//     - Low side Motor current sensing is scaled 0 amp to 14 amp peak (10 amp
//       RMS), i.e. 0 on ADC represents zzzz and max on ADC represents 16.5 amp.
//     - DRV8301 can be run in "Three input PWM mode", in which case it will
//       internally generate low-side complementary signal with minimum 
//      dead-time - WILL MSP4305529 work with this or is it missing ADCs on J5 ?
//
//
// ----------------
// | DRV8848 Defs | BDC -  - works with 20 or 40 pin LP    Limited to 4-18V
// ----------------
//   Hardware    Connector MSP430  MSP430  MSP430  MSP430   Tiva        C2000
//   Function      Pin LP   5529   FR5969  FR4133   G2553   TM4-123XL   28027F
//   --------      ------  ------  ------  ------  ------   ---------   ------
//   Potentiomtr X  J1-5    P 1.8   P 4.3   P 2.7   P 1.3   PE4/AIN9    GPIO34
//   BIN1  (PWM)    J1-9    P 4.2   P 3.5   P 8.2   P 2.1   PA6         AIO10
//   BIN2  (PWM)    J1-10   P 4.1   P 3.6   P 8.3   P 2.2   PA7         AIO12
//
// nSLEEP/ENABLE X  J2-2    P 2.0   P 1.2   P 1.7   P 2.6XI PB2         GPIO19
//   nFAULT      X  J2-3    P 2.2   P 3.0   P 1.6   P 2.7XO PE0         GPIO12
//   AIN1  (PWM)    J2-8    P 2.6   P 1.5   P 1.3   P 2.5   PA4         GPIO6
//   AIN2  (PWM)    J2-9    P 2.3   P 1.4   P 1.4   P 2.4   PA3         GPIO7
//
//  DRV8848 Notes:
//     - Potentiometer on bd can be used to set VREF for current limit voltage
//
//
// Due to conflicts with CC3100/CC3200 MOSI/MISO, treat SensorHub as RTU only
// ---------------- - Don't even TRY to have SensorHub on same stack as Motor BP
// | SensorHub BP | - works with 20 or 40 pin LP   (40 pin can access RF module)
// ----------------
//   Hardware    Connector MSP430  MSP430  MSP430  MSP430   Tiva        C2000
//   Function      Pin LP   5529   FR5969  FR4133   G2553   TM4-123XL   28027F
//   --------      ------  ------  ------  ------  ------   ---------   ------
//   RX             J1-3    P 3.4   P 2.6   P 1.1   P 1.1   PB0         GPIO28
//   TX             J1-4    P 3.3   P 2.5   P 1.0   P 1.2   PB1         GPIO29
//   Interrupt      J1-6    P 6.6   P 2.4   P 8.0   P 1.4   PE5         AIO4
//   SPI TX !       J1-8    P 2.7   P 3.4   P 2.5   P 2.0   PA5         AIO2
//   RF I2C         J1-9    P 4.2   P 3.5   P 8.2   P 2.1   PA6         AIO10
//   RF I2C         J1-10   P 4.1   P 3.6   P 8.3   P 2.2   PA7         AIO12
//
//   Interrupt      J2-2    P 2.0   P 1.2   P 1.7   P 2.6XI PB2         GPIO19
//   Interrupt      J2-3    P 2.2   P 3.0   P 1.6   P 2.7XO PE0         GPIO12
// Conflicts with CC3100/CC3200 default MOSI/MISO
//   Sensor I2C     J2-6    P 3.0   P 1.6   P 5.2   P 1.7   PB7/SSI2Tx  GP16/32
//   Sensor I2C     J2-7    P 3.1   P 1.7   P 5.3   P 1.6   PB6/SSI2Rx  GP17/33
//   SPI RX         J2-8    P 2.6   P 1.5   P 1.3   P 2.5   PA4         GPIO6
//   SPI - CS       J2-9    P 2.3   P 1.4   P 1.4   P 2.4   PA3         GPIO7
//   SPI CLK        J2-10   P 8.1   P 1.3   P 1.5   P 2.3   PA2         AIO14
//
//
// ----------------
// | C2K LED BP   | LED POWER - only works with 40 pin LP
// ----------------
//   Hardware    Connector MSP430  MSP430  MSP430  MSP430   Tiva        C2000
//   Function      Pin LP   5529   FR5969  FR4133   G2553   TM4-123XL   28027F
//   --------      ------  ------  ------  ------  ------   ---------   ------
//   I Red          J1-2    P 6.5    -      -        -      PB5/AIN11   ADCINA6
//   RX             J1-3    P 3.4    -      -        -      PB0         GPIO28
//   TX             J1-4    P 3.3    -      -        -      PB1         GPIO29
//   SCLK           J1-7    P 3.2    -      -        -      PB4/SSI2Clk GPIO18
//   I Blue         J1-8    P 2.7    -      -        -      PA5         ADCINA2
//   V Green        J1-9    P 4.2    -      -        -      PA6         ADCINB2
//   V Red          J1-10   P 4.1    -      -        -      PA7         ADCINB4
//
//   VDC - In       J2-10   P 8.1    -      -        -      PA2         ADCINB6
//
//                  J5-3   A0/P6.0    -      -        -     AIN7        ADCINA7
//                  J5-4   A1/P6.1    -      -        -     AIN6        ADCINA3
//   I Green        J5-5   A2/P6.2    -      -        -     AIN5        ADCINA1
//                  J5-6   A3/P6.3    -      -        -     AIN4        ADCINA0
//   I Blue         J5-7   A4/P6.4    -      -        -     AIN2        ADCINB1
//                  J5-8  A12/P7.0    -      -        -     AIN1        ADCINB3
//                  J5-9    P 3.6     -      -        -     AIN0        ADCINB7
//
//   PWM A Hi       J6-1  TA2.2/P2.5  -      -        -     M1PWM6      EPWM1A
//   PWM A Lo       J6-2  TA2.1/P2.4  -      -        -     M1PWM7      EPWM1B
//   PWM B Hi       J6-3  TA0.4/P1.5  -      -        -     T3CCP1 !    EPWM2A
//                  J6-4  TA0.3/P1.4  -      -        -     M0PWM6      EPWM2B
//                  J6-5  TA0.2/P1.3  -      -        -     M0PWM7      EPWM3A
//                  J6-6  TA0.1/P1.2  -      -        -     WT1CCP0     EPWM3B
//
//
// Overall Restrictions:
//   C2000 Launchpads share SPI and I2C on connector pins J2-J6 and J2-7.
//     - Cannot run I2C SensorHub BP in parallel with any SPI based Bosterpacks
//       (CC3xxx, DRV8711, ...)
//       _UNLESS_ SensorHub can be re-directed to use J6-connector pins 7 and 8
//
//
// ----------------   Note conflicts on SPI-CS and IRQ with DRV8711's pin usage
// | CC3200  Defs |   Note conflicts on IRQ, RX, TX with DRV8301's pin usage
// ----------------
//   Hardware    Connector MSP430  MSP430  MSP430  MSP430   Tiva        C2000
//  CC3200 Use     Pin LP   5529   FR5969  FR4133   G2553   TM4-123XL   28027F
//   --------      ------  ------  ------  ------  ------   ---------   ------
//   ADC_CH1/GP03   J1-2    P 6.5   P 4.2   P 8.1   P 1.0   PB5/AIN11   ADCINA6
//   RX     /GP13   J1-3    P 3.4   P 2.6   P 1.1   P 1.1   PB0         GPIO28
//   TX     /GP12   J1-4    P 3.3   P 2.5   P 1.0   P 1.2   PB1         GPIO29
//   GPIO   /GP06   J1-5    P 1.8   P 4.3   P 2.7   P 1.3   PE4/AIN9    GPIO34
//   ADC_CH2/GP04   J1-6    P 6.6   P 2.4   P 8.0   P 1.4   PE5         AIO4
//   SCLK   /GP14   J1-7    P 3.2   P 2.2   P 5.1   P 1.5   PB4/SSI2Clk GPIO18
//   GPIO   /GP07   J1-8    P 2.7   P 3.4   P 2.5   P 2.0   PA5         AIO2
//   I2C_SCL/GP10   J1-9    P 4.2   P 3.5   P 8.2   P 2.1   PA6         AIO10
//   I2C_SCL/GP11   J1-10   P 4.1   P 3.6   P 8.3   P 2.2   PA7         AIO12
//
//   GPIO   /GP28   J2-2    P 2.0   P 1.2   P 1.7   P 2.6XI PB2         GPIO19
//   SPI-CS /GP17   J2-3    P 2.2   P 3.0   P 1.6   P 2.7XO PE0         GPIO12
//   TEST   /GP31   J2-4    P 7.4    N/C    P 5.0  TEST/TCK PF0/Switch2   N/C
//   RESET_OUT      J2-5    RST     RST     RST     RST     RESET       RESET
//   MOSI   /GP16   J2-6    P 3.0   P 1.6   P 5.2   P 1.7   PB7/SSI2Tx  GP16/32
//   MISO   /GP15   J2-7    P 3.1   P 1.7   P 5.3   P 1.6   PB6/SSI2Rx  GP17/33
//   GOIO   /GP25   J2-8    P 2.6   P 1.5   P 1.3   P 2.5   PA4         GPIO6
//   GPIO   /GP01   J2-9    P 2.3   P 1.4   P 1.4   P 2.4   PA3         GPIO7
//   GPIO   /GP22   J2-10   P 8.1   P 1.3   P 1.5   P 2.3   PA2         AIO14
//
//   ADC_CH0/GP02   J5-3   A0/P6.0    -      -        -     AIN7        ADCINA7
//   ADC_CH3/GP05   J5-4   A1/P6.1    -      -        -     AIN6        ADCINA3
//   ADC_CH1/GP03 DNP J5-5   A2/P6.2    -      -        -     AIN5        ADCINA1
//   ADC_CH2/GP04 DNP J5-6   A3/P6.3    -      -        -     AIN4        ADCINA0
//  I2S_SYNC/GP08   J5-7   A4/P6.4    -      -        -     AIN2        ADCINB1
//   I2S_CLK/GP30   J5-8  A12/P7.0    -      -        -     AIN1        ADCINB3
//  I2S_DOUT/GP09   J5-9    P 3.6     -      -        -     AIN0        ADCINB7
//   I2S_DIN/GP00   J5-10
//
// BUT NO 0 ohm RESISTORs - DNP
//   PWM    /GP11   J6-1  TA2.2/P2.5  -      -        -     M1PWM6      EPWM1A
//   PWM    /GP10   J6-2  TA2.1/P2.4  -      -        -     M1PWM7      EPWM1B
//   PWM    /GP24   J6-3  TA0.4/P1.5  -      -        -     T3CCP1 !    EPWM2A
//   PWM    /GP09   J6-4  TA0.3/P1.4  -      -        -     M0PWM6      EPWM2B
//   CCAP   /GP25   J6-5  TA0.2/P1.3  -      -        -     M0PWM7      EPWM3A
//   CCAP   /GP28   J6-6  TA0.1/P1.2  -      -        -     WT1CCP0 !   EPWM3B
//   GPIO   /GP07   J6-7
//   GPIO   /GP05   J6-8
//   GPIO RX/GP23   J6-9  <- IS POP 0 ohm
//   GPIO TX/GP24   J6-10 <- IS POP 0 ohm
//
//  CC3200 Notes:
//     - GP00 (pin 50) on J5-10 doubles as a SPI-CS line
//     - ADCs are limited to 1.5 V max !
// the following _MAY_ cause problems, if sharing SPI lines with Motor BPs 
//     - For the SPI lines, the CC3200 acts like a Master.(unless re-do resistors)
//     - For run state, ENABLE/nHIB must be HIGH.
//     - For run state, CS Assert has the pin go HIGH. De-Assert is LOW  VERIFY ???
//     - For requesting service, IRQ goes LOW.   (Trigger off off pin "falling")

#endif                             //  __BOARD_COMMON_H__
