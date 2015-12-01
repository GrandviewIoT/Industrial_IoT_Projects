// 09/16/15 - L4_75 _fails_, but was a really quick and dirty ports, so ISRs/IRQs
//            probably need to be tweaked.

// 09/15/15 - F4_01 _FINALLY_ works with my code (numerous hacks). Lots of bites
//            in the ass, most self-induced. Lack of decent code review, too much brute force !

// 09/15/15 - F0_72 - getting blown out by IAR because image > 16 K. Code = 28K
//            For M0/M0+, IAR limits KS to 16K, not 32K.  ARGGG !!!

// 09/13/15 - F4_46 fails to work with MEMS, _EVEN WITH THE ORIGINAL CODE_
//            It hangs waiting on a I2C_Reply, during BSP_HUM_TEMP_Init().
//                ==> Double check I2C on F4_46 is same one as F4_01 on D14/D15
//                    If not, update x_nucleo_iks01a1.h accordingly

/********1*********2*********3*********4*********5*********6*********7**********
*
*
*                              main_stm32_sensorhub.c
*
*
*  Standalone support to run STM32 with a X-Nucleo Sensor Hub board (IKS01A1),
*  which contains a wealth of sensors on a single board (temperature,
*  ...
*
*  It is designed to test/prove that the sensors all work properly and to
*  demonstrate the use of multiple sensors in a single application.
*  All of the sensors are I2C based, using the dedicated I2C pins (xx,XX)
*  on the Arudino-UNO compliant Nucleo board.
*  Sensor output is routed to UART.
*
*  UART = 115200 baud,  8-N-1
*
*
*  History:
*    03/19/15 - Initial version bring up. Works putting out raw values. Duquaine
*
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
*******************************************************************************/

#include "user_api.h"            // pull in high level User API defs as well
                                 //    as stdint.h / stdbool.h / DriverLib / HAL
#include "device_config_common.h"

#include "x_nucleo_iks01a1.h"          // defs for AxesRaw_TypeDef etc
#include "x_nucleo_iks01a1_pressure.h" // API defs for each sensor
#include "x_nucleo_iks01a1_hum_temp.h"
#include "x_nucleo_iks01a1_imu_6axes.h"
#include "x_nucleo_iks01a1_magneto.h"

#include <string.h>                    // strxxx and memxxx defs
#include <stdio.h>                     // sprintf defs
#include <math.h>
#include <errno.h>

void floatToInt(float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec);

                                      // globals
      UART_HandleTypeDef  UartHandle;
      DMA_HandleTypeDef   hdma_tx;
      DMA_HandleTypeDef   hdma_rx;
volatile  uint32_t        Uart_BaudRate;
volatile  uint8_t         UART_TxBuffer [256];
volatile  uint8_t         UART_RxBuffer [128];

volatile float            PRESSURE_Value; // hold data values read from sensors
volatile float            HUMIDITY_Value;
volatile float            TEMPERATURE_Value;
volatile AxesRaw_TypeDef  ACC_Value;
volatile AxesRaw_TypeDef  GYR_Value;
volatile AxesRaw_TypeDef  MAG_Value;

         int32_t          data[9];        // work areas to process sensor data
         int32_t          pd1, pd2;
         int32_t          d1, d2, d3, d4;
         char             dataOut[256];   // sprintf'd UART output data area

         int              mems_rc;
         char             initMsg[]  = "MEMS Initialization STARTED.\r\n";
         char             cmpltMsg[] = "MEMS Initialization completeded SUCCESSfully.\r\n";
         char             errorMsg[] = "MEMS Initialization FAILED! Terminating.\r\n";


/******************************************************************
* @brief  Splits a float into two integer values - a "whole number
*         part, and a fractional (precision) part.
*
* @param  in the float value as input
* @param  out_int the pointer to the integer part as output
* @param  out_dec the pointer to the decimal part as output
* @param  dec_prec the decimal precision to be used
* @retval None
*******************************************************************/
static void  floatToInt (float in, int32_t *out_int,
                         int32_t *out_dec, int32_t dec_prec)
{
    *out_int = (int32_t) in;
    in       = in - (float) (*out_int);
    *out_dec = (int32_t) trunc(in * pow(10,dec_prec));
}


/*******************************************************************************
*                                   main
*******************************************************************************/
int  main (int argc, char** argv)
{
    sys_Init (0,0);                 // initialize board: setup clocks, GPIOs,...

    Uart_BaudRate = 115200;
    CONSOLE_INIT (Uart_BaudRate);   // initialize the console (UARTMD/VCP)

    pin_Config (LED1, GPIO_OUTPUT, 0);  // Configure on-board Nucleo D13/LED1
    pin_Low (LED1);                     // set it off initially


    CONSOLE_WRITE (initMsg);        // Indicate iniatlize MEMS started

               /**************************************************************
               *          Initialize each of the sensors
               **************************************************************/
    mems_rc = BSP_PRESSURE_Init();
    if (mems_rc == 0)
       mems_rc = BSP_HUM_TEMP_Init();
       else CONSOLE_WRITE ("  BSP_PRESSURE_Init() failed ! \r\n");
    if (mems_rc == 0)
       mems_rc = BSP_IMU_6AXES_Init();
    if (mems_rc == 0)
       mems_rc = BSP_MAGNETO_Init();

    if (mems_rc == 0)
       CONSOLE_WRITE (cmpltMsg);          // tell user we are good to go
       else {      // we hit an error at startup - write out msg and quit.
                   // Otherwise, MEMS code will crash on a Hard Fault because
                   // code lookup tables were not setup (= NULL) if error found
              pin_High (LED1);   // Set LED on hard, to denote error/locked
              CONSOLE_WRITE (errorMsg);   // tell user I2C init FAILED
              while (1)
                ;                         // stop, we are dead in the water
            }

    while (1)
      {
               /**************************************************************
               *          Go read in the input from each sensor
               **************************************************************/
               //-------------------------------------------------------------
               // read in Pressure data and convert to printable form for UART
               //-------------------------------------------------------------
        BSP_PRESSURE_GetPressure ((float*) &PRESSURE_Value);
        floatToInt (PRESSURE_Value, &pd1, &pd2, 2);

               //-------------------------------------------------------------
               // read in Humidity and Temperature data, convert to print form
               //-------------------------------------------------------------
        BSP_HUM_TEMP_GetHumidity ((float*) &HUMIDITY_Value);
        BSP_HUM_TEMP_GetTemperature ((float*) &TEMPERATURE_Value);
        floatToInt (HUMIDITY_Value, &d1, &d2, 2);
        floatToInt (TEMPERATURE_Value, &d3, &d4, 2);

               //------------------------------------------------------------
               // read Accelerometer data into a X/Y/Z struct.
               //------------------------------------------------------------
        BSP_IMU_6AXES_X_GetAxesRaw ((AxesRaw_TypeDef*) &ACC_Value);
               // split it apart so whe can print it out to UART console
        data[0] = ACC_Value.AXIS_X;
        data[1] = ACC_Value.AXIS_Y;
        data[2] = ACC_Value.AXIS_Z;

               //------------------------------------------------------------
               // read Gyroscope data into a X/Y/Z struct.
               //------------------------------------------------------------
        BSP_IMU_6AXES_G_GetAxesRaw ((AxesRaw_TypeDef*) &GYR_Value);
               // split it apart so whe can print it out to UART console
        data[3] = GYR_Value.AXIS_X;
        data[4] = GYR_Value.AXIS_Y;
        data[5] = GYR_Value.AXIS_Z;

               //------------------------------------------------------------
               // read Magnometer data into a X/Y/Z struct.
               //------------------------------------------------------------
        BSP_MAGNETO_M_GetAxesRaw ((AxesRaw_TypeDef*) &MAG_Value);
               // split it apart so whe can print it out to UART console
        data[6] = MAG_Value.AXIS_X;
        data[7] = MAG_Value.AXIS_Y;
        data[8] = MAG_Value.AXIS_Z;

               /**************************************************************
               *          Write out the sensor results to the UART
               **************************************************************/
               // write out Pressure, Humidity, and Temperature data
        sprintf (dataOut, "PRESSURE: %d.%d     HUM: %d.%d     TEMP: %d.%d\n\r",
                (int) pd1, (int) pd2,  (int) d1, (int) d2, (int) d3, (int) d4);
        CONSOLE_WRITE (dataOut);

        sprintf (dataOut, "ACC_X: %d, ACC_Y: %d, ACC_Z: %d\n\r",
                 (int) data[0], (int) data[1], (int) data[2]);
        CONSOLE_WRITE (dataOut);

        sprintf (dataOut, "GYR_X: %d, GYR_Y: %d, GYR_Z: %d\n\r",
                 (int) data[3], (int) data[4], (int) data[5]);
        CONSOLE_WRITE (dataOut);

        sprintf (dataOut, "MAG_X: %d, MAG_Y: %d, MAG_Z: %d\n\r\n\r",
                 (int) data[6], (int) data[7], (int) data[8]);
        CONSOLE_WRITE (dataOut);

        pin_Toggle (LED1);         // toggle LED to show we are alive
        sys_Delay_Millis (1000);   // then wait 1 second before do next reading
      }                            //  end  while()
}

/******************************************************************************/
