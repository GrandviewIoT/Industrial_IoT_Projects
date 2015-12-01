
//*******1*********2*********3*********4*********5*********6*********7**********
//
//                                 arduino_compat.h
//
//
// Subset of Arduino API to allow porting of simple (non C++) applications.
//
// Some simple APIs, such as Serial, have been flattened to C style names, e.g.
// Serial::begin becomes Serial_begin, etc.
//
//
// History:
//   08/15/15 - Created to enable porting of Adafruit's GSM library to STM32.Duq
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


#ifndef  __ARD_COMPAT_H__
#define  __ARD_COMPAT_H__

#include "user_api.h"            // pull in std defs for User API


#define  OUTPUT   GPIO_OUTPUT
#define  INPUT    GPIO_INPUT

#define  HIGH     PIN_HIGH
#define  LOW      PIN_LOW

                         //---------------------------------------
                         //---------------------------------------
                         //        Common Base Level    APIs
                         //---------------------------------------
                         //---------------------------------------
#define  delay(ms_value)           board_delay_ms(ms_value)
#define  pinMode(pinX,io_dir)      board_gpio_pin_config(pinX,io_dir,0)
#define  digitalWrite(pinX,hi_lo)  board_gpio_write_pin(pinX,hi_lo)

#endif                   // __ARD_COMPAT_H__

