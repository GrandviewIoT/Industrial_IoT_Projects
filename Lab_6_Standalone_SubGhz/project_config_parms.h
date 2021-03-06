
/********1*********2*********3*********4*********5*********6*********7**********
*
*                            project_config_parms.h
*
* Provides a set of network and debug parms that are used ONLY for THIS specific
* projects.
*
* It is used to specify such things as CC3100/CC3200 Access Point and Password
* info, common BLE parms, whether the DEBUG_LOG or CONSOLE_WRITE/READ is to be
* used, how long we should wait for console config information, etc.
*
* It should only be used when you need to overrride the config parameters for a
* specific project.
*
*   -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
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

#ifndef __PROJ_CONF_PARMS_H__
#define __PROJ_CONF_PARMS_H__

             //-----------------------------------------------------------------
             // NOTE: the GATEWAY ADDRESS is specified in default_config_parms.h
             //-----------------------------------------------------------------

             //-----------------------------------------
             // just try to puing IBM  - xxxx 08/04/15
             //-----------------------------------------
//#define  REMOTE_SERVER_ID    "23.210.251.219"         /* aka:  www.ibm.com */
//#define  DEST_IP_ADDR_1         23
//#define  DEST_IP_ADDR_2        210
//#define  DEST_IP_ADDR_3        251
//#define  DEST_IP_ADDR_4        219

//#define  REMOTE_SERVER_PORT    80

             //-----------------------------------------
             // setup our E-Mail Server parms  - FAILS 08/04/15
             //-----------------------------------------
#define  REMOTE_SERVER_ID   "64.142.7.162"         /* aka:  mail.sonic.net */
#define  DEST_IP_ADDR_1         64
#define  DEST_IP_ADDR_2        142
#define  DEST_IP_ADDR_3          7
#define  DEST_IP_ADDR_4        162

#define  REMOTE_SERVER_PORT   587
#define  EMAIL_USER_NAME      "grandvu"
#define  EMAIL_PASSWORD       "nosie2"


// In this example, we are overriding just the CONSOLE_WRITE/CONSOLE_READ
// parameters (so we can write messages out to the UART). We then us the rest
// of the Network parameters that were setup  in default_project_config_parms.h

//#define  USES_CONSOLE_WRITE     1
//#define USES_CONSOLE_READ      1

           //---------------------------------------------
           //  put any project specific settings in here.
           //---------------------------------------------

// use the default_project_config_parms.h (in the ~/boards directory) as
// the template for what parameters are supported.

// If you want to override all of the parms, then it is simplest just to
// copy the entire contents of the default_project_config_parms.h file
// into here, and then modify them as necessary.
// Then comment out the #include "default_project_config_parms.h" statement below

// Otherwise. if you want to use the rest of the (non-overriden) parms located
// in the default parms config file, then enable the include for it below.
#include "default_project_config_parms.h"

#endif                          //  __PROJ_CONF_PARMS_H__

//*****************************************************************************
