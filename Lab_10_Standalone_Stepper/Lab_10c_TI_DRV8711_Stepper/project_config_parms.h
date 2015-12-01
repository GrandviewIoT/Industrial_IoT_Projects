
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
*  Copyright (C) 2014 Grandview Systems
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*******************************************************************************/

#ifndef __PROJ_CONF_PARMS_H__
#define __PROJ_CONF_PARMS_H__

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

// Ensure proper options for this project are turned on, in case programmer
// re-created the project and forgot to update needed C Pre-processor defines
#ifndef USES_PWN
#define  USES_PWM   1
#endif

#endif                          //  __PROJ_CONF_PARMS_H__

//*****************************************************************************
