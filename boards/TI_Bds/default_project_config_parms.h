
/********1*********2*********3*********4*********5*********6*********7**********
*
*                         default_project_config_parms.h
*
* Provides a set of default Network and debug parms that can be used across
* all the projects.
*
* It is used to specify such things as CC3100/CC3200 Access Point and Password
* info, common BLE parms, whether the DEBUG_LOG is to be used, 
* how long we should wait for console config information, ...
*
* If you need to overrride for a specific project, ...
*
*   -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
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

#ifndef __DEF_PROJ_CONF_PARMS_H__
#define __DEF_PROJ_CONF_PARMS_H__


//****************************************************************************
//
//                           CONSOLE and DEBUG_LOG   Parms
//
// Use this to:
//     - tweak how long we should wait for initial CONSOLE Input from the user
//       for any TCP or WiFi AP configuration overrides.
//     - tweak whether any CONSOLE input/output will be used by the app.
//     - tweak whether a DEBUG_LOG is used (sends debug messages out to 
//       the Console)
//
// The CONSOLE_STARTUP_INPUT_WAIT indicates the number of seconds that
// we should wait for any initial input from the user.
//
// Setting CONSOLE_STARTUP_INPUT_WAIT to 0 indicates do not wait for any user
// input, just always use the default parms at startup.
//
// Setting CONSOLE_STARTUP_INPUT_WAIT to a high value like 9999 indicates we
// must always wait for the user to provide configuration input/overrides
// before connecting to any remote system.
//****************************************************************************

#define  CONSOLE_STARTUP_INPUT_WAIT        0

#if ! defined(CONSOLE_BAUD_RATE)
#define  CONSOLE_BAUD_RATE            115200
#endif

#if ! defined(USES_CONSOLE_WRITE)
             // discard all calls to CONSOLE_WRITE if it is not enabled
#define  CONSOLE_WRITE(output_string)
#define  CONSOLE_WRITE_CHAR(out_char)
#define  CONSOLE_GET_CHAR()                   0
#else
#define  CONSOLE_WRITE(output_string)        board_uart_write_string(output_string)
#define  CONSOLE_WRITE_CHAR(out_char)        board_uart_write_char(out_char)
#define  CONSOLE_GET_CHAR()                  board_uart_get_char()
#endif

#if ! defined(USES_CONSOLE_READ)      // USES_CONSOLE_READ_EDIT  instead ???
#define  CONSOLE_READ_STRING(in_buf,max_lng)  0
#define  CONSOLE_CHECK_FOR_INPUT()            0
#else
             // discard all calls to CONSOLE_READ if it is not enabled
#define  CONSOLE_READ_STRING(in_buf,max_lng) board_uart_read_string(in_buf,max_lng)
#define  CONSOLE_CHECK_FOR_INPUT()           board_uart_rx_data_check()
#endif

#if ! defined(USES_DEBUG_LOG)
             // discard all calls to DEBUG_LOG if it is not enabled
#define  DEBUG_LOG(output_string)
#else
#define  DEBUG_LOG(output_string)          board_uart_write(output_string)
#endif


//****************************************************************************
//
//                           TCP    Client/Server   Parms
//
// These are used by projects utilizing the TCP that need to connect to a
// remote server, or to setup a local TCP Server for Listening.
//
// These provide a DEFAULT set of config parms if the user does not override
// them from the CONSOLE.
//
// Modbus uses default port of 502
// MQTT   uses default port of 1883
//****************************************************************************
       // we use #ifndef in case the user previously defined the value
       // in a higher level project specific file

            // Set whether we want to configure ourself dynamically from DHCP
            // or statically with a specific "hard-wired" IP address.

#ifndef LOCAL_IP_ADDRESS_IS_STATIC
            // Set LOCAL_IP_ADDRESS_IS_STATIC to 1 to use a static address
            // Set LOCAL_IP_ADDRESS_IS_STATIC to 0 to use a dynamic DHCP address
#define    LOCAL_IP_ADDRESS_IS_STATIC 0       /* get a dynamic IP address from DHCP */
//#define  LOCAL_IP_ADDRESS_IS_STATIC 1       /* use a static IP address */
#endif

#ifndef LOCAL_IP_ADDRESS
#define   LOCAL_IP_ADDRESS     "62.9.48.15"
#define   LOCAL_IP_ADDR_1       62
#define   LOCAL_IP_ADDR_2        9
#define   LOCAL_IP_ADDR_3       48
#define   LOCAL_IP_ADDR_4       15
#endif

            // TCP Configuration we we are acting as a Client
#ifndef REMOTE_SERVER_ID
#define   REMOTE_SERVER_ID     "62.9.48.30"
#endif

#ifndef REMOTE_SERVER_PORT
 #if defined(USES_MQTT)
 #define  REMOTE_SERVER_PORT    1883
 #else
 #define  REMOTE_SERVER_PORT     502
 #endif
#endif


            // TCP Configuration we we are acting as a Server
#ifndef LOCAL_SERVER_PORT
 #if defined(USES_MQTT)
 #define  LOCAL_SERVER_PORT     1883
 #else
 #define  LOCAL_SERVER_PORT      502
 #endif
#endif


            // TCP Configuration we we are acting as a MQTT Client
#ifndef MQTT_BROKER_SERVER
#define   MQTT_BROKER_SERVER   "62.9.48.30"
//#define  MQTT_BROKER_SERVER   "gigap7"   // our local Mosquitto server  DNS LOOKUP FAILS !
#endif

#ifndef MQTT_SERVER_PORT
#define   MQTT_SERVER_PORT      1883
#endif

#ifndef PUBLISH_TOPIC
#define   PUBLISH_TOPIC        "mqtt_pub_demo_que"
#endif

#ifndef SUBSCRIBE_TOPIC
#define   SUBSCRIBE_TOPIC      "mqtt_sub_demo_que"
#endif


//****************************************************************************
//
//                           WiFi  Access Point (AP)   Parms
//
// These are used by projects utilizing the CC3000, CC3100, or CC3200 that
// must to connect to an AP.
//
// These provide a DEFAULT set of config parms if the user does not override
// them from the CONSOLE.
//****************************************************************************

#ifndef NETWORK_TYPE
 #if defined(USES_W5200) || defined(USES_W5500)
 #define   NETWORK_TYPE         NET_TYPE_WIRED_TCP
 #elif defined(USES_BLUENRG_BLE)
 #define   NETWORK_TYPE         NET_TYPE_BLE
 #elif defined(USES_UART_RTU)
 #define   NETWORK_TYPE         NET_TYPE_SERIAL_RTU
 #else
             // default to WiFi if user does not explicitly configure as Wired
 #define   NETWORK_TYPE         NET_TYPE_WIFI_TCP
 #endif
#endif

#ifndef ADDR_TYPE
#define   ADDR_TYPE            NET_ADDRESS_DEFAULT
#endif

#ifndef NETWORK_AP_SSID
#define   NETWORK_AP_SSID      "LinksysWD"               // NEW EP1200
#define   NETWORK_AP_PASSSWD   ""
#define   NETWORK_AP_SEC_TYPE  NET_SEC_AP_OPEN
//#define   NETWORK_AP_SSID      "linksys"               // OLD WRT54
//#define   NETWORK_AP_PASSSWD   "Fu672Yo7448Brain08"
//#define   NETWORK_AP_SEC_TYPE  NET_SEC_AP_WPA2
#endif

#ifndef NETWORK_AP_GATEWAY
#define   NETWORK_AP_GATEWAY   "LinksysWD"               // NEW EP1200
//#define   NETWORK_AP_GATEWAY   "linksys"               // OLD WRT54
#endif

#endif                          //  __DEF_PROJ_CONF_PARMS_H__

//*****************************************************************************
