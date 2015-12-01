
/********1*********2*********3*********4*********5*********6*********7**********
*
*
*                              main_mqtt_pub_sub_client.c
*
*
*  Simple MCU MQTT client to publish a fixed "Hello" packet to a remote server,
*  and waits for a simple reply.   (sends/rcvs a simple MODBUS packet of data.)
*       ==>  PUB + SUB  ???
*
*  It is designed to test/prove that basic connectivity from the MCU to a
*  designated MQTT server works.
*
*  This models what the PC/Lunix-based Test app "xxxx.c" performs.
*
*  It can be used to test against the PC/Lunix-based Test app "grabport.c",
*  or it can be run against an AB or Siemens PLC running the "PLC_Hello_Server"
*  ladder-logic application.
*
*
*  Grandview DB/DC Systems             DRDA Portable Toolkit
*    Licensed Property of Grandview DC/DC Systems
*    (c) Copyright  Grandview DB/DC Systems  1994, 2002
*    All rights reserved
*
*  History:
*    04/04/15 - Create to test MQTT support. Duquaine
*    04/04/15 - Worked first shot out of the barrel with both CC3100 and W5200.
*    05/27/15 - MSP430-FR6989 verified that it works (< 24 hours after rcvd board)! Duqu
*    06/08/15 - Integrate in ADC, PWM, CRC changes to match rest of STM32 bds.
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

#if DO_LOOPBACK
#define PUBLISH_TOPIC         "mqtt_demo_que"
#define SUBSCRIBE_TOPIC       "mqtt_demo_que"
//#define PUBLISH_TOPIC         "/msp/cc3100/demo"
////#define PUBLISH_TOPIC       "/msp/cc3100/demo/fromLP"
//#define SUBSCRIBE_TOPIC       "/msp/cc3100/demo"
#else
#define PUBLISH_TOPIC         "mqtt_pub_demo_que"
#define SUBSCRIBE_TOPIC       "mqtt_sub_demo_que"
#endif

//#define  MQTT_BROKER_SERVER   "gigap7"   // our local Mosquitto server  DNS LOOKUP FAILS !
//#define  MQTT_BROKER_SERVER  "iot.eclipse.org"  // remote PAHO test server

#include "boarddef.h"                 // pull in defs for MCU board being used

#include "MQTTClient.h"               // pull in defs for MQTT API calls

#include <stdlib.h>
#include <string.h>
#include <errno.h>

#define DO_LOOPBACK         1     // uncomment this so that we subscribe same
                                  // topic we publish to, creating a loopback

#define MQ_BUFF_SIZE            62                // MQTT message buffer size
#define MAC_ADDR_LEN            (6)

#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))

    int   CONFIG_MQ_TIMEOUT_MS = 1000;           // was 1000 (1 second)

    int    netwk_type        = NETWORK_TYPE;        // set default config parms
    int    ip_addr_type      = NET_ADDRESS_DEFAULT;
    int    netwk_ap_security = NETWORK_AP_SEC_TYPE;
    char   *netwk_ap_ssid    = NETWORK_AP_SSID;
    char   *netwk_ap_passwd  = NETWORK_AP_PASSSWD;
    char   *server_id        = MQTT_BROKER_SERVER;
    int    server_port       = MQTT_SERVER_PORT;
    int    console_read_wait = CONSOLE_STARTUP_INPUT_WAIT;

                         //-----------------------------------------------------
                         //             MQTT  Global  variables
                         //-----------------------------------------------------
    Network                 net_cb;
    Client                  hMQTTClient;                   // MQTT Client Handle
    MQTTPacket_connectData  cdata = MQTTPacket_connectData_initializer;
    MQTTMessage             pub_msg;
    unsigned char           snd_buf [MQ_BUFF_SIZE+2];
    unsigned char           rcv_buf [MQ_BUFF_SIZE+2];

    unsigned char           macAddressVal[MAC_ADDR_LEN];
    unsigned char           macAddressLen = MAC_ADDR_LEN;

    char                    macStr[18];      // Formatted MAC Address String
    char                    uniqueID[9];     // Unique ID generated from TLV RAND NUM and MAC Address

    int                     num_pub_msgs = 0;
    long                    num_sub_msgs = 0;

    int                     rc         = 0;
    int                     quit_flag  = 0;      // 1 = user type in quit/exit

    char                   uart_cmd_buf [40];    // holds any user input via UART

    int                     coil_on    = 1;

                         /* TCP MBAP Hdr ----------------------> | Actual MBUS Cmd  */
                         /*                                      | Force Single Coil */
                         /*  -- TID -- Protocol Id  Length   UID | Func  Coil #2    On/Off */
 unsigned char  coil_msg [12]  = { 0x00,0x01, 0x00,0x00, 0x00,0x06,0x11, 0x05, 0x00,0x02, 0xFF,0x00 };
 unsigned char  reply_buf [12] = { 0, 0, 0, 0, 0, 0, 0 };  /* reply should be echo of above */

#define  TID_LO_OFFSET    1       /* offset for TID low order byte       */
#define  ON_OFF_OFFSET   10       /* offset into msg buf for ON/OFF flag */

void  messageArrived (MessageData *md);            // local function prototypes
void  generateUniqueID (void);
void  uart_get_config_info (void);
void  process_user_cmds (void);


// ??? In future, add UART support to query for a user configurable Server_name and port
/*******************************************************************************
* uart_get_config_info
*
*            read in configuration information for Network and/or server name/id
*            and port number from UART.
*******************************************************************************/
void  uart_get_config_info (void)
{
}


/*******************************************************************************
* process_user_cmds
*
*            Read in a user command from the UART (Console), and process it.
*
*            This routine will then set processing flags that the main()
*            routine is sensitive to.
*******************************************************************************/
void  process_user_cmds (void)
{

#if defined(USES_CONSOLE_READ)
    int   rc;

    CONSOLE_WRITE ("Enter configuration parameter.\n\r");
    rc = CONSOLE_READ_STRING (uart_cmd_buf, sizeof(uart_cmd_buf));
       // note: CONSOLE_READ also echos back the char to user

    if (rc <= 0)
       return;               // no input, or user is still typing in cmd

       //-----------------------------------------
       //  process user commands to us
       //-----------------------------------------
    if (strcmp("quit",uart_cmd_buf) || strcmp("exit",uart_cmd_buf))
       quit_flag = 1;        // user wants to terminate sending to MQTT

    return;                  // cmd complete
#endif

}

/********************************************************************************
*    generateUniqueID            - change this to a:  board_get_random_id() call   ??? !!!
********************************************************************************/
#if defined(USES_TIVAWARE)
#include "driverlib/crc.h"             // Tivaware version of DriverLib
#endif

#if defined(STM32F401xx) || defined(STM32F072xB) || defined(STM32F103xB) || defined(STM32F334x8) || defined(STM32L152xE) || defined(STM32L053xx)
     // code was moved to board_F4.c and board_F0.c
     extern  CRC_HandleTypeDef   CrcHandle;
     extern  uint32_t            u32Buffer[];  // only supports 32-bit data bufs
#else
  #if defined(USE_HAL_DRIVER)            // STM32
     CRC_HandleTypeDef   CrcHandle;
     uint32_t            u32Buffer[6];  // only supports 32-bit data bufs
  #endif
#endif


void  generateUniqueID (void)
{
    uint32_t  crcResult;
    int       i;

#if defined(USES_MSPWARE)
            // MSP430 and MSP432: generate random seed from CRC unit
    CRC32_setSeed (TLV_RANDOM_NUM_1, CRC32_MODE);
    CRC32_set32BitData (TLV_RANDOM_NUM_2);
    CRC32_set32BitData (TLV_RANDOM_NUM_3);
    CRC32_set32BitData (TLV_RANDOM_NUM_4);
    for (i = 0; i < 6; i++)
        CRC32_set8BitData (macAddressVal[i], CRC32_MODE);
    crcResult = CRC32_getResult (CRC32_MODE);
#endif

#if defined(USES_TIVAWARE)
            // Tiva: generate random seed from CRC unit
            // set initial seed as 12345678
    board_unique_crcid_init (12345678, (CRC_CFG_SIZE_8BIT|CRC_CFG_INIT_SEED));

    board_unique_crcid_compute (macAddressVal, 6, CRC_CFG_SIZE_8BIT);

    crcResult = 12345678;     // BROKEN - NEEDS FIX
#endif

#if defined(USE_HAL_DRIVER)
            // STM32: generate random seed from CRC unit
#if defined(F4)
    __HAL_RCC_CRC_CLK_ENABLE();      // turn on CRC module's clocks
#endif
#if defined(STM32F072xB)
    __CRC_CLK_ENABLE();
#endif
       // Configure the CRC peripheral
    CrcHandle.Instance = CRC;
    i = HAL_CRC_Init (&CrcHandle);
    if (i != HAL_OK)
       {
          while (1);                 // Initialization Error - hang for debugger
       }
    for (i = 0;  i < 6;  i++)
       u32Buffer[i] = macAddressVal[i];  // convert from 8-bit to L.O. 32-bit

    crcResult = HAL_CRC_Accumulate (&CrcHandle, u32Buffer, 6);  // yields 0
    crcResult = HAL_CRC_Calculate (&CrcHandle, u32Buffer, 6);   // ditto WTF
#endif

#if defined(MSP430)
    board_unique_crcid_init (0L, 0);     // no seed value supplied
    crcResult = board_unique_crcid_compute (macAddressVal, sizeof(macAddressVal),
                                            8);
#endif

    if (crcResult != 0)
       sprintf (uniqueID, "%06X", crcResult);
       else sprintf (uniqueID, "%06X", &macAddressVal[0]);  // use MAC as seed
}


//****************************************************************************
//
//!    \brief MQTT message received callback - Called when a subscribed topic
//!                                            receives a message.
//! \param[in]                  data is the data passed to the callback
//!
//! \return                        None
//
//****************************************************************************
void  messageArrived (MessageData *data)
{
    char  topic_buf[24];
    char  data_buf[MQ_BUFF_SIZE];

    char  *tok;
    long  color;

    LED1_TOGGLE();    // toggle the LED each time we rcvd a msg

    num_sub_msgs++;   // inc count of # Subscriber msgs we have received

        // Check for buffer overflow on topic name
    if (data->topicName->lenstring.len >= sizeof(topic_buf))
      {
        DEBUG_LOG ("Topic name too long!\n\r");
        return;
      }
        // Check for buffer overflow on data buffer
    if (data->message->payloadlen >= sizeof(data_buf))
      {
        DEBUG_LOG ("Payload too long!\n\r");
        return;
      }

    strncpy (topic_buf, data->topicName->lenstring.data,
             MIN(sizeof(topic_buf), data->topicName->lenstring.len));
    topic_buf [data->topicName->lenstring.len] = 0;  // add trailing \0

    strncpy (data_buf, data->message->payload,
             MIN(sizeof(data_buf), data->message->payloadlen));
    data_buf [data->message->payloadlen] = 0;        // add trailing \0

#if DECODE_LATER_WITH_JSON
    tok     = strtok (data_buf, " ");
    color   = strtol (tok, NULL, 10);
    TA0CCR1 = PWM_PERIOD * (color/255.0);     // set new CCR1 PWM duty cycle
    tok     = strtok (NULL, " ");
    color   = strtol (tok, NULL, 10);
    TA0CCR2 = PWM_PERIOD * (color/255.0);     // set new CCR2 PWM duty cycle
    tok     = strtok (NULL, " ");
    color   = strtol (tok, NULL, 10);
    TA0CCR3 = PWM_PERIOD * (color/255.0);     // set new CCR3 PWM duty cycle
#endif

    return;
}


/*******************************************************************************
*                                   main
*******************************************************************************/
int  main (int argc, char** argv)
{
    board_stop_WDT();               // stop Watchdog timer

    board_init (0);  // init board: setup clocks, GPIOs,... with default clock speed

    LED1_INIT();                    // Setup LED1 for output

    if (console_read_wait > 0)
       uart_get_config_info();      // get server info from user via UART

    CONSOLE_WRITE ("\n\rMCU is initialized. MQTT client starting connection to Network/AP\n\r");

       //-------------------------------------------------------------------
       //  Begin MQTT initialization sequence
       //    - Connect to the TCP network  (wired Ethernet or WiFi AP)
       //      via network_init() call.
       //    - Establish a TCP connection to the remote Server/Broker
       //-------------------------------------------------------------------
               //**********************************************************
               //         Crank up the network support
               //*********************************************************/
    rc = Network_Init (&net_cb, ip_addr_type,
                       netwk_ap_ssid, netwk_ap_passwd,
                       netwk_ap_security);
    if (rc < 0)
       {
          CONSOLE_WRITE ("Network startup (e.g. connect to AP) failed. Terminating.\n\r");
          while (1) ;        // hang for debugger
       }
    CONSOLE_WRITE ("\n\rSuccessfully connected to the Network/AP.\n\r");

               //**********************************************************
               //         Connect to remote server
               //*********************************************************/
    rc = Connect_Server (&net_cb, server_id, server_port);

    if (rc != 0)
       {
          CONSOLE_WRITE (" Failed to connect to MQTT server node. Terminating.\n\r");
          while (1) ;        // hang for debugger
       }
    CONSOLE_WRITE ("Successfully connected to MQTT's TCP server node.\n\r");

        //------------------------------------------------------
        // Extract our local (Ethernet/WiFI) MAC Address.
        // This will be used to create a unique MQTT client Id.
        //------------------------------------------------------
#if defined(USES_CC3100)
    sl_NetCfgGet (SL_MAC_ADDRESS_GET, NULL, &macAddressLen,
                  (unsigned char*) macAddressVal);
#endif

#if defined(USES_W5200)
    getSHAR ((unsigned char*) &macAddressVal);          // get our MAC address
#endif

        // Convert our MAC Address into a formatted string
    snprintf (macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
              macAddressVal[0], macAddressVal[1], macAddressVal[2],
              macAddressVal[3], macAddressVal[4], macAddressVal[5]);

    mnet_get_local_mac_address ((unsigned char*) &macAddressVal[0]);
//  mnet_get_local_ip_address (unsigned long *ip_address, int *dhcp_status);

       //-------------------------------------------------------------------
       //  Initialize our MQTT Client handle.
       //
       // Pass in our network driver data struct, the send/rcv buffers that
       // MQTT should use for TCP I/O, and the max size of those buffers
       //-------------------------------------------------------------------
    MQTTClient (&hMQTTClient, &net_cb, CONFIG_MQ_TIMEOUT_MS,
                snd_buf, MQ_BUFF_SIZE+2,
                rcv_buf, MQ_BUFF_SIZE+2);

        // Generate a 32-bit unique ID using TLV Random Number and MAC Address
    generateUniqueID();            // we use this as an ID on the MQTT message

       //-------------------------------------------------------------------
       //  Perform MQTT level Connect to Broker.
       //
       //  Note that MQTT requires the client provide a Unique ID when
       //  connecting, so that it can easily track each different client.
       //-------------------------------------------------------------------
    cdata.MQTTVersion      = 3;
    cdata.clientID.cstring = uniqueID;
    rc = MQTTConnect (&hMQTTClient, &cdata);

//  if (rc != SUCCESS)        // rc always = -1, yet we keep working. WTF ???
       {
////     CONSOLE_WRITE (" Failed to start MQTT client \n\r");
//       while (1) ;        // hang for debugger
       }
    CONSOLE_WRITE (" Connected to MQTT Broker - started MQTT client successfully \n\r");

       //--------------------------------------------------------------------
       //  Tell MQTT Broker we want to listen for (subscribe to) any
       //  messages sent to the SUBSCRIBE_TOPIC.
       //
       // Rcvd subscribe messages will be handled off to the messageArrived()
       // callback.
       //--------------------------------------------------------------------  // does not appear to be setting new timeout value on Timer ! ?  not calling countdown() ?
    rc = MQTTSubscribe (&hMQTTClient, SUBSCRIBE_TOPIC, QOS0, messageArrived);  // 04/12/15 - Still having timing usses. CONNECT works, but SUBSCRIBE_ACK fails and give error
                                                                               //            because expired() is indicating it has already timed out  ==> timer->end_xx logic FUed, or WDT is ticking way too fast
    if (rc != 0)      // we are getting -1 on the Subscribe call
       {
         CONSOLE_WRITE (" Failed to subscribe to requested MQTT topic. Terminating.\n\r");
         while (1) ;        // hang for debugger
       }
    CONSOLE_WRITE (" Subscribed to MQTT topic successfully.\n\r");

       //-------------------------------------------------------------------
       //     main  WHILE  LOOP
       //-------------------------------------------------------------------
    while (quit_flag == 0)
      {
                //--------------------------------------------------------
                // pass control to MQTT logic, to see if any MQTT packets
                // were received.  ACKs will be internally handled.
                // SUBSCRIBE messages will drive messageArrived() above.
                //--------------------------------------------------------
        rc = MQTTYield (&hMQTTClient, 10);
        if (rc != 0)
           {
                 // no packets received, try again later
           }

        if (CONSOLE_CHECK_FOR_INPUT())
           {      // user is typing in a command - process it
                  // If user types in quit or exit, quit_flag will be set = 1
             process_user_cmds();
           }

//      if (publishID)           // flag indicating send button was pushed
           {
             pub_msg.dup        = 0;
             pub_msg.id         = 0;
             pub_msg.payload    = uniqueID;
             pub_msg.payloadlen = 8;
             pub_msg.qos        = QOS0;
             pub_msg.retained   = 0;
             rc = MQTTPublish (&hMQTTClient, PUBLISH_TOPIC, &pub_msg);

             if (rc != 0)
                {
                  CONSOLE_WRITE (" Failed to publish message to MQTT broker. Terminating.\n\r");
                  while (1) ;        // hang for debugger
                }
//           CONSOLE_WRITE (" Published message successfully \n\r");

             num_pub_msgs++;         // inc count of # msgs we have published
//           publishID = 0;          // clear the send button flag
           }

        board_delay_ms (100);
      }
}
