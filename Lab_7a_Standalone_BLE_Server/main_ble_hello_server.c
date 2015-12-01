// See IAR menu:  Tools -> Configure Custom Argument Variables
//                Use Global Variables to create SYM_REF to STM32 HAL Libs

// 06/19/15 - WVD -  L1 keeps looping forever in HAL_GPIO_EXTI_Callback() ??? !!!
//            waiting for IRQ EXTI line to drop.   See bluenrg_interface.c
//            May also have exposure with Pushbutton EXTI logic (PC_13 EXTI13) not hooked up.

/********1*********2*********3*********4*********5*********6*********7**********
*
*
*                              main_ble_hello_server.c
*
*
*  Simple MCU BLE Server which advertises a custom ("vendor specific") service,
*  and waits for an incoming connection from a peripheral client. It receives a
*  fixed "Hello" packet from BLE the client, and sends back a simply reply
*  to the client. (It rcvs/sends a short simple MODBUS packet of data.)
*
*  It is designed to test/prove that basic connectivity from a client MCU BLE
*  board to the BLE advertising server works.
*
*  It can be used to test against the       ??? is there is a PC based BLE client app ???
*  PC/Lunix-based Test app "grabport.c",
*
*
*  Grandview DB/DC Systems             DRDA Portable Toolkit
*    Licensed Property of Grandview DC/DC Systems
*    (c) Copyright  Grandview DB/DC Systems  1994, 2002
*    All rights reserved
*
*  History:
*    03/16/15 - Converted over to common communications mnet API. Duquaine
*    03/19/15 - Got revised version fully working with STM32 F4_01.
*    06/10/15 - Got working with STM32 F0_72. Duq
*    06/12/15 - Got working with STM32 F3_34 on first try. Duq
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

#include "user_api.h"                 // MCU and Board specific parms/pins/etc
#include "project_config_parms.h"

#include "mnet_call_api.h"            // pull in defs for common COMM API
#include <errno.h>

void  uart_get_config_info (void);    // Forward ref
void  User_Process (void);

#define  NETWORK_TYPE         NET_TYPE_BLE

              //--------------------------------------------------
              // Configuration Network Information of BLUENRG BLE
              //--------------------------------------------------
//-----------------------------------------------------------------------------
//  NOTE:   SERVER (1) = the peripheral performing a function/service (e.g.
//                       Heart Rate monitor). It advertises itself to others.
//                       It acts as a GATT SERVER to publish its data.
//          CLIENT (0) = Central monitor that listens for services/devices
//                       it is interested in talking to. Typically it is a
//                       Smartphone or Tablet. It
//                       acts as a GATT Client, receiving data from GATT Server.
//-----------------------------------------------------------------------------

    BLE_RoleTypeDef    BLE_Role = SERVER;    // need to make configurable via parms.h

// ??? !!!  THESE ADDRESSES NEED TO BE MADE CONFIGUREABLE  in parms.h !!! ???   WVD

    uint8_t            CLIENT_BDADDR[] = { 0xBB, 0x00, 0x00, 0xE1, 0x80, 0x02 };
    uint8_t            SERVER_BDADDR[] = { 0xAA, 0x00, 0x00, 0xE1, 0x80, 0x02 };


extern volatile uint8_t   set_connectable;    // defined in ble_sample_service.c
extern volatile int       connected;          //     KEY CONTROL VARIABLES
extern volatile uint8_t   notification_enabled;

    int    netwk_type        = NETWORK_TYPE;        // set default config parms
    int    ip_addr_type      = NET_ADDRESS_DEFAULT;

    int    netwk_ap_security = NETWORK_AP_SEC_TYPE;
    char   *netwk_ap_ssid    = NETWORK_AP_SSID;
    char   *netwk_ap_passwd  = NETWORK_AP_PASSSWD;
    char   *server_id        = REMOTE_SERVER_ID;
    int    server_port       = REMOTE_SERVER_PORT;
    int    console_read_wait = CONSOLE_STARTUP_INPUT_WAIT;

    int    net_id     = 0;
    int    sock_id    = 0;
    int    numc       = 0;
    int    ret_code   = 0;

    int    coil_on    = 1;

                         /* TCP MBAP Hdr ----------------------> | Actual MBUS Cmd  */
                         /*                                      | Force Single Coil */
                         /*  -- TID -- Protocol Id  Length   UID | Func  Coil #2    On/Off */
 unsigned char  coil_msg [12]  = { 0x00,0x01, 0x00,0x00, 0x00,0x06,0x11, 0x05, 0x00,0x02, 0xFF,0x00 };
 unsigned char  reply_buf [12] = { 0, 0, 0, 0, 0, 0, 0 };  /* reply should be echo of above */

#define  TID_LO_OFFSET    1       /* offset for TID low order byte       */
#define  ON_OFF_OFFSET   10       /* offset into msg buf for ON/OFF flag */



// ??? In future, add UART support to query for a user configurable Server_name and port
/*******************************************************************************
* uart_get_config_info
*
*            read in configuration information for Network and/or server name/id
*            and port number from UART.     (Optionally, also coil_on/off)
*******************************************************************************/
void  uart_get_config_info (void)
{
}


/*******************************************************************************
*                                   main
*******************************************************************************/
int  main (int argc, char** argv)
{
    sys_Init (0,0);                 // initialize board: setup clocks, GPIOs,...
                                    // Does HAL_Init() as part of its processing
#if defined(USES_CONSOLE_WRITE)
    board_uart_init (115200);       // startup UART at 115200 baud
#endif

    if (console_read_wait > 0)
       uart_get_config_info();         // get server info from user via UART


    pin_Config (LED1, GPIO_OUTPUT, 0); // Configure LED1 as a toggle when we rcv

    BSP_PB_Init (BUTTON_USER, BUTTON_MODE_GPIO);  // Configure User Button


               /**************************************************************
               *          Crank up network support
               **************************************************************/
    net_id = mnet_connect_network (netwk_type, ip_addr_type,
                                   netwk_ap_ssid, netwk_ap_passwd, netwk_ap_security,
                                   FLAGS_SERVER_GATT_ADVERTISER);

    if (net_id < 0)
       {
          CONSOLE_WRITE ("Network startup (e.g. BLE connect) failed.\n\r");
          while (1) ;        // hang for debugger
       }


  while (1)
    {
      HCI_Process();   // Invoke BLE processing to see if anything rcvd
      User_Process();  // Invoke user logic to see if anything needs to be sent
    }




#if defined(LATER)
               /**************************************************************
               *    Set us up as a Server, that advertises our custom Service
               **************************************************************/
    srv_sock_id = mnet_server_listen (net_id, server_port, CONFIG_SERVICE_NAME,
                                      0);
//                                   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^      <-----
//                         Need to also pass a set/array of UUIDs for the array
//                         Can server_port be used for anything, or set it to 0 ?
//  Alternative:
//     srv_sock_id = mnet_server_listen (net_id, server_port, CONFIG_SERVICE_NAME,
//                                       &ptr_uuid_array, num_entries, 0);
//          OR DEFINE A SPECIFIC SEPARATE API CALL FOR BLE  (as have to for Pub/Sub)
//          e.g. (a) may need to issue multiple service calls (like sensor app)
//               (b) may need to separately specify service UUID, vs char UUID entries
//                   as well as sep counts for each !   e.g.
//     srv_ble_xxx = mnet_create_service (net_id, &service_uuid, svc_array_count,
//                                        char_uuids, char_array_count, 0);
//     how are callbacks handled ?
//     must I internally pick apart char_uuids and save pointers to the
//     data area, and then dynamically look up the entry, when a notification
//     comes in ?
//     Or make user handle that part of it, because he passes in the callback addr
//     in the char_uuid entry
//
//     Maybe need a more flexible approach
//         svc_id = mnet_create_service (net_id, &service_uuid, svc_array_count);
//         rc     = mnet_define_char_entry (svc_id, char_uuid, data_ptr, data_length, callback_adddr);
//                    (which gets mapped to associated aci_gatt_xxx call)
//         rc     = mnet_start_service (svc_id);   // to denote end of char definitions

    if (srv_sock_id < 0)
       {
          while (1) ;        // hang for debugger
       }


               /**************************************************************
               *      Poll until we get an incoming BLE client connection.
               **************************************************************/
// VVVVVV   --- this will play out as a GAP callback !!! ---    VVVVVV
    clnt_sock_id = EAGAIN;
    while (clnt_sock_id == EAGAIN)      // loop till get a incoming connection
       clnt_sock_id = mnet_server_accept (srv_sock_id, 0);

    if (clnt_sock_id < 0)
       {       //-----------------------------------------------
               // got an error on the incoming connect/listen
               //-----------------------------------------------
          while (1) ;        // hang for debugger
       }

               /**************************************************************
               *  Wait to receive a single packet of data (MODBUS) from client
               **************************************************************/
// VVVVVV   --- this will play out as a GATT rcv callback !!! ---    VVVVVV
    numc = EAGAIN;                        // default = stll waiting for all data
    while (numc == EAGAIN)                // wait till all data has been rcvd
       numc = mnet_recv (clnt_sock_id, input_msg, sizeof(input_msg), 0);

    if (numc < 0)
       {
          while (1) ;        // hang for debugger
       }

               /**************************************************************
               *      Send a single packet of reply data (MODBUS) to client.
               **************************************************************/
// VVVVVV   --- this will play out as a GATT send call !!! ---    VVVVVV
    memcpy (reply_coil_msg, input_msg, numc);
    nums = mnet_send (clnt_sock_id, reply_coil_msg, numc, 0);

               /**************************************************************
               *          Close the socket, and shutdown network
               **************************************************************/
// VVVVVV   --- will this man ifest as a GAP disconnect callback ??? ---    VVVVVV
     ret_code = mnet_close_socket (sock_id);

     ret_code = mnet_disconnect_network (net_id);

     while (1)
       {          // just hang, so debugger can see we completed OK
       }
#endif                                // defined(LATER)

}



/******************************************************************************
* @brief  Process user input (i.e. pressing the USER button on Nucleo board)
*         and send a LED toggle command to the remote board.
*
* @param  None
* @retval None
******************************************************************************/
    uint8_t data[20] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F','G','H','I','J'};

void  User_Process (void)
{
  if (set_connectable)
     {        //----------------------------------------------------------
              // Setup connection infrastruture to remote devices.
              //  - SERVER: will setup Service Name and start Advertising.
              //            Will get a GAP_ConnectionComplete_CB() callback
              //            when it finally connects to a Client.
              //  - CLIENT: will setup CENTRAL parms and preps connection
              //            logic to connect to specified Advertisers.
              //            Will get a GAP_ConnectionComplete_CB() callback
              //            when it finally connects to an Advertiser.
              //----------------------------------------------------------
       Make_Connection();
       set_connectable = FALSE;
     }

  if (BLE_Role == CLIENT)
     {
       if (connected  &&  ! notification_enabled)
          {
            enableNotification();   // enable Notification callbacks for CLIENTs
          }
     }
       //----------------------------------------------------------------------
       // Check if the user has pushed the button, to force a message to be sent
       //-----------------------------------------------------------------------
  if (BSP_PB_GetState(BUTTON_USER) == RESET)
     {
       while (BSP_PB_GetState(BUTTON_USER) == RESET) ;  // wait till button released

       if (connected && notification_enabled)
          {
                /* Button was pressed - Send a toggle command to the remote device */
            sendData (data, sizeof(data));
          }
     }
}

