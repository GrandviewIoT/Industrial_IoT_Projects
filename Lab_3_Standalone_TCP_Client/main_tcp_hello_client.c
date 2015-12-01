    int  retry_count = 0;

/********1*********2*********3*********4*********5*********6*********7**********
*
*
*                              tcp_hello_client.c
*
*
*  Simple MCU TCP client to send a fixed "Hello" packet to a remote server,
*  and waits for a simple reply.   (sends/rcvs a simple MODBUS packet of data.)
*
*  It is designed to test/prove that basic connectivity from the MCU to a
*  designated server works.
*
*  This models what the PC/Lunix-based Test app "pingport.c" performs.
*
*  It can be used to test against the PC/Lunix-based Test app "grabport.c",
*  or it can be run against an AB or Siemens PLC running the "PLC_Hello_Server"
*  ladder-logic application.
*
*  Test App: PC/Linux Test Server App usage:   grabport  502  data  trace
*            (when server_port is set to 502)
*
*  CAUTION: if using TI Tiva for any TCP apps, be sure the Linker Stack size
*           is set to 800 (default is 600), otherwise you may get Faults.
*
*  FRAM / CC3100 usage: FRAM Launchpads (like FR5969) require power to
*                _both_ USB connectors (Launchpad and CC3100 connector),
*                else the CC3100 can hang due to insufficient power.
*
*  Grandview DB/DC Systems             DRDA Portable Toolkit
*    Licensed Property of Grandview DC/DC Systems
*    (c) Copyright  Grandview DB/DC Systems  1994, 2002
*    All rights reserved
*
*  History:
*    03/10/15 - Converted over to common TCP API. Duquaine
*    03/14/15 - W5200 (STM32) finally works ! Fixed W5200 timing windows !
*    04/02/15 - Tiva and CC3100 work with new common TCP API. Duq
*    04/06/15 - MSP432 ARM and CC3100 work with new common TCP API. Duq
*    04/11/15 - MSP430-F5529 and CC3100 works with new common TCP API. Duq
*    05/27/15 - MSP430-FR6989 verified that it works (< 24 hours after rcvd board)! Duq
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

#include "user_api.h"                // pull in defs for User API calls
#include "mnet_call_api.h"           // pull in defs for common COMM TCP API
#include "project_config_parms.h"


void  uart_get_config_info(void);    // function prototypes
void  error_terminate(int err_num, char *err_msg);
void  error_report(int err_num, char *err_msg);

               //**************************************************************
               //              Network and TCP server Variables
               //
               // By default, these are pulled in from project_config_parms.h
               //**************************************************************
    int    netwk_type        = NETWORK_TYPE;        // set default config parms
    int    ip_addr_type      = NET_ADDRESS_DEFAULT;
    int    netwk_ap_security = NETWORK_AP_SEC_TYPE;
    char   *netwk_ap_gateway = NETWORK_AP_GATEWAY;  // WiFi SSID or Wired DNS Srvr
    char   *netwk_ap_ssid    = NETWORK_AP_SSID;     // ??? MERGE THESE two ???
    char   *netwk_ap_passwd  = NETWORK_AP_PASSSWD;
    char   *server_id        = REMOTE_SERVER_ID;
    int    server_port       = REMOTE_SERVER_PORT;

    int    console_read_wait = CONSOLE_STARTUP_INPUT_WAIT;


               //**************************************************************
               //              Application  Variables
               //**************************************************************
    int    net_id      = 0;
    int    sock_id     = 0;
    int    numc        = 0;
    int    ret_code    = 0;
    int    saved_error = 0;      // saves the ERRNO for DEBUGGER analysis

    int    quit_flag   = 0;      // 1 = user typed in quit/exit
    char   uart_char   = 0;
    char   uart_cmd_buf [40];    // holds any user input via UART

    char   test_uart1 = 1;

    int    coil_on    = 1;

                         // TCP MBAP Hdr ----------------------> | Actual MBUS Cmd
                         //                                      | Force Single Coil
                         //  -- TID -- Protocol Id  Length   UID | Func  Coil #2   On/Off
 unsigned char  coil_msg [12] = { 0x00,0x01, 0x00,0x00, 0x00,0x06,0x11, 0x05, 0x00,0x02, 0xFF,0x00 };
 unsigned char  reply_buf[12] = { 0, 0, 0, 0, 0, 0, 0 }; // reply should be echo of above

#define  TID_LO_OFFSET    1       /* offset for TID low order byte       */
#define  ON_OFF_OFFSET   10       /* offset into msg buf for ON/OFF flag */



// ??? In future, add UART support to query for a user configurable Server_name
//     and port
/*******************************************************************************
* uart_get_config_info
*
*            read in configuration information for Network and/or server name/id
*            and port number from UART.     (Optionally, also coil_on/off)
*******************************************************************************/
void  uart_get_config_info (void)
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


/*******************************************************************************
*                                   main
*******************************************************************************/

int  main (int argc, char** argv)
{
    int  i;

    sys_Init (0,0);      // init board: setup clocks, GPIOs,... with default clk

#if defined(USES_CONSOLE_WRITE) || defined(USES_CONSOLE)
    CONSOLE_INIT (115200);
#endif

pin_Config (LED1, GPIO_OUTPUT, 0);  // DEBUG
pin_High (LED1);

    if (console_read_wait > 0)
       uart_get_config_info();      // get server info from user via UART

    CONSOLE_WRITE ("\n\rMCU is initialized. Starting connection to Network/AP.\n\r");

               //*************************************************************
               //         Crank up network support
               //*************************************************************
    net_id = mnet_connect_network (netwk_type, ip_addr_type,
                                   netwk_ap_ssid, netwk_ap_passwd,
                                   netwk_ap_security, 0);
    if (net_id < 0)
       error_terminate (errno,"Network startup (e.g. connect to AP) failed. Terminating.\n\r");
    CONSOLE_WRITE ("\n\rSuccessfully connected to the Network/AP.\n\r");

for (i = 1; i < 10; i++)    // DEBUG
{ sys_Delay_Millis (200);
  pin_Toggle (LED1);        // denote got thru connect network
}

               //*************************************************************
               //         Connect to remote server

               // Sometimes takes two tries before see ARPs sent out !
               // Oftentimes, the first connect request never see an ARP issued.
               //  ==> bug/timing window in W5200 ?
               //      what a pain in the ass ...    need updated fiirmware ?
               //
               //    On second try, are seeing 6 ARPs sent out, but no one ever
               //    answers. What the hell is the problem ?  (6 = max retries)
               //    On the PC, it works just fine !!!  TILT
               //    TOTAL BULLSHIT.   JWIFDN !!!
               //
               //    Is the Linksys wired router acting like an ass, and refusing
               //    to provide Gateway services to the static IP address ???
               //
               // Does this happen on Tiva/CC3100 when using WiFi ???
               //*************************************************************
retry_count = 0;
retry_connect:
    ret_code = mnet_connect_server (net_id, server_id, server_port,
                                    MNET_NULL_PTR, &sock_id, 0);

for (i = 1; i < 6; i++)    // DEBUG
{ sys_Delay_Millis (500);
  pin_Toggle (LED1);        // denote got thru connect server
}
    if (sock_id < 0)
       { retry_count++;
         if (retry_count > 5)
            error_terminate (errno," Connect to Remote Server failed. Terminating.\n\r");
            else goto retry_connect;
       }
    CONSOLE_WRITE ("\n\r Successfully connected to the remote TCP Server.\n\r");

               /**************************************************************
               *                        main  loop
               *
               *     Send and receive a single packet of (MODBUS) data,
               *     then do a 1 second pause, and loop do it again.
               *
               *     Note we do a TCP close/re-connect on each pass, because
               *     if we are testing against grabport, etc, it closes the
               *     socket to us after each single TCP send/rcv sequence.
               **************************************************************/
    quit_flag = 0;               // loop till user at console says stop
    while (quit_flag == 0)
     {
      coil_on ^= 1;              // alternately toggle remote coil on/off
      if (coil_on == 1)
         coil_msg [ON_OFF_OFFSET] = 0xFF;          // FF 00 denotes ON
         else coil_msg [ON_OFF_OFFSET] = 0x00;     // 00 00 denotes OFF
      coil_msg [TID_LO_OFFSET] += 1;               // increment TID

               //*************************************************************
               //         Send MODBUS message to remote server
               //*************************************************************
      numc = mnet_send (sock_id, coil_msg, sizeof(coil_msg), 0);

               //*************************************************************
               //         Receive MODBUS reply from remote server
               //*************************************************************
      numc = EAGAIN;                      // default = waiting for end of data
      while (numc == EAGAIN)              // wait till all data has been rcvd
         numc = mnet_recv (sock_id, reply_buf, sizeof(reply_buf), 0);

      if (numc < 0)
         {     // we got an error on receive - probably TCP connection dropped.
           error_report (errno,"Receive of reply from Server failed. Terminating.\n\r");
           quit_flag = 1;                 // denote we should bail out of loop
           continue;                      // force bail out of main loop
         }

      CONSOLE_WRITE ("\n\r Send and Receive of Data to Server completed OK.\n\r");

#if (USES_CONSOLE_READ)
      if (CONSOLE_CHECK_FOR_INPUT())
         {      // we have some console input available. See if it is a Q or q
                // letter to denote quit
           uart_char = CONSOLE_GET_CHAR();
           CONSOLE_WRITE_CHAR (uart_char);  // echo it
           if (uart_char == 'Q' || uart_char == 'q')
              quit_flag = 1;                // denote we should bail out of loop
         }
#endif

           //*************************************************************
           //   Handle server's TCP "close" request that it sends to us.
           //*************************************************************
      ret_code = mnet_close_connection (sock_id, 300); // ensure socket to server closed

      sys_Delay_Millis (1000);             // delay 1 second between transmits

           //*************************************************************
           //   re-connect to MODBUS Server for next pass.
           //*************************************************************
      ret_code = mnet_connect_server (net_id, server_id, server_port,
                                      MNET_NULL_PTR, &sock_id, 0);
      if (sock_id < 0)
         {      // the re-connect failed. Server was probably shutdown
           error_report (errno,"Re-Connect to Remote Server failed. Terminating.\n\r");
           quit_flag = 1;                 // denote we should bail out of loop
           continue;                      // force bail out of main loop
         }
    }                                     //  end  while  (quit_flag == 0)

           //*************************************************************
           //        Close the socket, and shutdown network
           //*************************************************************
     CONSOLE_WRITE ("\n\r Writes to Server completed.\n\r Terminating.\n\r");
     ret_code = mnet_close_connection (sock_id, 1000);   // wait up to 1 sec);

     ret_code = mnet_disconnect_network (net_id);

     while (1)
       {          // just hang, so debugger can see we completed OK
       }
}


/*******************************************************************************
* error_report
*
*           We hit an unexpected error. Report it, then allow graceful shutdown.
*           Save the errno, write out console message.
*******************************************************************************/
void  error_report (int err_num, char *err_msg)
{
    saved_error = err_num;     // save the error number  (for debugger)
    CONSOLE_WRITE (err_msg);
}


/*******************************************************************************
* error_terminate
*
*           We hit an error where we cannot continue (network down,
*           server down, ...)
*           Save the errno, write out console message, and hang/stop execution.
*******************************************************************************/
void  error_terminate (int err_num, char *err_msg)
{
    saved_error = err_num;     // save the error number  (for debugger)
    CONSOLE_WRITE (err_msg);
pin_High (LED1);   // signal a hard stop
    while (1) ;                // stop execution, hang for debugger
}

//******************************************************************************
