
/********1*********2*********3*********4*********5*********6*********7**********
*
*
*                              tcp_hello_server.c
*
*
*  Simple MCU TCP Server to listen for a incoming client connect, receive a
*  fixed "Hello" packet from the client, and send back a simply reply to
*  the client.   (rcvs/sends a simple MODBUS packet of data.)
*
*  It is designed to test/prove that basic connectivity from the MCU to
*  any incoming client works.
*
*  This models what the PC/Lunix-based Test app "grabport.c" performs.
*
*  It can be used to test against the PC/Lunix-based Test app "pingport.c",
*  or it can be run against an AB or Siemens PLC running the "PLC_Hello_Client"
*  ladder-logic application.
*
*  CAUTION: if using TI Tiva for any TCP apps, be sure the Linker Stack size
*           is set to 800 (default is 600), otherwise you may get Faults.
*
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
*    03/14/15 - Converted over to common TCP API and got working. Duquaine
*    03/14/15 - W5200 (STM32) finally works ! Fixed W5200 timing windows !
*    04/02/15 - Tiva and CC3100 work with new common TCP API. Duq
*    04/06/15 - MSP432 ARM and CC3100 work with new common TCP API. Duq
*    04/11/15 - MSP430-F5529 and CC3100 works with new common TCP API. Duq
*    05/27/15 - MSP430-FR6989 verified that it works (< 24 hours after rcvd board)! Duq
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

#include "user_api.h"                // pull in defs for User API calls
#include "device_config_common.h"
#include "mnet_call_api.h"           // pull in defs for common COMM TCP API

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
    char   *netwk_ap_ssid    = NETWORK_AP_SSID;
    char   *netwk_ap_passwd  = NETWORK_AP_PASSSWD;
    char   *server_id        = REMOTE_SERVER_ID;
    int    server_port       = REMOTE_SERVER_PORT;

    int    console_read_wait = CONSOLE_STARTUP_INPUT_WAIT;

               //**************************************************************
               //              Application  Variables
               //**************************************************************
    int    net_id       = 0;
    int    srv_sock_id  = 0;
    int    clnt_sock_id = 0;

    int    numc        = 0;
    int    nums        = 0;
    int    ret_code    = 0;
    int    saved_error = 0;

    long   num_incoming_connects = 0;   // tracks total # client connects

    int    quit_flag   = 0;      // 1 = user typed in quit/exit
    char   uart_char   = 0;

    int    coil_on     = 1;

 unsigned char  input_msg [12] =       { 0, 0, 0, 0, 0, 0, 0 };  /* reply should be echo of this */
 unsigned char  reply_coil_msg [12]  = { 0x00,0x01, 0x00,0x00, 0x00,0x06,0x11, 0x05, 0x00,0x02, 0xFF,0x00 };
                                      /* TCP MBAP Hdr ----------------------> | Actual MBUS Cmd  */
                                      /*                                      | Force Single Coil */
                                      /*  -- TID -- Protocol Id  Length   UID | Func  Coil #2    On/Off */

#define  TID_LO_OFFSET    1           /* offset for TID low order byte       */
#define  ON_OFF_OFFSET   10           /* offset into msg buf for ON/OFF flag */



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
    sys_Init (0, 0);     // init board: setup clocks, GPIOs,... with default clk

    pin_Config (LED1, GPIO_OUTPUT, 0);

    if (console_read_wait > 0)
       uart_get_config_info();      // get server info from user via UART

    CONSOLE_WRITE ("\n\rMCU is initialized. Starting connection to Network/AP.\n\r");

               //************************************************************
               //         Crank up network support
               //************************************************************
    net_id = mnet_connect_network (netwk_type, ip_addr_type,
                                   netwk_ap_ssid, netwk_ap_passwd,
                                   netwk_ap_security, 0);
    if (net_id < 0)
       error_terminate (errno,"Network startup (e.g. connect to AP) failed. Terminating.\n\r");
    CONSOLE_WRITE ("\n\rSuccessfully connected to the Network/AP.\n\r");

               //************************************************************
               //    Set us up as a Server to LISTEN on our assigned port.
               //************************************************************
    srv_sock_id = mnet_server_listen (net_id, server_port, server_id, 0);
    if (srv_sock_id < 0)
       error_terminate (errno,"Listen on local port failed. Terminating.\n\r");
    CONSOLE_WRITE ("Server Listen completed OK. Waiting for incoming client.\n\r");

               /***************************************************************
               *                          main  loop
               *
               *     Receive a single packet of data (MODBUS) and echo a reply,
               *     then do a 1 second pause, and loop do it again.
               *
               *     Note we do a TCP close/re-accept on each pass, because
               *     if we are testing against pingport, etc it closes the
               *     socket to us after each single TCP send/rcv sequence.
               ***************************************************************/
    quit_flag = 0;               // loop till user at console says stop
    while (quit_flag == 0)
     {
           //*************************************************************
           //    Poll until we get a NEW incoming TCP client connection.
           //*************************************************************
      clnt_sock_id = EAGAIN;
      while (clnt_sock_id == EAGAIN)      // loop till get a incoming connection
        {
          clnt_sock_id = mnet_server_accept (srv_sock_id, 0);
          sys_Delay_Millis  (1);          // Slight delay between conn checks
        }

      if (clnt_sock_id < 0)
         {      //----------------------------------------------------------
                // Got an error on the incoming connect/listen.
                // Probably a network crash (AP dropped, cable modem dropped,...)
                //----------------------------------------------------------
           error_report (errno,"Listen/Accept for Incoming client failed. Terminating.\n\r");
           quit_flag = 1;
           continue;                       // force bail out of main loop
         }
      CONSOLE_WRITE ("Incoming client connected OK. Waiting for Data packet.\n\r");

      num_incoming_connects++;      // inc total number of connects we have seen

           //***************************************************************
           //  Wait to receive a single packet of data (MODBUS) from client
           //***************************************************************
      numc = EAGAIN;                       // default = wait for end of data
      while (numc == EAGAIN)               // wait till all data has been rcvd
         numc = mnet_recv (clnt_sock_id, input_msg, sizeof(input_msg), 0);

      if (numc < 0)
         {     // we got an error on receive - probably TCP connection dropped.
           error_report (errno,"Receive of data from Client failed. Terminating.\n\r");
           quit_flag = 1;
           continue;                       // force bail out of main loop
         }

      if (input_msg[ON_OFF_OFFSET] = 0xFF)  // process Coil On/Off setting
         { pin_High (LED1); }               // 0xFF = ON, so turn on "coil"
         else { pin_Low (LED1); }           // otherwise turn off the "coil"

           //*************************************************************
           //     Send a single packet of reply (MODBUS) data to client.
           //*************************************************************
      memcpy (reply_coil_msg, input_msg, numc);
      nums = mnet_send (clnt_sock_id, reply_coil_msg, numc, 0);

           //*************************************************************
           //     Close the session's socket
           //*************************************************************
       CONSOLE_WRITE (" Receive of Data and Reply to Client completed OK.\n\r");
       sys_Delay_Millis  (100);    // Do a delay before close, else slow clients
                                   // can lose the last few bytes of data
       ret_code = mnet_close_connection (clnt_sock_id);

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

     }                                      //  end  while  (quit_flag == 0)

           //*************************************************************
           //     User wants to shutdown server, and shutdown network
           //*************************************************************
     ret_code = mnet_disconnect_network (net_id);

     CONSOLE_WRITE (" Terminating.\n\r");
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
    while (1) ;                // stop execution, hang for debugger
}

//******************************************************************************
