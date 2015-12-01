
        //----------------------------------------------------------------------
        //  ONLY COMPILE THIS IF CONFIGURED FOR BlueNRG BLE  (ST)
        //----------------------------------------------------------------------
#if defined(USES_BLUENRG_BLE)
#define  __BLUENRG_CODE__

//   uint8  IP[4]      = { 62, 9,  48, 15 };         // our local IP Address

/*******************************************************************************
*                                                                     WVD  CODE
*                            ble_bluenrg_driver.c
*
* BLE Driver utilizing a standardized API for use with non-BSD based versions of
* BLE support for MCUs using (ST) based BlueNRG BLE X-Nucleo shields IDB04A1.
*
*
* Overall Usage from "User Application"
* --------------------------------------
*  Client:                 WEAKNESS - can only connect to 1 server at a time ?
*                          OR         is it 1 context per Server ?
*                                     Then how avoid multiple local IP addrs ?
*     netid   = mnet_connect_network (net_type,addr_type,AP_SSID,AP_PWD,AP_security);
*     sockid  = mnet_connect_server (netid, Server_Id, Server_Port, [Service_Name], flags);
*     srvsock = mnet_server_listen (Server_Port, [Service_Name], flags);
*     sockid  = mnet_server_accept (srvsock, flags);
*     leng    = mnet_send (sockid, buf, buf_length, flags);
*     leng    = mnet_recv (sockid, buf, max_length, flags);
*     leng    = mnet_check_for_recv_data (sockid, flags);
*     rc      = mnet_close_socket (sockid);
*     rc      = mnet_disconnect_network (netid);
*     rc      = mnet_select_sockets (rcvfds, txfds, ssfds, flags);
*
* WILL ALSO NEED NETWORK SCAN API too for wiFi and BLE
*
* History:
* --------
*  11/20/14 - Got full send/rcv Modbus client support working using CC3100. D
*  11/29/14 - Got initial version of Modbus server working using Tiva/CC3100. Du
*  12/09/14 - Replaced brain-dead strlcpy() with strncpy(), after it blew up
*             for the Nth time on MSP430 (after having previously randomly
*             blown up on Tiva and STM32). As several observers have said
*             "There is a reason this function is not in any ISO standard."
*  12/19/14 - Move RxFilterIdMask and ver to relieve stack cruch on small
*             memory MCUs. These are rarely used LINK level structs. Duquaine
*  06/20/15 - Ensure EXTI IRQ is _NOT_ enabled _UNTIL_ SPI stuff is all
*             initialized, otherwise can get into a race condition with EXTI
*             interrupts at startup on slower processors (L0, L1). Duq
*  06/20/15 - Increased STACKSIZE to handle startup nesting issues. Duq
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

#include "mnet_call_api.h"            // Defs for common COMM TCP/BLE/MQTT API

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>


    int              debug_spi    = 0;        // SPI DEBUG LOOP for Scope Trace
    unsigned char    tcp_tmp_array[6];        // Debug

void  BlueNRG_GPIO_Init (SPI_HandleTypeDef* hspi);    // DEBUG

//------------------------------------------------------------------------
//
//                   BlueNRG BLE  Specific  Variables
//
//------------------------------------------------------------------------
#define  BDADDR_SIZE   6

    SPI_HandleTypeDef  *BlueNRG_hspi_ptr; // ptr to SPI module used by BlueNRG BLE

              //--------------------------------------------------
              // Configuration Network Information of BLUENRG BLE
              //--------------------------------------------------
extern volatile uint8_t   set_connectable;    // defined in ble_sample_service.c
extern volatile int       connected;          // KEY CONTROL VARIABLES
extern volatile uint8_t   notification_enabled;

//-----------------------------------------------------------------------------
// Uncomment the line corresponding to the role you want to have
//     NOTE:   SERVER = the peripheral performing a function/service (e.g. Heart
//                      Rate monitor). It advertises itself to others.
//                      It acts as a GATT SERVER to publish its data.
//             CLIENT = Central monitor that listens for services/devices
//                      it is interested in talking to. Typically it is a
//                      Smartphone or Tablet.
//                      It acts as a GATT Client, receiving data from GATT Server.
//-----------------------------------------------------------------------------

extern BLE_RoleTypeDef    BLE_Role;        // defined in main_ble_hello_server.c
                                           //    and     main_ble_hello_client.c

extern uint8_t            CLIENT_BDADDR[]; // ditto based on config parms file
extern uint8_t            SERVER_BDADDR[];

    uint8_t            bdaddr [BDADDR_SIZE];
    uint16_t           service_handle,  dev_name_char_handle,  appearance_char_handle;
    int                ret;

                     // CAUTION: you _MUST_ declare these as volatile or
                     // IAR's optimizer will optimize them out on some of the
                     // if (W5200_Status == checks, and cause weird BLE errors
volatile int16_t   BLE_Status2;           // DEBUG ONLY  CRAZY ASS COMPILER
volatile int16_t   BLE_Status;            // are globals to assist debugging


/*******************************************************************************
* mnet_connect_network
*
*            Connect to Wired or WiFi network.
*
*            net_flags dictates the nodes role, e.g. Client/Peripheral,
*                      Sever/Central, ...
*
*            On success, the function returns a non-negative integer of the
*            network id to use.
*            On error, -1 is returned, and errno is set appropriately.
*******************************************************************************/
int  mnet_connect_network (int network_type, int net_addr_type, char *AP_SSID,
                           char *AP_PWD, int AP_security, int net_flags)
{
    int   retVal;
  GPIO_InitTypeDef  GPIO_InitStruct;    // ST LOGIC  TEMP HACK
  extern SPI_HandleTypeDef  SpiHandle;  // ST LOGIC  TEMP HACK

            //-----------------------------------------------
            // setup role as (GATT) Client or (GATT) Server
            //-----------------------------------------------
    if (net_flags == FLAGS_CLIENT_GATT_LISTENER)
       {           // set us up as a GATT Client   (and usually CENTRAL)
         BLE_Role = CLIENT;
       }
      else if (net_flags == FLAGS_SERVER_GATT_ADVERTISER)
              {    // set us up as a GATT Server   (and usually PERIPHERAL)
                 BLE_Role = SERVER;
              }
      else {       // Not a supported configuration (yet)
             errno = EBLE_CONF_NOT_SUPP;
             return (-1);
           }

            //---------------------------------------------------------
            //  Initialize and configure BlueNRG BLE device
            //     - Initialize GPIOs:   RESET, CS, SCLK, MISO, MOSI
            //     - Initialize SPI peripheral
            //     - Initialize IRQ pin and associated NVIC rupt entry
            //---------------------------------------------------------
       //---------------------------------------------------------------
       //  Initialize the GPIO used as the RESET pin for BlueNRG.
       //  We do this first to ensure it is properly reset, so no
       //  spurious interrupts get kicked back by the BlueNRG's IRQ pin.
       //---------------------------------------------------------------
//  board_gpio_pin_config (BlueNRG_RESET_PORT, BlueNRG_RESET_PIN,
//                         GPIO_OUTPUT, GPIO_PULLUP);
    pin_Config (BLE_BLUENRG_RESET, GPIO_OUTPUT, PIN_USE_PULLUP);

    ASSERT_BlueNRG_RESET();     // Ensure RESET pin is asserted to reset the HW.
                                // It is cleared in the BlueNRG_RST() call below

       //-----------------------------------------------------------------
       //  Initialize the GPIO used as the SPI CS Chip Select for BlueNRG
       //-----------------------------------------------------------------
//  board_gpio_pin_config (BlueNRG_CS_PORT, BlueNRG_CS_PIN,
//                         GPIO_OUTPUT, GPIO_PULLUP);
    pin_Config (BLE_BLUENRG_SPI_CS, GPIO_OUTPUT, PIN_USE_PULLUP);

    DEASSERT_CS_BlueNRG();           // Ensure CS is set to NOT active

       //-------------------------------------------------------------
       //  Initialize board's associated SPI support for this device
       //      - Initializes GPIOs for SCLK, MISO, MOSI
       //      - Initializes SPI peripheral
       //-------------------------------------------------------------
    spi_Init_Extended (BLE_BLUENRG_SPI_ID,
                                 SPI_MASTER,
                                 BLE_BLUENRG_SPI_MODE,   // uses mode 0
                                 BLE_SPI_BAUDRATE_25_MHZ,
                                 0,                      // No DMA
                                 &SpiHandle);

    spi_Set_Max_Timeout (BLE_BLUENRG_SPI_ID, BLE_SPI_TIMEOUT_DURATION);

       //----------------------------------------------------------------
       //  Initialize the GPIO used as the IRQ INTERRUPT pin for BlueNRG.
       //  This signals request for service from the the BlueNRG to us.
       //
       //  The HAL_GPIO_EXTI_Callback() ISR handler in bluenrg_interface.c
       //  processes the interrupts from the BlueNRG.
       //
       //  Note: to avoid "early interrupts", this leaves the NVIC IRQ
       //        setup but disabled. _MUST_ issue NVIC Enable for it before
       //        issue any aci_xxx() calls.
       //
       //  To avoid timing races on slower processors, hold off doing this
       //  _UNTIL_ all SPI initizliastion is complete.
       //----------------------------------------------------------------
//     board_irq_pin_config (BlueNRG_IRQ_PORT, BlueNRG_IRQ_PIN,
//                        BlueNRG_IRQ_RISING_MODE, BlueNRG_IRQ_PULLUP_SETTING,
//                        BlueNRG_EXTI_IRQn, 4);
    pin_Config_IRQ_Pin (BLE_BLUENRG_IRQ_PIN, GPIO_RUPT_MODE_RISING, 0, BLE_BLUENRG_IRQn, 4);

// 06/20/15 - L0 goes crazy at this point - nesting/looping EXTI calls in a tight loop. Does NOT occur on ST L0 sampler code !!!!
    HAL_NVIC_EnableIRQ (BLE_BLUENRG_IRQn);    // now enable the BlueNRG IRQ line

    HCI_Init();                // Initialize the BlueNRG HCI support

    BlueNRG_RST();             // Reset BlueNRG hardware and then ENABLE BlueNRG

       //--------------------------------
       //  Setup our 6 byte BLE address
       //--------------------------------
    if (BLE_Role == CLIENT)
       {
         Osal_MemCpy (bdaddr, CLIENT_BDADDR, BDADDR_SIZE);
       }
      else {
             Osal_MemCpy (bdaddr, SERVER_BDADDR, BDADDR_SIZE);
           }

    ret = aci_hal_write_config_data (CONFIG_DATA_PUBADDR_OFFSET,
                                     CONFIG_DATA_PUBADDR_LEN,  // 09/13/15 - F3_03 is dying in this on a TIMEOUT too
                                     bdaddr);    // 06/20/15 L0 Is dying this due to Timeout not seeing a reply!!!  WVD
    if (ret)
       {
//       PRINTF ("Setting BD_ADDR failed.\n");
//       Debug_Write (" Failed to start the network device\n\r");
         errno = EDEVSTART_FAILED;
         return (-1);
       }

    ret = aci_gatt_init();        // Init the GATT (data) component of stack
    if (ret)
       {
//       PRINTF ("GATT_Init failed.\n");
//       Debug_Write (" Failed to start the network device\n\r");
         errno = EDEVSTART_FAILED;
         return (-1);
       }


//-------------------------------------
// -- OR --  do these as Connect logic for client, and Listen logic for Server
    if (BLE_Role == SERVER)     // pass as either Network_Type or as Flags argument
       {                        // Setup our GAP (connection) role as the Peripheral
                                // GATT Server that Advertises its services
         ret = aci_gap_init (GAP_PERIPHERAL_ROLE, &service_handle,
                             &dev_name_char_handle, &appearance_char_handle);
       }
      else {                    // Setup our GAP (connection) role as the Central node
                                // acting as a GATT Client, listening to connect to a device
             ret = aci_gap_init (GAP_CENTRAL_ROLE, &service_handle,
                                 &dev_name_char_handle, &appearance_char_handle);
           }

    if (ret != BLE_STATUS_SUCCESS)
       {
//       PRINTF ("GAP_Init failed.\n");
         errno = EDEVSTART_FAILED;
         return (-1);
       }

                                // Setup GAP connection security to use a fixed 6 byte PIN
    ret = aci_gap_set_auth_requirement (MITM_PROTECTION_REQUIRED,
                                        OOB_AUTH_DATA_ABSENT,
                                        NULL,
                                        7,
                                        16,
                                        USE_FIXED_PIN_FOR_PAIRING,
                                        123456,                  // valid PIN to use for conn
                                        BONDING);
    if (ret == BLE_STATUS_SUCCESS)
       {
//       PRINTF ("BLE Stack Initialized.\n");
       }

    if (BLE_Role == SERVER)
       {        //-----------------------------------------------------------------
                // Setup/Register the Peripheral device's "Service" and "Attribute"
                // UUIDs and characteristics (length, data type, ...).
                //
                // Actual Advertisements won't flow until we hit Make_Connection()
                // logic in main's User_Process() logic.
                //-----------------------------------------------------------------
         ret = Add_Sample_Service();

//       if (ret == BLE_STATUS_SUCCESS)
//          PRINTF ("Service added successfully.\n");
//          else PRINTF ("Error while adding service.\n");

         ret = aci_hal_set_tx_power_level (1, 4);   /* Set output power level */
       }

    if (BLE_Role == CLIENT)
       {        //-------------------------------------------------------------------
                // (Tablet) Client just sits and waits for Advertisements to come in.
                // When it finds one it likes, it sets up a connection to it.
                //-------------------------------------------------------------------
                /* Set output power level */
         ret = aci_hal_set_tx_power_level (1, 4);
       }

    return (401);         // for now, we support just one BlueNRG device/Nucleo
}


/*******************************************************************************
* mnet_connect_server
*
*            Connect to a remote server.
*
*            On success, the function returns a non-negative integer of the
*            socket_id to use.
*            On error, -1 is returned, and errno is set appropriately.
*******************************************************************************/

int  mnet_connect_server (int netid, char *Server_Id, int Server_Port,
                          char *Service_Name, int *socket_id, int flags)
{
    char       *node;
    char       *service;

     BLE_Status = 0;

    if (netid != 401)
       { errno = EINVALID_NETID;
         return (-1);                // Invalid Network Id
       }

    if (Server_Id == MNET_NULL_PTR)
       { errno = EINVALID_SERVER;
         return (-1);                     // No valid Host name or IP Address
       }

    if (Service_Name == MNET_NULL_PTR)
       service = "502";
       else if (Service_Name[0] == 0)
               service = "502";
               else service = Service_Name;

       //------------------------------------------------------------
       //  Ensure DNS was kicked off to extract IP address
       //------------------------------------------------------------
// ??? Check status of DNS FSM, and wait until it says we are Good to go

                      //----------------------------------
                      //  Create a socket on BLUENRG BLE
                      //----------------------------------
// add a search table in future

                      //--------------------------------------------
                      //  Connect to remote BLE Server via BLUENRG
                      //--------------------------------------------

    if (BLE_Status != 0)
       {
//       DEBUG_Write (" [TCP Client]  TCP connection Error \n\r");
         errno = BLE_Status;         // could return ERR_INPROGRESS, or ENOHOST, ...
         return (-1);
       }

    *socket_id = 123;     // we only allow one BLE connection. pass back a const
    return (0);           // denote completed successfully
}


/*******************************************************************************
* mnet_check_for_recv_data
*
*            Check if any data was received on a connection.
*
*            On success, the function returns a non-negative integer of the
*            length of received data waiting.
*            On error, -1 is returned, and errno is set appropriately.
*******************************************************************************/
int  mnet_check_for_recv_data (int sockid, int flags)
{
    unsigned short   RSR_len = 0;

//    RSR_len = getSn_RX_RSR (sockid);

    return (RSR_len);
}


/*******************************************************************************
* mnet_close_connection
*
*            Close a connection.         (to either a server or to a client)
*
*            On success, the function returns a non-negative integer.
*            On error, -1 is returned, and errno is set appropriately.
*******************************************************************************/
int  mnet_close_connection (int sockid)
{
//  disconnect (sockid);              // close the socket on W5200
//  BLE_Status = SOCK_FIN_WAIT;     // denote disconnect started
//  while (BLE_Status != SOCK_CLOSED)
//     BLE_Status = getSn_SR (W5200_SockID);  // wait till disconnect complete

        // We need to ensure that the connection is coompletely closed,
        // otherwise if we do a quick back to back disconnect/connect sequence,
        // we can get strange errors because the socket was not fully closed
        // on the W5200.

    return (1);
}


/*******************************************************************************
* mnet_disconnect_network
*
*            Terminate connection to a Wired or WiFi network.
*
*            On success, the function returns a non-negative integer.
*            On error, -1 is returned, and errno is set appropriately.
*******************************************************************************/
int  mnet_disconnect_network (int netid)
{
    if (netid != 5200)
       { errno = EINVALID_NETID;
         return (-1);                // Invalid Network Id
       }

         // nothing special required for Wired Ethernet ala W5200
    return (1);
}


/*******************************************************************************
* mnet_network_scan
*
*            Scan for available WiFi APs, or Bluetooth BLE devices.
*
*            On success, the function returns a non-negative integer.
*            On error, -1 is returned, and errno is set appropriately.
*******************************************************************************/

int  mnet_network_scan (int network_type, int start_top_flag)
{
       // not applicable to Wired Ethernet ala W5200

    errno = ENET_FUNC_NOTSUP;
    return (-1);
}


/*******************************************************************************
* mnet_network_get_next
*
*            pass back information about the next discovered AP or BLE device.
*
*            On success, the function returns a non-negative integer,
*            and fills in the node_name, length, and security_type.
*            On error, -1 is returned, and errno is set appropriately.
*******************************************************************************/

int  mnet_network_get_next (char **node_name,  int *name_length, int *sec_type,
                            int *type_info)
{
       // not applicable to Wired Ethernet ala W5200

    errno = ENET_FUNC_NOTSUP;
    return (-1);
}


/*******************************************************************************
* mnet_recv
*
*            Read in a received block of data from a connection.
*
*            On success, the function returns a non-negative integer of the
*            actual length of data received.
*            On error, -1 is returned, and errno is set appropriately.
*******************************************************************************/
int  mnet_recv (int sockid, unsigned char *buf, int max_length, int flags)
{
    BLE_Status = 0;

       /* Ensure Rx data is present --  HACK FOR NOW  */
//  W5200_RSR_len = getSn_RX_RSR (sockid);
//  if (W5200_RSR_len >= max_length)
//     { BLE_Status = recv (sockid, buf, max_length);   // read received data
//       return (BLE_Status);
//     }
//   else if (W5200_RSR_len < max_length &&  W5200_RSR_len != -1)
//            return (EAGAIN);     // full length not rcvd yet - try again later

    errno = ELENGTH_MISMATCH;
    return (-1);                // no data (or not enough) in pipe
}


/*******************************************************************************
* mnet_send
*
*            Send a block of data over a connection.
*
*            On success, the function returns a non-negative integer of the
*            actual length of data sent.
*            On error, -1 is returned, and errno is set appropriately.
*******************************************************************************/
int  mnet_send (int sockid, unsigned char *buf, int buf_length, int flags)
{
    BLE_Status = 0;
//  BLE_Status = send (sockid, buf, buf_length, (bool) WINDOWFULL_FLAG_OFF);

    if (BLE_Status >= buf_length)
       return (buf_length);       // denote it all was sent

    errno = ELENGTH_MISMATCH;
    return (-1);                  // denote error instead
}


/*******************************************************************************
* mnet_server_accept
*
*            Accept a incoming client connection, on a server that is listening.
*
*            On success, the function returns a non-negative integer for the
*            socket_id to use.
*            Returns EAGAIN if no client has connected.
*            On error, -1 is returned, and errno is set appropriately.
*******************************************************************************/
int  mnet_server_accept (int srvsock, int flags)
{
    int16_t        newSockID = 0;

    BLE_Status = 0;

                     //-----------------------------------
                     //  try to ACCEPT an Incoming client
                     //-----------------------------------
//  BLE_Status = getSn_SR (srvsock);

//  if (BLE_Status == SOCK_CLOSED
//    || (BLE_Status >= SOCK_FIN_WAIT && BLE_Status <= SOCK_LAST_ACK))
       {    // incoming connection start failed
//       DEBUG_Write (" [TCP Server] Accept connection Error \n\r");
         errno = BLE_Status;
         return (-1);
       }

    return (EAGAIN);                  // denote no connection rcvd yet
}


/*******************************************************************************
* mnet_server_listen
*
*            Listen for incoming client connection, to a server application.
*
*            On success, the function returns a non-negative integer for the
*            socket_id to use.
*            On error, -1 is returned, and errno is set appropriately.
*******************************************************************************/
int  mnet_server_listen (int netid, int Server_Port, char *Service_Name, int flags)
{
    BLE_Status = 0;

    if (netid != 401)
       { errno = EINVALID_NETID;
         return (-1);                // Invalid Network Id
       }

    BLE_Status = Add_Sample_Service();        // BLE Server

    if (BLE_Status == BLE_STATUS_SUCCESS)
       {
//       PRINTF ("Service added successfully.\n");
       }
      else {
//            PRINTF ("Error while adding service.\n");
              errno = EINVALID_NETID;
              return (-1);                // Invalid Network Id
           }

        /* Set output power level */
    BLE_Status = aci_hal_set_tx_power_level (1, 4);


//  listen (W5200_listenSockID); // at this point, go listen for an incoming client

    return (123);
}


#if NOT_USED_BY_BLE
/*******************************************************************************
* mnet_select_sockets
*
*            Check for any activity (rcv, send, listen) on a group of sockets.
*
*            On success, the function returns a non-negative integer.
*            On error, -1 is returned, and errno is set appropriately.
*******************************************************************************/
int  mnet_select_sockets (fd_set rcvfds, fd_set txfds, fd_set ssfds,
                          struct timeval tv, int flags)
{
    int s_rc;

       // real objective of this routine is to see if we have any pending data


//sl_Select (int16_t nfds, SlFdSet_t *readsds, SlFdSet_t *writesds, SlFdSet_t *exceptsds, struct SlTimeval_t *timeout)

    s_rc = 1;

#if (RESEARCH_THIS)
    while ((s_rc = select (ctx->s+1, rfds, NULL, NULL, tv)) == -1)
      {
        if (errno == EINTR)
          {
            if (ctx->debug)
               {
                 fprintf (stderr, "A non blocked signal was caught\n");
               }
                   /* Necessary after an error */
            FD_ZERO (rfds);
            FD_SET (ctx->s, rfds);
          }
         else {
                return -1;
              }
      }

    if (s_rc == 0)
      {
        errno = ETIMEDOUT;
        return -1;
      }
#endif                             // (RESEARCH_THIS)

    return s_rc;
}
#endif                                // NOT_USED_BY_BLE


#if NOT_USED_BY_BLE
/*******************************************************************************
* mnet_socket_flush                                             TCP FLUSH
*
*            Pushes any buffered TCP data out to network
*******************************************************************************/

int  mnet_socket_flush (int sockid)
{
    int  rc;
    int  rc_sum = 0;

#if (NEEDS_RESEARCH)
    do {
            /* Extract any garbage from the socket */
        char  devnull [MODBUS_TCP_MAX_ADU_LENGTH];
#ifndef OS_WIN32
        rc = recv (ctx->s, devnull, MODBUS_TCP_MAX_ADU_LENGTH, MSG_DONTWAIT);
#else
           /* On Win32, it's a bit more complicated to not wait */
        fd_set rfds;
        struct timeval tv;

        tv.tv_sec  = 0;
        tv.tv_usec = 0;
        FD_ZERO (&rfds);
        FD_SET (ctx->s, &rfds);
        rc = select (ctx->s+1, &rfds, NULL, NULL, &tv);
        if (rc == -1)
             {
             return -1;
           }

        if (rc == 1)
           {
                  /* There is data to flush */
             rc = recv (ctx->s, devnull, MODBUS_TCP_MAX_ADU_LENGTH, 0);
           }
#endif                                   // ifndef OS_WIN32
        if (rc > 0)
           {
             rc_sum += rc;
           }
      } while (rc == MODBUS_TCP_MAX_ADU_LENGTH);

#endif                                   // (NEEDS_RESEARCH)

    return rc_sum;
}
#endif                         // NOT_USED_BY_BLE


#if NOT_USED_BY_BLE
/*******************************************************************************
* mnet_set_ipv4_options
*
*            Set TCP options  (for IP 4)
*******************************************************************************/

int  mnet_set_ipv4_options (int sockid)
{
    int  rc;
    int  option;

#if (RESEARCH_THIS)
       /* Set the TCP no delay flag */
       /* SOL_TCP = IPPROTO_TCP */
    option = 1;
    rc = setsockopt (sockid, IPPROTO_TCP, TCP_NODELAY,
                     (const void *) &option, sizeof(int));
    if (rc == -1)
       {
         return -1;
       }

#endif                 /// RESEARCH_THIS

    return 0;
}
#endif                         // NOT_USED_BY_BLE

#endif                         // defined(USES_BLUENRG_BLE)

/******************************************************************************/
