
/********1*********2*********3*********4*********5*********6*********7**********
 * Copyright (c) 2014 IBM Corp.                                 from MSP432 Port
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution.
 *
 * The Eclipse Public License is available at
 *    http://www.eclipse.org/legal/epl-v10.html
 * and the Eclipse Distribution License is available at
 *   http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *    Allan Stockdill-Mander - initial API and implementation and/or initial documentation
 *    E. Chen - Modified for CC3100
 *    W. Duquaine - Modified to work with mnet API support, to hide CC3100
 *           details from the User App.  Also tweaked 3 API calls (NewNetwork,
 *           ConnectNetwork, TLSConnectNetwork) to push down CC3100 details.
 *           I also wasn't thrilled with their names, since ConnectNetwork is NOT
 *           indicative that you are actually connecting to a server.    04/04/15
 *           Since there are already about 3 different flavors of the MQTT TCP
 *           start up  APIs floating around, I do NOT feel too guilt ridden
 *           about defining this new (IMHO) better set.
 *******************************************************************************/

#include "user_api.h"
#include "boarddef.h"                   // pull in defs for MCU board being used

#include "MQTTW5200.h"

#include "mnet_call_api.h"              // pull in defs for common COMM TCP API
#define  NETWORK_TYPE         NET_TYPE_WIRED_TCP

unsigned long    MilliTimer;
//extern uint32_t  uwTick;


#if defined(STM32_MCU)
          // HAL_IncTick is implemented in ~/STM32_Bds/STM32_xx/board_xx.c module for each STM32
#else
void  HAL_IncTick (void)
//void  SysTickIntHandler (void)     name for Tiva/MSP43x version in MQTTCC3100
{
        // This replaces the "weak" default that STM32 supplies

        // STM32 depends upon HAL_IncTick() ISR and associated SYSTICK logic
        // in HAL Lib's stm32f4xx_hal.c code.
        // It updates uwTick on every Systick interrupt.

    MilliTimer++;
}
#endif


#if defined(STM32_MCU)
          // HAL_GetTick is implemented in ~/STM32_Bds/STM32_xx/board_xx.c module for each STM32
#else
uint32_t  HAL_GetTick (void)
{
        // This replaces the "weak" default that STM32 supplies in stm32fnxx_hal.c
    return (MilliTimer);
}
#endif


void  HAL_Delay (__IO uint32_t Delay)
{
  uint32_t  tickstart = 0;

  tickstart = HAL_GetTick();
  while ((HAL_GetTick() - tickstart) < Delay)
    {   // loop till Systick value reaches the ending time
    }
}


char  expired (Timer *timer)
{
    long   amt_left;

/// MilliTimer = uwTick;                       // WVD Change

    amt_left = timer->end_time - MilliTimer;
    return (amt_left < 0);
}


void  countdown_ms (Timer *timer, unsigned int timeout)
{
//  MilliTimer = uwTick;                       // WVD Change

    timer->end_time = MilliTimer + timeout;
}


void  countdown (Timer *timer, unsigned int timeout)
{
//  MilliTimer = uwTick;                       // WVD Change

    timer->end_time = MilliTimer + (timeout * 1000);
}


int  left_ms (Timer *timer)
{
    long   amt_left;

//  MilliTimer = uwTick;                       // WVD Change

    amt_left = timer->end_time - MilliTimer;
    return (amt_left < 0) ? 0 : amt_left;
}


void  InitTimer (Timer *timer)
{
    timer->end_time = 0;
}


//void NewNetwork (Network *netcb)       ORIG - IBM Def
int  Network_Init (Network *netcb, int net_addr_type,
                   char *AP_SSID,  char *AP_PWD,  int AP_security)
{
    netcb->my_socket  = 0;
    netcb->mqttread   = w5200_read;  // pointers to low level TCP I/O rtns
    netcb->mqttwrite  = w5200_write;
    netcb->disconnect = w5200_disconnect;

        //-----------------------------------------------------------------
        // Invoke mnet TCP W5200 driver support to connect to the WiFi AP
        //-----------------------------------------------------------------
    netcb->net_id = mnet_connect_network (NET_TYPE_WIRED_TCP, net_addr_type,
                                          AP_SSID, AP_PWD,
                                          AP_security, 0);
    if (netcb->net_id < 0)
       {
          return (netcb->net_id);    // error - could not connect to WiFi AP
       }
    return (netcb->net_id);          // is positive - so everything went OK
}


void  Network_Term (Network *netcb)
{
        // gracefully shutdown the AP connection, otherwise the CC3100
        // can sometimes get confused if not cleanly brought down.
    mnet_disconnect_network (netcb->net_id);
}


//_i32  ConnectNetwork (Network *n, char *srvr_name, int port)   ORIG - IBM Def
int32_t  Connect_Server (Network *netcb, char *srvr_name, int port)
{
//  SlSockAddrIn_t sAddr;
    int            addrSize;
    int            retVal;
    unsigned long  ipAddress;

    retVal = mnet_connect_server (netcb->net_id, srvr_name, port,
                                  MNET_NULL_PTR, &netcb->my_socket, 0);
    if (netcb->my_socket < 0)
       return (netcb->my_socket);   // we had error on Connect

    return (0);                     // connect to server succeeded

#if TIVA_CC3100_LOGIC
                 //------------------------------------------------------
                 //  Invoke DNS to lookup the IP Address of this Server.
                 //
                 //  This will accept either a name, or a string version
                 //  of an IP address.
                 //   - For a name it will do the DNS lookup and return
                 //     the associated IP address in &ipAddress.
                 //   - for a string IP address, it will convert it to
                 //     binary and return it in &ipAddress.
                 //------------------------------------------------------
    retVal = sl_NetAppDnsGetHostByName (srvr_name, strlen(srvr_name),
                                        &ipAddress, AF_INET);
    if (retVal < 0)
       {
         return -1;                // DNS could not find that Server
       }

    sAddr.sin_family = AF_INET;
    sAddr.sin_port   = sl_Htons ((unsigned short) port);
    sAddr.sin_addr.s_addr = sl_Htonl (ipAddress);

    addrSize = sizeof(SlSockAddrIn_t);

    netcb->my_socket = sl_Socket (SL_AF_INET,SL_SOCK_STREAM, 0);
    if (netcb->my_socket < 0)
       {
         return -1;   // error (probably max sessions or malloc() issue)
       }
                 //------------------------------------------------------
                 //    Issue TCP connect to the remote server
                 //------------------------------------------------------
    retVal = sl_Connect (netcb->my_socket, (SlSockAddr_t*) &sAddr, addrSize);
    if ( retVal < 0 )
       {
         sl_Close (netcb->my_socket); // error when tried to connect to server
         return retVal;               // bail
       }

       // STM32 depends upon HAL_IncTick() +associated SYSTICK logic in HAL Lib.

#if defined(USES_TIVAWARE)
        // Initialize the SysTick Timer and its interrupt
    SysTickPeriodSet (SysCtlClockGet() / 1000);   // 1 millisec interrupts
    SysTickEnable();
    SysTickIntEnable();
    IntMasterEnable();
#endif

    return (retVal);
#endif                              // TIVA_CC3100_LOGIC
}


//int  TLSConnectNetwork (Network *n, char *srvr_name, int port,   ORIG - IBM Def
int  TLS_Connect_Server (Network *netcb, char *srvr_name, int port,
                         SlSockSecureFiles_t *certificates, unsigned char sec_method,
                         unsigned int cipher, char server_verify)
{
    int                retVal = 0;

#if ADD_TLS_IN_FUTURE
    SlSockAddrIn_t     sAddr;
    int                addrSize;
    unsigned long      ipAddress;
    SlSockSecureMethod method;
    SlSockSecureMask   mask;

                 //------------------------------------------------------
                 //  Invoke DNS to lookup the IP Address of this Server
                 //------------------------------------------------------
    retVal = sl_NetAppDnsGetHostByName (srvr_name, strlen(srvr_name),
                                        &ipAddress, AF_INET);
    if (retVal < 0)
       {
         return -1;                // DNS could not find that Server
       }

    sAddr.sin_family = AF_INET;
    sAddr.sin_port   = sl_Htons ((unsigned short) port);
    sAddr.sin_addr.s_addr = sl_Htonl (ipAddress);

    addrSize = sizeof (SlSockAddrIn_t);

    netcb->my_socket = sl_Socket (SL_AF_INET,SL_SOCK_STREAM, SL_SEC_SOCKET);
    if (netcb->my_socket < 0)
       {
         return -1;  // error (probably max sessions or malloc() issue)
       }

    method.secureMethod = sec_method;
    retVal = sl_SetSockOpt (netcb->my_socket, SL_SOL_SOCKET,
                            SL_SO_SECMETHOD, &method, sizeof(method));
    if (retVal < 0)
       {
         return retVal;
       }

    mask.secureMask = cipher;
    retVal = sl_SetSockOpt (netcb->my_socket, SL_SOL_SOCKET,
                            SL_SO_SECURE_MASK, &mask, sizeof(mask));
    if (retVal < 0)
       {
         return retVal;
       }

    if (certificates != NULL)
       {
         retVal = sl_SetSockOpt (netcb->my_socket, SL_SOL_SOCKET, SL_SO_SECURE_FILES,
                                 certificates->secureFiles, sizeof(SlSockSecureFiles_t));
         if (retVal < 0)
            {
              return retVal;
            }
       }
                 //------------------------------------------------------
                 //    Issue TLS secure TCP connect to the remote server
                 //------------------------------------------------------
    retVal = sl_Connect (netcb->my_socket, (SlSockAddr_t*) &sAddr, addrSize);
    if (retVal < 0)
       {
         if (server_verify || retVal != -453)
            {
              sl_Close (netcb->my_socket); // connection to remote server failed
              return retVal;               // bail
            }
       }

        // STM32 depends upon HAL_IncTick() and associated SYSTICK logic in HAL Lib.
#if defined(USES_TIVAWARE)
        // Initialize the SysTick Timer and its interrupt
    SysTickPeriodSet (SysCtlClockGet() / 1000);   // 1 millisec interrupts
    SysTickEnable();
    SysTickIntEnable();
    IntMasterEnable();
#endif

#endif                            // ADD_TLS_IN_FUTURE

    return retVal;
}


int  w5200_read (Network* netcb, unsigned char *buffer, int len, int timeout_ms)
{
    struct timeval timeVal;
    fd_set         fdset;
    int            rc = 0;
    int            recvLen = 0;
    int            sel_rc;

    recvLen = mnet_check_for_recv_data (netcb->my_socket, 0);

       // we got some rcv traffic. try to read in a packet
    if (recvLen > 0)
       {
         recvLen = mnet_recv (netcb->my_socket, buffer, len, 0);
       }
      else return (0);    // we have not rcvd any data on this socket

    return (recvLen);
}


int  w5200_write (Network *netcb, unsigned char *buffer, int len, int timeout_ms)
{
    int          rc;

    rc = mnet_send (netcb->my_socket, buffer, len, 0);

    return (rc);
}


void w5200_disconnect (Network *netcb)
{
    mnet_close_connection (netcb->my_socket);

    netcb->my_socket = -1;               // denote socket is closed/invalid
}
