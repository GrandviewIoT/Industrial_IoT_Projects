/*******************************************************************************
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

#include "MQTTCC3100.h"

#include "boarddef.h"                   // pull in defs for MCU board being used
#include "mnet_call_api.h"              // pull in defs for common COMM TCP API
#define  NETWORK_TYPE         NET_TYPE_WIFI_TCP

#if defined(MSP430_MCU)
  extern unsigned long  _g_systick_millisecs;
#define  MilliTimer     _g_systick_millisecs
#else
    unsigned long       MilliTimer;
#endif

void  SysTickIntHandler (void) 
{
    MilliTimer++;
}

char  expired (Timer *timer)
{
    long  left;

    left = timer->end_time - MilliTimer;
    return (left < 0);
}


void  countdown_ms (Timer *timer, unsigned int timeout)
{
    timer->end_time = MilliTimer + timeout;
}


void  countdown (Timer *timer, unsigned int timeout)
{
    timer->end_time = MilliTimer + (timeout * 1000);
}


int  left_ms (Timer *timer)
{
    long  left;

    left = timer->end_time - MilliTimer;
    return (left < 0) ? 0 : left;
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
    netcb->mqttread   = cc3100_read;  // pointers to low level TCP I/O rtns
    netcb->mqttwrite  = cc3100_write;
    netcb->disconnect = cc3100_disconnect;

        //-----------------------------------------------------------------
        // Invoke mnet TCP CC3100 driver support to connect to the WiFi AP
        //-----------------------------------------------------------------
    netcb->net_id = mnet_connect_network (NET_TYPE_WIFI_TCP, net_addr_type,
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
int32_t  Connect_Server (Network *n, char *srvr_name, int port)
{
    SlSockAddrIn_t sAddr;
    int            addrSize;
    int            retVal;
    unsigned long  ipAddress;

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

    n->my_socket = sl_Socket (SL_AF_INET,SL_SOCK_STREAM, 0);
    if ( n->my_socket < 0 )
       {
         return -1;   // error (probably max sessions or malloc() issue)
       }
                 //------------------------------------------------------
                 //    Issue TCP connect to the remote server
                 //------------------------------------------------------
    retVal = sl_Connect (n->my_socket, (SlSockAddr_t*) &sAddr, addrSize);
    if ( retVal < 0 )
       {
         sl_Close (n->my_socket); // error occurred when tried to connect to server
         return retVal;           // bail
       }

#if defined(USES_MSPWARE)
        // Initialize the MSP432 ??? SysTick Timer and its interrupt
    SysTick_registerInterrupt (SysTickIntHandler);
    SysTick_setPeriod (48000);
    SysTick_enableModule();
    SysTick_enableInterrupt();
#endif

#if defined(USES_TIVAWARE)
        // Initialize the SysTick Timer and its interrupt
    SysTickPeriodSet (SysCtlClockGet() / 1000);   // 1 millisec interrupts
    SysTickEnable();
    SysTickIntEnable();
    IntMasterEnable();
#endif

    return retVal;
}


//int  TLSConnectNetwork (Network *n, char *srvr_name, int port,   ORIG - IBM Def
int  TLS_Connect_Server (Network *n, char *srvr_name, int port,
                         SlSockSecureFiles_t *certificates, unsigned char sec_method,
                         unsigned int cipher, char server_verify)
{
    SlSockAddrIn_t     sAddr;
    int                addrSize;
    int                retVal;
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

    n->my_socket = sl_Socket (SL_AF_INET,SL_SOCK_STREAM, SL_SEC_SOCKET);
    if (n->my_socket < 0)
       {
         return -1;  // error (probably max sessions or malloc() issue)
       }

    method.secureMethod = sec_method;
    retVal = sl_SetSockOpt (n->my_socket, SL_SOL_SOCKET,
                            SL_SO_SECMETHOD, &method, sizeof(method));
    if (retVal < 0)
       {
         return retVal;
       }

    mask.secureMask = cipher;
    retVal = sl_SetSockOpt (n->my_socket, SL_SOL_SOCKET,
                            SL_SO_SECURE_MASK, &mask, sizeof(mask));
    if (retVal < 0)
       {
         return retVal;
       }

    if (certificates != NULL)
       {
         retVal = sl_SetSockOpt (n->my_socket, SL_SOL_SOCKET, SL_SO_SECURE_FILES,
                                 certificates->secureFiles, sizeof(SlSockSecureFiles_t));
         if (retVal < 0)
            {
              return retVal;
            }
       }
                 //------------------------------------------------------
                 //    Issue TLS secure TCP connect to the remote server
                 //------------------------------------------------------
    retVal = sl_Connect (n->my_socket, (SlSockAddr_t*) &sAddr, addrSize);
    if ( retVal < 0 )
       {
         if (server_verify || retVal != -453)
            {
              sl_Close (n->my_socket);   // connection to remote server failed
              return retVal;             // bail
            }
       }

#if defined(USES_MSPWARE)
        // Initialize the MSP432 ??? SysTick Timer and its interrupt
    SysTick_registerInterrupt (SysTickIntHandler);
    SysTick_setPeriod (48000);
    SysTick_enableModule();
    SysTick_enableInterrupt();
#endif

#if defined(USES_TIVAWARE)
        // Initialize the SysTick Timer and its interrupt
    SysTickPeriodSet (SysCtlClockGet() / 1000);   // 1 millisec interrupts
    SysTickEnable();
    SysTickIntEnable();
    IntMasterEnable();
#endif

    return retVal;
}


int  cc3100_read (Network* n, unsigned char *buffer, int len, int timeout_ms)
{
    SlTimeval_t  timeVal;
    SlFdSet_t    fdset;
    int          rc = 0;
    int          recvLen = 0;
    int          sel_rc;

    SL_FD_ZERO (&fdset);
    SL_FD_SET (n->my_socket, &fdset);

    timeVal.tv_sec  = 0;
    timeVal.tv_usec = timeout_ms * 1000;
    sel_rc = sl_Select (n->my_socket + 1, &fdset, NULL, NULL, &timeVal);
    if (sel_rc == 1)
      {               // we got some rcv traffic. try to read in a packet
         do {
                rc = sl_Recv (n->my_socket, buffer + recvLen, len - recvLen, 0);
                recvLen += rc;
            } while (recvLen < len);
      }
    return recvLen;
}


int  cc3100_write (Network *n, unsigned char *buffer, int len, int timeout_ms)
{
    SlTimeval_t  timeVal;
    SlFdSet_t    fdset;
    int          rc = 0;
    int          readySock;

    SL_FD_ZERO (&fdset);
    SL_FD_SET (n->my_socket, &fdset);

    timeVal.tv_sec = 0;
    timeVal.tv_usec = timeout_ms * 1000;
    do {
           readySock = sl_Select (n->my_socket + 1, NULL, &fdset, NULL, &timeVal);
       } while (readySock != 1);

    rc = sl_Send (n->my_socket, buffer, len, 0);
    return rc;
}


void cc3100_disconnect (Network* n)
{
    sl_Close (n->my_socket);
}

//int publish(Network* n, char* topic, uint8_t* payload, unsigned int plength, bool retained) {
//   if (connected()) {
//      // Leave room in the buffer for header and variable length field
//      uint16_t length = 5;
//      length = writeString(topic,buffer,length);
//      uint16_t i;
//      for (i=0;i<plength;i++) {
//         buffer[length++] = payload[i];
//      }
//      uint8_t header = MQTTPUBLISH;
//      if (retained) {
//         header |= 1;
//      }
//      return cc3100_write(&n, header,buffer,length-5);
//   }
//   return 0;
//}
//
//
//uint16_t writeString(char* string, uint8_t* buf, uint16_t pos) {
//   char* idp = string;
//   uint16_t i = 0;
//   pos += 2;
//   while (*idp) {
//      buf[pos++] = *idp++;
//      i++;
//   }
//   buf[pos-i-2] = (i >> 8);
//   buf[pos-i-1] = (i & 0xFF);
//   return pos;
//}
