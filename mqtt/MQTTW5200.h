
/********1*********2*********3*********4*********5*********6*********7**********
 * Copyright (c) 2014 IBM Corp.                                   TI MSP432 Port
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
 *    W Duquaine. Modified to allow multiple MCUs (Tiva, MSP432, MSP430)
*******************************************************************************/

#ifndef MQTTW5200_H_
#define MQTTW5200_H_

#include "mnet_call_api.h"          // TCP defs

//#include "simplelink.h"
//#include "netapp.h"
//#include "socket.h"

#include <stdint.h>

#if defined(USES_MSPWARE)
#include "systick.h"
#endif
#if defined(USES_TIVAWARE)
#include "driverlib/systick.h"      // uses Tivaware version of DriverLib
#endif

typedef  void   SlSockSecureFiles_t;     /* Hack for now */

#ifndef NET_ADDRESS_DEFAULT
                                         // net_addressing definitions:  CC3100
#define  NET_ADDRESS_DEFAULT  0          //   Default IP addressing, normally IP4
#define  NET_ADDRESS_IP4      1          //   Specifically use IP4
#define  NET_ADDRESS_IP6      2          //   Specifically use IP6
#endif

#ifndef NET_SEC_AP_DEFAULT
                                         // AP_security definitions:     CC3100
#define  NET_SEC_AP_DEFAULT   0          //   Wired network, AP security not used
#define  NET_SEC_AP_OPEN      0          //   AP is open - no security turned on
#define  NET_SEC_AP_WEP       1          //   AP uses basic WEP
#define  NET_SEC_AP_WPA       2          //   AP uses basic WPA
#define  NET_SEC_AP_WPA2      3          //   AP uses "standard" WPA2
#define  NET_SEC_AP_WPA2_ENT  4          //   AP uses "enterprise" WPA2
#endif

typedef struct Timer  Timer;

struct Timer {
               unsigned long  systick_period;
               unsigned long  end_time;
             };

typedef struct Network  Network;

struct Network
{
    int   net_id;
    int   my_socket;
    int   (*mqttread) (Network*, unsigned char*, int, int);
    int   (*mqttwrite) (Network*, unsigned char*, int, int);
    void  (*disconnect) (Network*);
};

char expired(Timer*);
void countdown_ms(Timer*, unsigned int);
void countdown(Timer*, unsigned int);
int  left_ms(Timer*);

void InitTimer(Timer*);

//void NewNetwork(Network*);                   // ORIG IBM Defs
//_i32 ConnectNetwork(Network*, char*, int);
//int  TLSConnectNetwork(Network*, char*, int, SlSockSecureFiles_t*, unsigned char, unsigned int, char);
int Network_Init (Network *netcb, int net_addr_type,
                  char *AP_SSID,  char *AP_PWD,  int AP_security);
int32_t Connect_Server (Network *n, char *srvr_name, int port);
int  TLS_Connect_Server (Network *n, char *srvr_name, int port,
                         SlSockSecureFiles_t *certificates, unsigned char sec_method,
                         unsigned int cipher, char server_verify);
void Network_Term (Network *n);

int  w5200_read(Network*, unsigned char*, int, int);
int  w5200_write(Network*, unsigned char*, int, int);
void w5200_disconnect(Network*);

#endif /* MQTTW5200_H_ */
