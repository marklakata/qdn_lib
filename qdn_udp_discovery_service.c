/*********************************************************************************************************************************
FILE            : server.c
PURPOSE         : UDP server application code
VERSION         : 1.00
LANGUAGE        : IAR C/C++ compiler v5.40
TARGET MACHINE  : STM32F10xxE Cortex Controller
PROGRAMMER      : Gerard Vurens
DATE            : (c) 2011, Gener8 Corporation
REMARKS         : 
*********************************************************************************************************************************/
//#include "hw.h"
#include "g8_udp_discovery_service.h"
#include "project.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"
#include <string.h>


#ifndef UDP_DISCOVERY_SERVER_NAME
#error Please define UDP_DISCOVERY_SERVER_NAME() as a string in project.h, such as #define UDP_DISCOVERY_SERVER_NAME() "LMS_MBD"
#endif
#ifndef UDP_DISCOVERY_PORT
#error Please define the port as a #define UDP_DISCOVERY_PORT 12345
#endif

//*******************************************************************************
// Function Name  : udp_server_callback
// Description    : Called when a data is received on the UDP connection
// Input          : arg	-- the user supplied argument
//                  pcb	-- the UDP Process Control Block that has received the data
//                  p -- the packet buffer
//                  addr -- the remote IP address from which the packet was received
//                  port -- the remote port from which the packet was received
// Return         : None.
// Remarks        : 
// *******************************************************************************
static void udp_server_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, struct ip_addr *addr, u16_t port)
{
  // We have received a packet over the UDP server port
  // Check if this message is the identify command (byte[0] = 17, byte[1] = 2 (identify)
  if((((char *)p->payload)[0] == 17) && (((char *)p->payload)[1] == 2))
  {
    // Connect to the remote client through the supplied port
    udp_connect(upcb, addr, port);
    
    // Tell the client the Product Name
    const char* s = UDP_DISCOVERY_SERVER_NAME();
    strcpy(((char*)p->payload) + 3, s);       
    ((char *)p->payload)[2] = strlen(s);                        // Length byte of string
     p->len = p->tot_len = strlen((char*)p->payload);
    
    udp_send(upcb, p);

    // free the UDP connection, so we can accept new clients
    udp_disconnect(upcb);
  }

  // Free the p buffer
  pbuf_free(p);
	
  // Bind the upcb to IP_ADDR_ANY address and the UDP_PORT port
  // Be ready to get a new request from another client  
  udp_bind(upcb, IP_ADDR_ANY, UDP_DISCOVERY_PORT());
	
  // Set a receive callback for the upcb
  udp_recv(upcb, udp_server_callback, NULL);    	
}

//*******************************************************************************
// Function Name  : server_init
// Description    : Initialize the server application.
// Input          : None.
// Return         : None.
// Remarks        : 
// *******************************************************************************
void UDP_DiscoveryServiceInit(void)
{
   struct udp_pcb *upcb;                                 
   
   // Create a new UDP control block
   upcb = udp_new();
   
   // Bind the upcb to the UDP_PORT port
   // Using IP_ADDR_ANY allow the upcb to be used by any local interface
   udp_bind(upcb, IP_ADDR_ANY, UDP_DISCOVERY_PORT());
   
   // Set a receive callback for the upcb
   udp_recv(upcb, udp_server_callback, NULL);
  
}



