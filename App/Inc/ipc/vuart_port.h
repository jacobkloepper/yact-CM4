#ifndef VUART_PORT_H
#define VUART_PORT_H

/* OpenAMP virtual UART transport for web_command_protocol. Appears on the A7 as
   /dev/ttyRPMSG0 once vuart_port_init() has run. */

/* Call once, after MX_IPCC_Init() and MX_OPENAMP_Init(). Returns 0 on success. */
int  vuart_port_init(void);

/* Call every main-loop iteration: services the mailbox and delivers any
   received bytes to cmd_rx(). */
void vuart_port_poll(void);

#endif /* VUART_PORT_H */