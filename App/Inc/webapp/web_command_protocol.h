#ifndef WEB_COMMAND_PROTOCOL_H
#define WEB_COMMAND_PROTOCOL_H

#include <stdint.h>

/* Line protocol between the A7 web service and this firmware.
 *
 * Threading: cmd_rx() and cmd_poll() must be called from the same context
 * (the main loop). OPENAMP_check_for_message() invokes the receive callback
 * from the main loop, so this holds as wired in vuart_port.c. */

/* Transport hook, implemented by vuart_port.c
   Return 0 if the whole buffer was sent, non-zero to be retried later. */
int  cmd_port_write(const uint8_t* p_data, uint16_t len);

void cmd_init(void);
void cmd_rx(const uint8_t* p_data, uint16_t len);  /* feed received bytes, any chunking */
void cmd_poll(void);                               /* call every main-loop iteration    */

#endif /* WEB_COMMAND_PROTOCOL_H */