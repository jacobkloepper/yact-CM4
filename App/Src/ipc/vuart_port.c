#include "vuart_port.h"
#include "web_command_protocol.h"
#include "openamp.h"
#include "virt_uart.h"

static VIRT_UART_HandleTypeDef huart0;

/* OpenAMP cannot send until it has seen a first message from Linux (that is
   how it learns the A7 endpoint address). The web service sends STATUS
   requests as soon as it opens the tty, which satisfies this. */
static volatile uint8_t peer_known;

/* Runs from OPENAMP_check_for_message(), i.e. in the main loop. Keep it short:
   copy the bytes into the command parser and return. */
static void rx_cb(VIRT_UART_HandleTypeDef *huart)
{
    peer_known = 1;
    cmd_rx(huart->pRxBuffPtr, huart->RxXferSize);
}

int vuart_port_init(void)
{
    if (VIRT_UART_Init(&huart0) != VIRT_UART_OK)
        return -1;
    if (VIRT_UART_RegisterCallback(&huart0, VIRT_UART_RXCPLT_CB_ID, rx_cb) != VIRT_UART_OK)
        return -1;
    return 0;
}

void vuart_port_poll(void)
{
    OPENAMP_check_for_message();
}

int cmd_port_write(const uint8_t *data, uint16_t len)
{
    if (!peer_known)
        return -1;
    return (VIRT_UART_Transmit(&huart0, (uint8_t *)data, len) == VIRT_UART_OK) ? 0 : -1;
}