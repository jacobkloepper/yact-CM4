#include "uart_driver.h"
#include "link_driver.h"
#include "stm32mp157dxx_cm4.h"
#include "stm32mp1xx_hal_def.h"
#include "stm32mp1xx_hal_uart.h"
#include <inttypes.h>
#include <string.h>

#define MAX_UARTS 4
static uart_driver_t* s_reg[MAX_UARTS];

static uart_driver_t* find_by_instance(USART_TypeDef* instance)
{
    for (uint8_t i = 0; i < MAX_UARTS; i++)
    {
        if (s_reg[i] && s_reg[i]->cfg.instance == instance)
        {
            return s_reg[i];
        }
    }
    return NULL;
}

static uart_driver_t* find_by_handle(UART_HandleTypeDef* h)
{
    for (uint8_t i = 0; i < MAX_UARTS; i++)
    {
        if (s_reg[i] && (&s_reg[i]->huart == h))
        {
            return s_reg[i];
        }
    }
	return NULL;
}

static link_status_t uart_init(link_driver_t* base)
{
    uart_driver_t* d = (uart_driver_t*)base;

    memset(&d->huart, 0, sizeof(d->huart));
    d->huart.Instance           = d->cfg.instance;
    d->huart.Init.BaudRate      = d->cfg.baud;
    d->huart.Init.WordLength    = d->cfg.word_length;
    d->huart.Init.StopBits      = d->cfg.stop_bits;
    d->huart.Init.Parity        = d->cfg.parity;
    d->huart.Init.Mode          = UART_MODE_TX_RX;
    d->huart.Init.HwFlowCtl     = UART_HWCONTROL_NONE;
    d->huart.Init.OverSampling  = UART_OVERSAMPLING_16;

    base->tx_done = 0;
    base->rx_done = 0;
    base->err_flags = 0;

    return (HAL_UART_Init(&d->huart) == HAL_OK) ? LINK_OK : LINK_ERROR;
}

static link_status_t uart_deinit(link_driver_t* base)
{
    uart_driver_t* d = (uart_driver_t*)base;
    return (HAL_UART_DeInit(&d->huart) == HAL_OK) ? LINK_OK : LINK_ERROR;
}

static link_status_t uart_start_tx(link_driver_t* base, const uint8_t* buf, uint16_t len)
{
    uart_driver_t* d = (uart_driver_t*)base;
    HAL_StatusTypeDef ret = HAL_UART_Transmit_IT(&d->huart, (uint8_t*)buf, len);
    return (ret == HAL_OK) ? LINK_OK : (ret == HAL_BUSY) ? LINK_BUSY : LINK_ERROR;
}

static link_status_t uart_start_rx(link_driver_t* base, uint8_t* buf, uint16_t len)
{
    uart_driver_t *d = (uart_driver_t *)base;
    HAL_StatusTypeDef ret = HAL_UART_Receive_IT(&d->huart, buf, len);
    return (ret == HAL_OK) ? LINK_OK : (ret == HAL_BUSY) ? LINK_BUSY : LINK_ERROR;
}

static void uart_abort(link_driver_t* base)
{
    HAL_UART_Abort(&((uart_driver_t*)base)->huart);
}

static const link_driver_ops_t UART_OPS = {
    .init = uart_init,
    .deinit = uart_deinit,
    .start_tx = uart_start_tx,
    .start_rx = uart_start_rx,
    .abort = uart_abort
};

void uart_driver_create(uart_driver_t* ud, const uart_cfg_t* cfg)
{
    memset(ud, 0, sizeof *ud);
    ud->base.ops = &UART_OPS;
    ud->cfg = *cfg;

    // Add to our uart driver list
    for (uint8_t i = 0; i < MAX_UARTS; i++)
    {
        if (!s_reg[i])
        {
            s_reg[i] = ud;
            break;
        }
    }
}

/* TEMP */
typedef struct {
    uint32_t isr;        /* USART3->ISR on entry                    */
    uint32_t cr1;        /* USART3->CR1 on entry                    */
    uint16_t rx_cnt;     /* huart.RxXferCount on entry              */
    uint16_t rx_cnt_out; /* huart.RxXferCount after the HAL handler */
    uint8_t  rx_state;   /* huart.RxState on entry (0x22 = BUSY_RX) */
    uint8_t  g_state;    /* huart.gState on entry  (0x21 = BUSY_TX) */
} irq_trace_t;

#define IRQ_TRACE_LEN 32
volatile irq_trace_t g_irq_trace[IRQ_TRACE_LEN];
volatile uint32_t    g_irq_n;          /* total interrupts seen */

void uart_driver_irq(USART_TypeDef *instance)
{
    uart_driver_t *u = find_by_instance(instance);
    if (!u) return;

    volatile irq_trace_t *t = NULL;
    if (g_irq_n < IRQ_TRACE_LEN) {
        t = &g_irq_trace[g_irq_n];
        t->isr      = instance->ISR;
        t->cr1      = instance->CR1;
        t->rx_cnt   = u->huart.RxXferCount;
        t->rx_state = (uint8_t)u->huart.RxState;
        t->g_state  = (uint8_t)u->huart.gState;
    }
    g_irq_n++;

    HAL_UART_IRQHandler(&u->huart);

    if (t) t->rx_cnt_out = u->huart.RxXferCount;
}

/* TEMP */
// ^^^^^^^^^^^^^^^ TEMP UNCOMMENT BELOW
//void uart_driver_irq(USART_TypeDef *instance)
//{
//    uart_driver_t* d = find_by_instance(instance);
//    if (d)
//    {
//        HAL_UART_IRQHandler(&d->huart);
//    }
//}

/* HAL callbacks */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* h)
{
    uart_driver_t* d = find_by_handle(h);
    if (d)
    {
        d->base.tx_done = 1;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* h)
{
    uart_driver_t* d = find_by_handle(h);
    if (d)
    {
        d->base.rx_done = 1;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* h)
{
    uart_driver_t* d = find_by_handle(h);
    if (!d)
    {
        return;
    }

    uint32_t e = h->ErrorCode;
    uint32_t f = 0;
    if (e & HAL_UART_ERROR_PE)  f |= LINK_ERR_PARITY;
    if (e & HAL_UART_ERROR_FE)  f |= LINK_ERR_FRAMING;
    if (e & HAL_UART_ERROR_NE)  f |= LINK_ERR_NOISE;
    if (e & HAL_UART_ERROR_ORE) f |= LINK_ERR_OVERRUN;
    if (!f)                     f  = LINK_ERR_OTHER;

    d->base.err_flags |= f;
}
