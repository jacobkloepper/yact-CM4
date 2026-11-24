#include "uart_driver.h"
#include "link_driver.h"
#include "stm32mp157dxx_cm4.h"
#include "stm32mp1xx_hal_def.h"
#include "stm32mp1xx_hal_uart.h"
#include <inttypes.h>
#include <string.h>

#define MAX_UARTS 4
static uart_driver_t* p_driver_reg[MAX_UARTS];

static uart_driver_t* find_by_instance(USART_TypeDef* p_instance)
{
    for (uint8_t i = 0; i < MAX_UARTS; i++)
    {
        if (p_driver_reg[i] && p_driver_reg[i]->cfg.p_instance == p_instance)
        {
            return p_driver_reg[i];
        }
    }
    return NULL;
}

static uart_driver_t* find_by_handle(UART_HandleTypeDef* h)
{
    for (uint8_t i = 0; i < MAX_UARTS; i++)
    {
        if (p_driver_reg[i] && (&p_driver_reg[i]->huart == h))
        {
            return p_driver_reg[i];
        }
    }
	return NULL;
}

static link_status_t uart_init(link_driver_t* p_link_driver)
{
    uart_driver_t* p_driver = (uart_driver_t*)p_link_driver;

    memset(&p_driver->huart, 0, sizeof(p_driver->huart));
    p_driver->huart.Instance           = p_driver->cfg.p_instance;
    p_driver->huart.Init.BaudRate      = p_driver->cfg.baud;
    p_driver->huart.Init.WordLength    = p_driver->cfg.word_length;
    p_driver->huart.Init.StopBits      = p_driver->cfg.stop_bits;
    p_driver->huart.Init.Parity        = p_driver->cfg.parity;
    p_driver->huart.Init.Mode          = UART_MODE_TX_RX;
    p_driver->huart.Init.HwFlowCtl     = UART_HWCONTROL_NONE;
    p_driver->huart.Init.OverSampling  = UART_OVERSAMPLING_16;

    p_link_driver->tx_done = 0;
    p_link_driver->rx_done = 0;
    p_link_driver->err_flags = 0;

    return (HAL_UART_Init(&p_driver->huart) == HAL_OK) ? LINK_OK : LINK_ERROR;
}

static link_status_t uart_deinit(link_driver_t* p_link_driver)
{
    uart_driver_t* p_driver = (uart_driver_t*)p_link_driver;
    return (HAL_UART_DeInit(&p_driver->huart) == HAL_OK) ? LINK_OK : LINK_ERROR;
}

static link_status_t uart_start_tx(link_driver_t* p_link_driver, const uint8_t* p_buf, uint16_t len)
{
    uart_driver_t* p_driver = (uart_driver_t*)p_link_driver;
    HAL_StatusTypeDef ret = HAL_UART_Transmit_IT(&p_driver->huart, (uint8_t*)p_buf, len);
    return (ret == HAL_OK) ? LINK_OK : (ret == HAL_BUSY) ? LINK_BUSY : LINK_ERROR;
}

static link_status_t uart_start_rx(link_driver_t* p_link_driver, uint8_t* p_buf, uint16_t len)
{
    uart_driver_t* p_driver = (uart_driver_t *)p_link_driver;
    HAL_StatusTypeDef ret = HAL_UART_Receive_IT(&p_driver->huart, p_buf, len);
    return (ret == HAL_OK) ? LINK_OK : (ret == HAL_BUSY) ? LINK_BUSY : LINK_ERROR;
}

static void uart_abort(link_driver_t* p_link_driver)
{
    HAL_UART_Abort(&((uart_driver_t*)p_link_driver)->huart);
}

static const link_driver_ops_t UART_OPS = {
    .init = uart_init,
    .deinit = uart_deinit,
    .start_tx = uart_start_tx,
    .start_rx = uart_start_rx,
    .abort = uart_abort
};

void uart_driver_create(uart_driver_t* p_driver, const uart_cfg_t* p_config)
{
    memset(p_driver, 0, sizeof(*p_driver));
    p_driver->link_driver.p_ops = &UART_OPS;
    p_driver->cfg = *p_config;

    // Add to our uart driver list
    for (uint8_t i = 0; i < MAX_UARTS; i++)
    {
        if (!p_driver_reg[i])
        {
            p_driver_reg[i] = p_driver;
            break;
        }
    }
}

void uart_driver_irq(USART_TypeDef *p_instance)
{
    uart_driver_t* p_driver = find_by_instance(p_instance);
    if (p_driver)
    {
        HAL_UART_IRQHandler(&p_driver->huart);
    }
}

/* HAL callbacks */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* p_handle)
{
    uart_driver_t* p_driver = find_by_handle(p_handle);
    if (p_driver)
    {
        p_driver->link_driver.tx_done = 1;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* p_handle)
{
    uart_driver_t* p_driver = find_by_handle(p_handle);
    if (p_driver)
    {
        p_driver->link_driver.rx_done = 1;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* p_handle)
{
    uart_driver_t* p_driver = find_by_handle(p_handle);
    if (!p_driver)
    {
        return;
    }

    uint32_t e = p_handle->ErrorCode;
    uint32_t f = 0;
    if (e & HAL_UART_ERROR_PE)  f |= LINK_ERR_PARITY;
    if (e & HAL_UART_ERROR_FE)  f |= LINK_ERR_FRAMING;
    if (e & HAL_UART_ERROR_NE)  f |= LINK_ERR_NOISE;
    if (e & HAL_UART_ERROR_ORE) f |= LINK_ERR_OVERRUN;
    if (!f)                     f  = LINK_ERR_OTHER;

    p_driver->link_driver.err_flags |= f;
}
