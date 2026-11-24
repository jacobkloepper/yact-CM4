#include "spi_driver.h"
#include "link_driver.h"
#include "stm32mp1xx_hal_def.h"
#include "stm32mp1xx_hal_spi.h"
#include <string.h>

#define MAX_SPI 4

static spi_driver_t* p_driver_reg[MAX_SPI];

static spi_driver_t* find_by_instance(SPI_TypeDef* p_instance)
{
    for (uint8_t i = 0; i < MAX_SPI; i++)
    {
        if (p_driver_reg[i] && p_driver_reg[i]->cfg.p_instance == p_instance)
        {
            return p_driver_reg[i];
        }
    }
    return NULL;
}

static spi_driver_t* find_by_handle(SPI_HandleTypeDef* p_handle)
{
    for (uint8_t i = 0; i < MAX_SPI; i++)
    {
        if (p_driver_reg[i] && &p_driver_reg[i]->hspi == p_handle)
        {
            return p_driver_reg[i];
        }
    }
    return NULL;
}

static link_status_t map_status(HAL_StatusTypeDef ret)
{
    return (ret == HAL_OK) ? LINK_OK : (ret == HAL_BUSY) ? LINK_BUSY : LINK_ERROR;
}

static link_status_t spi_init(link_driver_t* p_link_driver)
{
    spi_driver_t* p_driver = (spi_driver_t*)p_link_driver;
    uint8_t master = (p_driver->cfg.role != SPI_ROLE_SLAVE);
    uint8_t hw_nss = p_driver->cfg.hw_nss && (p_driver->cfg.role != SPI_ROLE_LOOPBACK);

    // Fields not set stay zero (their default).
    memset(&p_driver->hspi, 0, sizeof(p_driver->hspi));
    p_driver->hspi.Instance                = p_driver->cfg.p_instance;
    p_driver->hspi.Init.Mode               = master ? SPI_MODE_MASTER : SPI_MODE_SLAVE;
    p_driver->hspi.Init.Direction          = SPI_DIRECTION_2LINES;
    p_driver->hspi.Init.DataSize           = SPI_DATASIZE_8BIT;
    p_driver->hspi.Init.CLKPolarity        = (p_driver->cfg.mode & 2) ? SPI_POLARITY_HIGH : SPI_POLARITY_LOW;
    p_driver->hspi.Init.CLKPhase           = (p_driver->cfg.mode & 1) ? SPI_PHASE_2EDGE : SPI_PHASE_1EDGE;
    p_driver->hspi.Init.NSS                = !hw_nss ? SPI_NSS_SOFT : (master ? SPI_NSS_HARD_OUTPUT : SPI_NSS_HARD_INPUT);
    p_driver->hspi.Init.BaudRatePrescaler  = p_driver->cfg.baud_prescaler;
    p_driver->hspi.Init.FirstBit           = SPI_FIRSTBIT_MSB;
    p_driver->hspi.Init.CRCPolynomial      = 7;

    p_link_driver->tx_done = 0;
    p_link_driver->rx_done = 0;
    p_link_driver->err_flags = 0;
    p_driver->p_lb_rx = NULL;
    p_driver->lb_len = 0;

    // HAL_SPI_MspInit does clocks, GPIO AF, and NVIC. 
    if (HAL_SPI_Init(&p_driver->hspi) != HAL_OK)
    {
        return LINK_ERROR;
    }

    // A software-NSS master must see its internal NSS as inactive or it raises mode fault. SPE = 0 here, so SSI may be written.
    if (master && !hw_nss)
    {
        SET_BIT(p_driver->hspi.Instance->CR1, SPI_CR1_SSI);
    }

    return LINK_OK;
}

static link_status_t spi_deinit(link_driver_t* p_link_driver)
{
    spi_driver_t* p_driver = (spi_driver_t*)p_link_driver;
    return (HAL_SPI_DeInit(&p_driver->hspi) == HAL_OK) ? LINK_OK : LINK_ERROR;
}

static link_status_t spi_start_tx(link_driver_t* p_link_driver, const uint8_t* p_buf, uint16_t len)
{
    spi_driver_t* p_driver = (spi_driver_t*)p_link_driver;
    uint8_t* p_rx;

    if (len > SPI_MAX_FRAME)
    {
        return LINK_ERROR;
    }

    switch (p_driver->cfg.role)
    {
        case SPI_ROLE_MASTER:
            p_rx = p_driver->scratch;            // Discard MISO data.
            break;
        case SPI_ROLE_LOOPBACK:
            if (!p_driver->p_lb_rx || len > p_driver->lb_len)
            {
                return LINK_ERROR;
            }
            p_rx = p_driver->p_lb_rx;
            p_driver->p_lb_rx = NULL;
            break;
        default:
            return LINK_ERROR;          // Slave never initiates a transfer.
    }
    return map_status(HAL_SPI_TransmitReceive_IT(&p_driver->hspi, (uint8_t*)p_buf, p_rx, len));
}

static link_status_t spi_start_rx(link_driver_t* p_link_driver, uint8_t* buf, uint16_t len)
{
    spi_driver_t* p_driver = (spi_driver_t*)p_link_driver;

    if (len > SPI_MAX_FRAME)
    {
        return LINK_ERROR;
    }

    switch (p_driver->cfg.role)
    {
        case SPI_ROLE_SLAVE:
            memset(p_driver->scratch, 0, len);
            return map_status(HAL_SPI_TransmitReceive_IT(&p_driver->hspi, p_driver->scratch, buf, len));
        case SPI_ROLE_LOOPBACK:
            p_driver->p_lb_rx = buf;
            p_driver->lb_len = len;
            return LINK_OK;     // transfer starts in start_tx()
        default:
            return LINK_ERROR;  // master only receives in start_tx()
    }
}

// Reset peripheral. Also re-aligns a slave that lost bit sync.
static void spi_abort(link_driver_t* p_link_driver)
{
    spi_driver_t* p_driver = (spi_driver_t*)p_link_driver;
    HAL_SPI_DeInit(&p_driver->hspi);
    spi_init(p_link_driver);
}

static const link_driver_ops_t SPI_OPS = {
    .init = spi_init,
    .deinit = spi_deinit,
    .start_tx = spi_start_tx,
    .start_rx = spi_start_rx,
    .abort = spi_abort
};

void spi_driver_create(spi_driver_t* p_driver, const spi_cfg_t* p_config)
{
    memset(p_driver, 0, sizeof(*p_driver));
    p_driver->link_driver.p_ops = &SPI_OPS;
    p_driver->cfg = *p_config;
    for (uint8_t i = 0; i < MAX_SPI; i++)\
    {
        if (!p_driver_reg[i])
        {
            p_driver_reg[i] = p_driver;
            break;
        }
    }
}

void spi_driver_irq(SPI_TypeDef* p_instance)
{
    spi_driver_t* p_driver = find_by_instance(p_instance);
    if (p_driver)
    {
        HAL_SPI_IRQHandler(&p_driver->hspi);
    }
}

// HAL callbacks
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* p_handle)
{
    spi_driver_t* p_driver = find_by_handle(p_handle);
    if (!p_driver)
    {
        return;
    }

    switch (p_driver->cfg.role)
    {
        case SPI_ROLE_MASTER:
            p_driver->link_driver.tx_done = 1;
            break;
        case SPI_ROLE_SLAVE:
            p_driver->link_driver.rx_done = 1;
            break;
        case SPI_ROLE_LOOPBACK:
            p_driver->link_driver.tx_done = 1;
            p_driver->link_driver.rx_done = 1;
            break;
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* p_handle)
{
    spi_driver_t* p_driver = find_by_handle(p_handle);
    if (!p_driver)
    {
        return;
    }

    uint32_t e = p_handle->ErrorCode;
    uint32_t f = 0;
    if (e & HAL_SPI_ERROR_FRE)      f |= LINK_ERR_FRAMING;
    if (e & HAL_SPI_ERROR_OVR)      f |= LINK_ERR_OVERRUN;
    if (e & HAL_SPI_ERROR_UDR)      f |= LINK_ERR_UNDERRUN;
    if (e & HAL_SPI_ERROR_MODF)     f |= LINK_ERR_BUS;
    if (!f)                         f = LINK_ERR_OTHER;
    p_driver->link_driver.err_flags |= f;
}