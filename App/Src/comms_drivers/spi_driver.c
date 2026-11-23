#include "spi_driver.h"
#include "link_driver.h"
#include "stm32mp1xx_hal_def.h"
#include "stm32mp1xx_hal_spi.h"
#include <string.h>

#define MAX_SPI 4

static spi_driver_t* s_reg[MAX_SPI];

static spi_driver_t* find_by_instance(SPI_TypeDef* instance)
{
    for (uint8_t i = 0; i < MAX_SPI; i++)
    {
        if (s_reg[i] && s_reg[i]->cfg.instance == instance)
        {
            return s_reg[i];
        }
    }
    return NULL;
}

static spi_driver_t* find_by_handle(SPI_HandleTypeDef* h)
{
    for (uint8_t i = 0; i < MAX_SPI; i++)
    {
        if (s_reg[i] && &s_reg[i]->hspi == h)
        {
            return s_reg[i];
        }
    }
    return NULL;
}

static link_status_t map_status(HAL_StatusTypeDef ret)
{
    return (ret == HAL_OK) ? LINK_OK : (ret == HAL_BUSY) ? LINK_BUSY : LINK_ERROR;
}

static link_status_t spi_init(link_driver_t* base)
{
    spi_driver_t* d = (spi_driver_t*)base;
    uint8_t master = (d->cfg.role != SPI_ROLE_SLAVE);
    uint8_t hw_nss = d->cfg.hw_nss && (d->cfg.role != SPI_ROLE_LOOPBACK);

    // Fields not set stay zero (their default).
    memset(&d->hspi, 0, sizeof(d->hspi));
    d->hspi.Instance                = d->cfg.instance;
    d->hspi.Init.Mode               = master ? SPI_MODE_MASTER : SPI_MODE_SLAVE;
    d->hspi.Init.Direction          = SPI_DIRECTION_2LINES;
    d->hspi.Init.DataSize           = SPI_DATASIZE_8BIT;
    d->hspi.Init.CLKPolarity        = (d->cfg.mode & 2) ? SPI_POLARITY_HIGH : SPI_POLARITY_LOW;
    d->hspi.Init.CLKPhase           = (d->cfg.mode & 1) ? SPI_PHASE_2EDGE : SPI_PHASE_1EDGE;
    d->hspi.Init.NSS                = !hw_nss ? SPI_NSS_SOFT : (master ? SPI_NSS_HARD_OUTPUT : SPI_NSS_HARD_INPUT);
    d->hspi.Init.BaudRatePrescaler  = d->cfg.baud_prescaler;
    d->hspi.Init.FirstBit           = SPI_FIRSTBIT_MSB;
    d->hspi.Init.CRCPolynomial      = 7;

    base->tx_done = 0;
    base->rx_done = 0;
    base->err_flags = 0;
    d->lb_rx = NULL;
    d->lb_len = 0;

    // HAL_SPI_MspInit does clocks, GPIO AF, and NVIC. 
    if (HAL_SPI_Init(&d->hspi) != HAL_OK)
    {
        return LINK_ERROR;
    }

    // A software-NSS master must see its internal NSS as inactive or it raises mode fault. SPE = 0 here, so SSI may be written.
    if (master && !hw_nss)
    {
        SET_BIT(d->hspi.Instance->CR1, SPI_CR1_SSI);
    }

    return LINK_OK;
}

static link_status_t spi_deinit(link_driver_t* base)
{
    spi_driver_t* d = (spi_driver_t*)base;
    return (HAL_SPI_DeInit(&d->hspi) == HAL_OK) ? LINK_OK : LINK_ERROR;
}

static link_status_t spi_start_tx(link_driver_t* base, const uint8_t* buf, uint16_t len)
{
    spi_driver_t* d = (spi_driver_t*)base;
    uint8_t* rx;

    if (len > SPI_MAX_FRAME)
    {
        return LINK_ERROR;
    }

    switch (d->cfg.role)
    {
        case SPI_ROLE_MASTER:
            rx = d->scratch;            // Discard MISO data.
            break;
        case SPI_ROLE_LOOPBACK:
            if (!d->lb_rx || len > d->lb_len)
            {
                return LINK_ERROR;
            }
            rx = d->lb_rx;
            d->lb_rx = NULL;
            break;
        default:
            return LINK_ERROR;          // Slave never initiates a transfer.
    }
    return map_status(HAL_SPI_TransmitReceive_IT(&d->hspi, (uint8_t*)buf, rx, len));
}

static link_status_t spi_start_rx(link_driver_t* base, uint8_t* buf, uint16_t len)
{
    spi_driver_t* d = (spi_driver_t*)base;

    if (len > SPI_MAX_FRAME)
    {
        return LINK_ERROR;
    }

    switch (d->cfg.role)
    {
        case SPI_ROLE_SLAVE:
            memset(d->scratch, 0, len);
            return map_status(HAL_SPI_TransmitReceive_IT(&d->hspi, d->scratch, buf, len));
        case SPI_ROLE_LOOPBACK:
            d->lb_rx = buf;
            d->lb_len = len;
            return LINK_OK;     // transfer starts in start_tx()
        default:
            return LINK_ERROR;  // master only receives in start_tx()
    }
}

// Reset peripheral. Also re-aligns a slave that lost bit sync.
static void spi_abort(link_driver_t* base)
{
    spi_driver_t* d = (spi_driver_t*)base;
    HAL_SPI_DeInit(&d->hspi);
    spi_init(base);
}

static const link_driver_ops_t SPI_OPS = {
    .init = spi_init,
    .deinit = spi_deinit,
    .start_tx = spi_start_tx,
    .start_rx = spi_start_rx,
    .abort = spi_abort
};

void spi_driver_create(spi_driver_t* d, const spi_cfg_t* cfg)
{
    memset(d, 0, sizeof(*d));
    d->base.ops = &SPI_OPS;
    d->cfg = *cfg;
    for (uint8_t i = 0; i < MAX_SPI; i++)\
    {
        if (!s_reg[i])
        {
            s_reg[i] = d;
            break;
        }
    }
}

void spi_driver_irq(SPI_TypeDef* instance)
{
    spi_driver_t* d = find_by_instance(instance);
    if (d)
    {
        HAL_SPI_IRQHandler(&d->hspi);
    }
}

// HAL callbacks
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* h)
{
    spi_driver_t* d = find_by_handle(h);
    if (!d)
    {
        return;
    }

    switch (d->cfg.role)
    {
        case SPI_ROLE_MASTER:
            d->base.tx_done = 1;
            break;
        case SPI_ROLE_SLAVE:
            d->base.rx_done = 1;
            break;
        case SPI_ROLE_LOOPBACK:
            d->base.tx_done = 1;
            d->base.rx_done = 1;
            break;
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* h)
{
    spi_driver_t* d = find_by_handle(h);
    if (!d)
    {
        return;
    }

    uint32_t e = h->ErrorCode;
    uint32_t f = 0;
    if (e & HAL_SPI_ERROR_FRE)      f |= LINK_ERR_FRAMING;
    if (e & HAL_SPI_ERROR_OVR)      f |= LINK_ERR_OVERRUN;
    if (e & HAL_SPI_ERROR_UDR)      f |= LINK_ERR_UNDERRUN;
    if (e & HAL_SPI_ERROR_MODF)     f |= LINK_ERR_BUS;
    if (!f)                         f = LINK_ERR_OTHER;
    d->base.err_flags |= f;
}