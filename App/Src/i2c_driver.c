#include "i2c_driver.h"
#include "link_driver.h"
#include "stm32mp157dxx_cm4.h"
#include "stm32mp1xx_hal_def.h"
#include "stm32mp1xx_hal_i2c.h"
#include "stm32mp1xx_hal_i2c_ex.h"
#include <string.h>

#define MAX_I2C 4
static i2c_driver_t* s_reg[MAX_I2C];

static i2c_driver_t* find_by_instance(I2C_TypeDef* instance)
{
    for (uint8_t i = 0; i < MAX_I2C; i++)
    {
        if (s_reg[i] && s_reg[i]->cfg.instance == instance)
        {
            return s_reg[i];
        }
    }
    return NULL;
}

// I2C not owned by CM4 return NULL.
static i2c_driver_t* find_by_handle(I2C_HandleTypeDef* h)
{
    for (uint8_t i = 0; i < MAX_I2C; i++)
    {
        if (s_reg[i] && &s_reg[i]->hi2c == h)
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

static link_status_t i2c_init(link_driver_t* base)
{
    i2c_driver_t* d = (i2c_driver_t *)base;

    memset(&d->hi2c, 0, sizeof(d->hi2c));
    d->hi2c.Instance = d->cfg.instance;
    d->hi2c.Init.Timing = d->cfg.timing;
    d->hi2c.Init.OwnAddress1 = (d->cfg.role == I2C_ROLE_SLAVE) ? ((uint32_t)d->cfg.own_addr7 << 1) : 0; // HAL wants 7-bit addr left shifted by one.
    d->hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    d->hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    d->hi2c.Init.OwnAddress2 = 0;
    d->hi2c.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    d->hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    d->hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    base->tx_done = 0;
    base->rx_done = 0;
    base->err_flags = 0;

    // HAL_I2C_MspInit handles clocks, AF, and NVIC.
    if (HAL_I2C_Init(&d->hi2c) != HAL_OK) return LINK_ERROR;
    if (HAL_I2CEx_ConfigAnalogFilter(&d->hi2c, I2C_ANALOGFILTER_ENABLE) != HAL_OK) return LINK_ERROR;
    if (HAL_I2CEx_ConfigDigitalFilter(&d->hi2c, 0) != HAL_OK) return LINK_ERROR;
    return LINK_OK;
}

static link_status_t i2c_deinit(link_driver_t* base)
{
    i2c_driver_t* d = (i2c_driver_t *)base;
    return (HAL_I2C_DeInit(&d->hi2c) == HAL_OK) ? LINK_OK : LINK_ERROR;
}

static link_status_t i2c_start_tx(link_driver_t* base, const uint8_t* buf, uint16_t len)
{
    i2c_driver_t* d = (i2c_driver_t *)base;
    HAL_StatusTypeDef ret;
    if (d->cfg.role == I2C_ROLE_MASTER)
    {
        ret = HAL_I2C_Master_Transmit_IT(&d->hi2c, (uint16_t)(d->cfg.peer_addr7 << 1), (uint8_t*)buf, len);
    } else {
        ret = HAL_I2C_Slave_Transmit_IT(&d->hi2c, (uint8_t*)buf, len);
    }
    return map_status(ret);
}

// HAL does not support slave transfer abort, so reset. Used on error/timeout paths.
static void i2c_abort(link_driver_t* base)
{
    i2c_driver_t* d = (i2c_driver_t*)base;
    HAL_I2C_DeInit(&d->hi2c);
    i2c_init(base);
}

static const link_driver_ops_t I2C_OPS = {
    .init = i2c_init,
    .deinit = i2c_deinit,
    .start_tx = i2c_start_tx,
    .statr_rx = i2c_start_rx,
    .abort = i2c_abort
};

void i2c_driver_create(i2c_driver_t* d, const i2c_cfg_t* cfg)
{
    memset(d, 0, sizeof(*d));
    d->base.ops = &I2C_OPS;
    d->cfg = *cfg;
    for (uint8_t i = 0; i < MAX_I2C; i++)
    {
        if (!s_reg[i])
        {
            s_reg[i] = d;
            break;
        }
    }
}

void i2c_driver_ev_irq(I2C_TypeDef* instance)
{
    i2c_driver_t* d = find_by_instance(instance);
    if (d)
    {
        HAL_I2C_EV_IRQHandler(&d->hi2c);
    }
}

void i2c_driver_er_irq(I2C_TypeDef* instance)
{
    i2c_driver_t* d = find_by_instance(instance);
    if (d)
    {
        HAL_I2C_ER_IRQHandler(&d->hi2c);
    }
}

/* HAL callbacks */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* h)
{
    i2c_driver_t* d = find_by_handle(h);
    if (d) 
    {
        d->base.tx_done = 1;
    }
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef* h)
{
    i2c_driver_t* d = find_by_handle(h);
    if (d) 
    {
        d->base.tx_done = 1;
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* h)
{
    i2c_driver_t* d = find_by_handle(h);
    if (d) 
    {
        d->base.rx_done = 1;
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef* h)
{
    i2c_driver_t* d = find_by_handle(h);
    if (d) 
    {
        d->base.rx_done = 1;
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* h)
{
    i2c_driver_t* d = find_by_handle(h);
    if (!d) return;

    uint32_t e = h->ErrorCode, f = 0;
    if (e & HAL_I2C_ERROR_AF)   f |= LINK_ERR_NACK;
    if (e & HAL_I2C_ERROR_BERR) f |= LINK_ERR_BUS;
    if (e & HAL_I2C_ERROR_ARLO) f |= LINK_ERR_ARBLOST;
    if (e & HAL_I2C_ERROR_OVR)  f |= LINK_ERR_OVERRUN;
    if (!f)                     f  = LINK_ERR_OTHER;
    d->base.err_flags |= f;
}