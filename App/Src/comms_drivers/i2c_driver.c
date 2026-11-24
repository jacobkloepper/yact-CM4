#include "i2c_driver.h"
#include "link_driver.h"
#include "stm32mp157dxx_cm4.h"
#include "stm32mp1xx_hal_def.h"
#include "stm32mp1xx_hal_i2c.h"
#include "stm32mp1xx_hal_i2c_ex.h"
#include <string.h>

#define MAX_I2C 4
static i2c_driver_t* p_driver_reg[MAX_I2C];

static i2c_driver_t* find_by_instance(I2C_TypeDef* p_instance)
{
    for (uint8_t i = 0; i < MAX_I2C; i++)
    {
        if (p_driver_reg[i] && p_driver_reg[i]->cfg.p_instance == p_instance)
        {
            return p_driver_reg[i];
        }
    }
    return NULL;
}

// I2C not owned by CM4 return NULL.
static i2c_driver_t* find_by_handle(I2C_HandleTypeDef* p_handle)
{
    for (uint8_t i = 0; i < MAX_I2C; i++)
    {
        if (p_driver_reg[i] && &p_driver_reg[i]->hi2c == p_handle)
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

static link_status_t i2c_init(link_driver_t* p_link_driver)
{
    i2c_driver_t* p_driver = (i2c_driver_t *)p_link_driver;

    memset(&p_driver->hi2c, 0, sizeof(p_driver->hi2c));
    p_driver->hi2c.Instance = p_driver->cfg.p_instance;
    p_driver->hi2c.Init.Timing = p_driver->cfg.timing;
    p_driver->hi2c.Init.OwnAddress1 = (p_driver->cfg.role == I2C_ROLE_SLAVE) ? ((uint32_t)p_driver->cfg.own_addr7 << 1) : 0; // HAL wants 7-bit addr left shifted by one.
    p_driver->hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    p_driver->hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    p_driver->hi2c.Init.OwnAddress2 = 0;
    p_driver->hi2c.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    p_driver->hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    p_driver->hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    p_link_driver->tx_done = 0;
    p_link_driver->rx_done = 0;
    p_link_driver->err_flags = 0;

    // HAL_I2C_MspInit handles clocks, AF, and NVIC.
    if (HAL_I2C_Init(&p_driver->hi2c) != HAL_OK) return LINK_ERROR;
    if (HAL_I2CEx_ConfigAnalogFilter(&p_driver->hi2c, I2C_ANALOGFILTER_ENABLE) != HAL_OK) return LINK_ERROR;
    if (HAL_I2CEx_ConfigDigitalFilter(&p_driver->hi2c, 0) != HAL_OK) return LINK_ERROR;
    return LINK_OK;
}

static link_status_t i2c_deinit(link_driver_t* p_link_driver)
{
    i2c_driver_t* p_driver = (i2c_driver_t *)p_link_driver;
    return (HAL_I2C_DeInit(&p_driver->hi2c) == HAL_OK) ? LINK_OK : LINK_ERROR;
}

static link_status_t i2c_start_tx(link_driver_t* p_link_driver, const uint8_t* p_buf, uint16_t len)
{
    i2c_driver_t* p_driver = (i2c_driver_t *)p_link_driver;
    HAL_StatusTypeDef ret;
    if (p_driver->cfg.role == I2C_ROLE_MASTER)
    {
        ret = HAL_I2C_Master_Transmit_IT(&p_driver->hi2c, (uint16_t)(p_driver->cfg.peer_addr7 << 1), (uint8_t*)p_buf, len);
    } else {
        ret = HAL_I2C_Slave_Transmit_IT(&p_driver->hi2c, (uint8_t*)p_buf, len);
    }
    return map_status(ret);
}

static link_status_t i2c_start_rx(link_driver_t* p_link_driver, uint8_t* p_buf, uint16_t len)
{
    i2c_driver_t* p_driver = (i2c_driver_t *)p_link_driver;
    HAL_StatusTypeDef ret;
    if (p_driver->cfg.role == I2C_ROLE_SLAVE)
        ret = HAL_I2C_Slave_Receive_IT(&p_driver->hi2c, p_buf, len);
    else
        ret = HAL_I2C_Master_Receive_IT(&p_driver->hi2c, (uint16_t)(p_driver->cfg.peer_addr7 << 1), p_buf, len);
    return map_status(ret);
}

// HAL does not support slave transfer abort, so reset. Used on error/timeout paths.
static void i2c_abort(link_driver_t* p_link_driver)
{
    i2c_driver_t* p_driver = (i2c_driver_t*)p_link_driver;
    HAL_I2C_DeInit(&p_driver->hi2c);
    i2c_init(p_link_driver);
}

static const link_driver_ops_t I2C_OPS = {
    .init = i2c_init,
    .deinit = i2c_deinit,
    .start_tx = i2c_start_tx,
    .start_rx = i2c_start_rx,
    .abort = i2c_abort
};

void i2c_driver_create(i2c_driver_t* p_driver, const i2c_cfg_t* p_config)
{
    memset(p_driver, 0, sizeof(*p_driver));
    p_driver->link_driver.p_ops = &I2C_OPS;
    p_driver->cfg = *p_config;
    for (uint8_t i = 0; i < MAX_I2C; i++)
    {
        if (!p_driver_reg[i])
        {
            p_driver_reg[i] = p_driver;
            break;
        }
    }
}

void i2c_driver_ev_irq(I2C_TypeDef* p_instance)
{
    i2c_driver_t* p_driver = find_by_instance(p_instance);
    if (p_driver)
    {
        HAL_I2C_EV_IRQHandler(&p_driver->hi2c);
    }
}

void i2c_driver_er_irq(I2C_TypeDef* p_instance)
{
    i2c_driver_t* p_driver = find_by_instance(p_instance);
    if (p_driver)
    {
        HAL_I2C_ER_IRQHandler(&p_driver->hi2c);
    }
}

/* HAL callbacks */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* p_handle)
{
    i2c_driver_t* p_driver = find_by_handle(p_handle);
    if (p_driver) 
    {
        p_driver->link_driver.tx_done = 1;
    }
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef* p_handle)
{
    i2c_driver_t* p_driver = find_by_handle(p_handle);
    if (p_driver) 
    {
        p_driver->link_driver.tx_done = 1;
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* p_handle)
{
    i2c_driver_t* p_driver = find_by_handle(p_handle);
    if (p_driver) 
    {
        p_driver->link_driver.rx_done = 1;
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef* p_handle)
{
    i2c_driver_t* p_driver = find_by_handle(p_handle);
    if (p_driver) 
    {
        p_driver->link_driver.rx_done = 1;
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* p_handle)
{
    i2c_driver_t* p_driver = find_by_handle(p_handle);
    if (!p_driver) return;

    uint32_t e = p_handle->ErrorCode, f = 0;
    if (e & HAL_I2C_ERROR_AF)   f |= LINK_ERR_NACK;
    if (e & HAL_I2C_ERROR_BERR) f |= LINK_ERR_BUS;
    if (e & HAL_I2C_ERROR_ARLO) f |= LINK_ERR_ARBLOST;
    if (e & HAL_I2C_ERROR_OVR)  f |= LINK_ERR_OVERRUN;
    if (!f)                     f  = LINK_ERR_OTHER;
    p_driver->link_driver.err_flags |= f;
}
