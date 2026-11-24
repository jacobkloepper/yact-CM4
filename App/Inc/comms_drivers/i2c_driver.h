#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#include "link_driver.h"
#include "main.h"
#include "stm32mp157dxx_cm4.h"
#include "stm32mp1xx_hal_i2c.h"

typedef enum {
    I2C_ROLE_MASTER = 0,
    I2C_ROLE_SLAVE
} i2c_role_t;

typedef struct {
    I2C_TypeDef*        p_instance;
    i2c_role_t          role;
    uint32_t            timing;
    uint8_t             own_addr7;
    uint8_t             peer_addr7;
} i2c_cfg_t;

typedef struct {
    link_driver_t       link_driver; // Must be first
    i2c_cfg_t           cfg;
    I2C_HandleTypeDef   hi2c;
} i2c_driver_t;

// Binds driver. Does not touch hardware until p_driver->link_driver.p_ops->init().
void i2c_driver_create(i2c_driver_t* p_driver, const i2c_cfg_t* p_config);

// Call from generated I2C IRQ handler.
void i2c_driver_ev_irq(I2C_TypeDef* p_instance);
void i2c_driver_er_irq(I2C_TypeDef* p_instance);

#endif /* I2C_DRIVER_H */