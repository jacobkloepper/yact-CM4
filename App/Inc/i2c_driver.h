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
    I2C_TypeDef*        instance;
    i2c_role_t          role;
    uint32_t            timing;
    uint8_t             own_addr7;
    uint8_t             peer_addr7;
} i2c_cfg_t;

typedef struct {
    link_driver_t       base;
    i2c_cfg_t           cfg;
    I2C_HandleTypeDef   hi2c;
} i2c_driver_t;

// Binds driver. Does not touch hardware until d->base->ops->init().
void i2c_driver_create(i2c_driver_t* d, const i2c_cfg_t* cfg);

// Call from generated I2C IRQ handler.
void i2c_driver_ev_irq(I2C_TypeDef* instance);
void i2c_driver_er_irq(I2C_TypeDef* instance);

#endif /* I2C_DRIVER_H */