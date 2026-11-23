#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include "link_driver.h"
#include "main.h"
#include "stm32mp1xx_hal_spi.h"
#include <stdint.h>

#define SPI_MAX_FRAME 256u

typedef enum {
    SPI_ROLE_MASTER = 0,    /* drives SCK/MOSI. TX endpoint */
    SPI_ROLE_SLAVE,         /* clocked by a master. RX endpoint. */
    SPI_ROLE_LOOPBACK       /* one master, MOSI wired to MISO. TX & RX endpoint. */
} spi_role_t;

typedef struct {
    SPI_TypeDef*    instance;
    spi_role_t      role;
    uint32_t        baud_prescaler;
    uint8_t         mode;
    uint8_t         hw_nss;
} spi_cfg_t;

typedef struct {
    link_driver_t       base;
    spi_cfg_t           cfg;
    SPI_HandleTypeDef   hspi;
    uint8_t             scratch[SPI_MAX_FRAME];
    uint8_t*            lb_rx;
    uint16_t            lb_len;
} spi_driver_t;

// Binds driver. Does not touch hardware until d->base->ops->init().
void spi_driver_create(spi_driver_t* d, const spi_cfg_t* cfg);

// Call from generated IRQ handler.
void spi_driver_irq(SPI_TypeDef* instance);

#endif /* SPI_DRIVER_H */