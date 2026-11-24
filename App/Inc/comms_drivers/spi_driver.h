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
    SPI_TypeDef*    p_instance;
    spi_role_t      role;
    uint32_t        baud_prescaler;
    uint8_t         mode;
    uint8_t         hw_nss;
} spi_cfg_t;

typedef struct {
    link_driver_t       link_driver;                // Must be first.
    spi_cfg_t           cfg;
    SPI_HandleTypeDef   hspi;
    uint8_t             scratch[SPI_MAX_FRAME];
    uint8_t*            p_lb_rx;
    uint16_t            lb_len;
} spi_driver_t;

// Binds driver. Does not touch hardware until p_driver->link_driver.p_ops->init().
void spi_driver_create(spi_driver_t* p_driver, const spi_cfg_t* p_config);

// Call from generated IRQ handler.
void spi_driver_irq(SPI_TypeDef* p_instance);

#endif /* SPI_DRIVER_H */