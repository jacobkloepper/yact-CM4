#include "links.h"
#include "uart_driver.h"
#include "i2c_driver.h"
#include "spi_driver.h"
#include <string.h>

#define LINK_I2C_ADDR  0x18             /* 7-bit, no shift */
#define I2C2_TIMING    0x10707DBCu      /* master: Init.Timing from CubeMX hi2c2 */
#define I2C5_TIMING    0x40505874u      /* slave:  Init.Timing from CubeMX hi2c5 */

static uart_driver_t uart3;
static i2c_driver_t  i2c_m, i2c_s;
static spi_driver_t  spi5;
static link_manager_t    mgr_uart, mgr_i2c, mgr_spi;

/* The ids are the contract with the web side: keep them in sync with
   config.LINKS in the Flask app. */
static const struct { const char *id; link_manager_t *mgr; } table[] = {
    { "uart3", &mgr_uart },
    { "i2c2",  &mgr_i2c  },
    { "spi5",  &mgr_spi  },
};
#define N_LINKS (sizeof table / sizeof table[0])
_Static_assert(N_LINKS <= LINKS_MAX, "raise LINKS_MAX");

void links_init(void)
{
    /* UART: USART3 TX jumpered to RX, one driver is both endpoints */
    uart_cfg_t ucfg = {
        .instance = USART3, .baud = 115200,
        .word_length = UART_WORDLENGTH_8B,
        .parity = UART_PARITY_NONE, .stop_bits = UART_STOPBITS_1,
    };
    uart_driver_create(&uart3, &ucfg);
    link_manager_init(&mgr_uart, &uart3.base, NULL);

    /* I2C: I2C2 master writes, I2C5 slave receives */
    i2c_cfg_t mcfg = { .instance = I2C2, .role = I2C_ROLE_MASTER,
                       .timing = I2C2_TIMING, .peer_addr7 = LINK_I2C_ADDR };
    i2c_cfg_t scfg = { .instance = I2C5, .role = I2C_ROLE_SLAVE,
                       .timing = I2C5_TIMING, .own_addr7  = LINK_I2C_ADDR };
    i2c_driver_create(&i2c_m, &mcfg);
    i2c_driver_create(&i2c_s, &scfg);
    link_manager_init(&mgr_i2c, &i2c_m.base, &i2c_s.base);

    /* SPI: SPI5 single-instance loopback (MOSI jumpered to MISO) */
    spi_cfg_t lb = { .instance = SPI5, .role = SPI_ROLE_LOOPBACK,
                     .baud_prescaler = SPI_BAUDRATEPRESCALER_256, .mode = 0 };
    spi_driver_create(&spi5, &lb);
    link_manager_init(&mgr_spi, &spi5.base, NULL);
}

unsigned    links_count(void)       { return (unsigned)N_LINKS; }
const char* links_id(unsigned i)    { return table[i].id; }
link_manager_t* links_mgr(unsigned i)   { return table[i].mgr; }

int links_index(const char *id)
{
    for (unsigned i = 0; i < N_LINKS; i++)
        if (strcmp(table[i].id, id) == 0) return (int)i;
    return -1;
}

void links_step_all(void)
{
    for (unsigned i = 0; i < N_LINKS; i++)
        link_manager_step(table[i].mgr);
}