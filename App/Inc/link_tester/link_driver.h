#ifndef LINK_DRIVER_H
#define LINK_DRIVER_H

#include <stdint.h>
#include <stddef.h>

/* Generic transport interface */
typedef enum {
    LINK_OK = 0,
    LINK_BUSY,
    LINK_ERROR
} link_status_t;

/* Error bits map from driver */
#define LINK_ERR_PARITY     (1u << 0) 
#define LINK_ERR_FRAMING    (1u << 1)
#define LINK_ERR_NOISE      (1u << 2)
#define LINK_ERR_OVERRUN    (1u << 3)
#define LINK_ERR_OTHER      (1u << 4) // Anything else, expand as needed
#define LINK_ERR_NACK       (1u << 5)
#define LINK_ERR_BUS        (1u << 6)
#define LINK_ERR_ARBLOST    (1u << 7)
#define LINK_ERR_UNDERRUN   (1u << 8)

typedef struct link_driver link_driver_t; // Need to pre-define to define the following struct.

typedef struct {
    link_status_t (*init)       (link_driver_t *p_link_driver);
    link_status_t (*deinit)     (link_driver_t *p_link_driver);
    link_status_t (*start_tx)   (link_driver_t *p_link_driver, const uint8_t *p_buf, uint16_t len);
    link_status_t (*start_rx)   (link_driver_t *p_link_driver, uint8_t *p_buf, uint16_t len);
    void (*abort)               (link_driver_t *p_link_driver);
} link_driver_ops_t;

/* Base struct. Flags written from interrupt context, atomic r/w. */
struct link_driver {
    const link_driver_ops_t *p_ops;
    volatile uint8_t tx_done;
    volatile uint8_t rx_done;
    volatile uint32_t err_flags; // From LINK_ERR_* defs
};

#endif /* LINK_DRIVER_H */