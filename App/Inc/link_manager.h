#ifndef LINK_MANAGER_H
#define LINK_MANAGER_H

#include <stdint.h>
#include "link_driver.h"

#define LINK_MAX_FRAME 256u

typedef enum {
    LINK_STATE_IDLE = 0,            /* No test. Start with link_manager_start(). */
    LINK_STATE_RUNNING,             /* Test in progress. Advance with link_manager_step(). */
    LINK_STATE_DONE,                /* Test finished. Report stats with TODO(). */
    LINK_STATE_FAILED               /* Driver fail. */
} link_state_t;

typedef struct {
    uint32_t tx_bytes;
    uint32_t rx_bytes;
    uint32_t frames_ok;
    uint32_t frames_bad;
    uint32_t byte_mismatches;
    uint32_t crc_errors;
    uint32_t timeouts;
    uint32_t parity_errors;
    uint32_t framing_errors;
    uint32_t noise_errors;
    uint32_t overruns;
    uint32_t other_errors;
    uint32_t start_ms;
    uint32_t end_ms;
} link_stats_t;

typedef struct {
    uint16_t frame_len;             /* <= LINK_MAX_FRAME */
    uint32_t frame_count;
    uint32_t frame_timeout_ms;
} link_test_cfg_t;

typedef struct {
    /* Link lifetime */
    link_driver_t*      driver;
    link_state_t        state;
    link_test_cfg_t     cfg;
    link_stats_t        stats;

    /* Per-frame */
    uint32_t frame_idx;
    uint32_t frame_deadline;
    uint8_t  frame_active;
    uint8_t  tx_pending;
    uint8_t  rx_pending;
    uint8_t  frame_failed;
    uint8_t  tx_buf[LINK_MAX_FRAME];
    uint8_t  rx_buf[LINK_MAX_FRAME];
} link_manager_t;

/* Bind a manager to a driver (static alloc. by caller) */
void link_manager_init(link_manager_t* m, link_driver_t* d);

/* Initialize driver and set running state. */
link_status_t link_manager_start(link_manager_t* m, const link_test_cfg_t* cfg);

/* Call each loop iteration. Non-blocking. */
void link_manager_step(link_manager_t* m);
void link_manager_abort(link_manager_t* m);
link_state_t link_manager_state(const link_manager_t* m);

/* Copy stats for upstream handoff. */
void link_manager_get_stats(const link_manager_t* m, link_stats_t* stats);

#endif /* LINK_MANAGER_H */