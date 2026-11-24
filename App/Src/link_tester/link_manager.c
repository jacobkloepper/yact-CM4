#include "link_manager.h"
#include "link_driver.h"
#include "main.h"
#include "stm32mp1xx_hal.h"
#include <string.h>

/* Static helpers */
static uint8_t split(const link_manager_t* m)
{
    return m->p_link_rx != m->p_link_tx;
}

static void abort_all(link_manager_t* m)
{
    m->p_link_tx->p_ops->abort(m->p_link_tx);
    if (split(m))
    {
        m->p_link_rx->p_ops->abort(m->p_link_rx);
    }
}

static void deinit_all(link_manager_t* m)
{
    m->p_link_tx->p_ops->deinit(m->p_link_tx);
    if (split(m))
    {
        m->p_link_rx->p_ops->deinit(m->p_link_rx);
    }
}

static void finish(link_manager_t* m, link_state_t state)
{
    abort_all(m);
    deinit_all(m);
    m->stats.end_ms = HAL_GetTick();
    m->state = state;
}

static void begin_frame(link_manager_t* m)
{
    // Get buffers ready
    for (uint16_t i = 0; i < m->cfg.frame_len; i++)
    {
        m->tx_buf[i] = (uint8_t)(m->frame_idx + i); // TODO: dummy data
    }
    memset(m->rx_buf, 0, m->cfg.frame_len);

    // Initialize sender/receiver
    if (m->p_link_rx->p_ops->start_rx(m->p_link_rx, m->rx_buf, m->cfg.frame_len) != LINK_OK ||
        m->p_link_tx->p_ops->start_tx(m->p_link_tx, m->tx_buf, m->cfg.frame_len) != LINK_OK)
    {
        finish(m, LINK_STATE_FAILED);
        return;
    }

    // Set frame flags
    m->tx_pending = 1;
    m->rx_pending = 1;
    m->frame_failed = 0;
    m->frame_active = 1;
    m->frame_deadline = HAL_GetTick() + m->cfg.frame_timeout_ms;
}

static void end_frame(link_manager_t* m)
{
    if (!m->frame_failed)
    {
        m->stats.frames_ok++;
    } else {
        m->stats.frames_bad++;
    }

    m->frame_active = 0;
    m->frame_idx++;
}

/* Getters */
link_state_t link_manager_state(const link_manager_t* m)
{
    return m->state;
}

void link_manager_get_stats(const link_manager_t* m, link_stats_t* stats)
{
    *stats = m->stats;
}

// Binds the driver to the link, and idles the link.
void link_manager_init(link_manager_t* m, link_driver_t* tx, link_driver_t* rx)
{
    memset(m, 0, sizeof *m);
    m->p_link_tx = tx;
    m->p_link_rx = rx ? rx : tx;
    m->state = LINK_STATE_IDLE;
}

// Initialize driver and start the test.
link_status_t link_manager_start(link_manager_t* m, const link_test_cfg_t* cfg)
{
    // Fail if link already running or poorly configured.
    if (m->state == LINK_STATE_RUNNING || cfg-> frame_len == 0 || cfg-> frame_len > LINK_MAX_FRAME)
    {
        return LINK_ERROR;
    }

    // Set basic members.
    m->cfg = *cfg;
    memset(&(m->stats), 0, sizeof (m->stats));
    m->frame_idx = 0;
    m->frame_active = 0;

    // Initialize driver.
    if (m->p_link_tx->p_ops->init(m->p_link_tx) != LINK_OK)
    {
        m->state = LINK_STATE_FAILED;
        return LINK_ERROR;
    }
    if (split(m) && m->p_link_rx->p_ops->init(m->p_link_rx) != LINK_OK)
    {
        m->p_link_tx->p_ops->deinit(m->p_link_tx);
        m->state = LINK_STATE_FAILED;
        return LINK_ERROR;
    }

    m->stats.start_ms = HAL_GetTick();
    m->state = LINK_STATE_RUNNING;
    return LINK_OK;
}

// Step through link manager state machine.
void link_manager_step(link_manager_t* m)
{
    // Exit if link not currently running.
    if (m->state != LINK_STATE_RUNNING)
    {
        return;
    }

    // Finish frame and start next
    if (!m->frame_active)
    {
        if (m->frame_idx >= m->cfg.frame_count)
        {
            finish(m, LINK_STATE_DONE);
            return;
        }
        begin_frame(m);
        return;
    }

    // Atomically snapshot and clear flags from ISR
    link_driver_t* tx = m->p_link_tx;
    link_driver_t* rx = m->p_link_rx;
    uint8_t tx_done, rx_done; uint32_t err;
    __disable_irq();

    tx_done = tx->tx_done;
    tx->tx_done = 0;

    rx_done = rx->rx_done;
    rx->rx_done = 0;

    err = tx->err_flags;
    tx->err_flags = 0;
    if (rx != tx)
    {
        err |= rx->err_flags;
        rx->err_flags = 0;
    }
    __enable_irq();

    // Update link errors
    if (err)
    {
        if (err & LINK_ERR_PARITY)      m->stats.parity_errors++;
        if (err & LINK_ERR_FRAMING)     m->stats.framing_errors++;
        if (err & LINK_ERR_NOISE)       m->stats.noise_errors++;
        if (err & LINK_ERR_OVERRUN)     m->stats.overruns++;
        if (err & LINK_ERR_OTHER)       m->stats.other_errors++;
        if (err & LINK_ERR_NACK)        m->stats.nacks++;
        if (err & LINK_ERR_BUS)         m->stats.bus_errors++;
        if (err & LINK_ERR_ARBLOST)     m->stats.arb_lost++;
        if (err & LINK_ERR_UNDERRUN)    m->stats.underruns++;
        m->frame_failed = 1;
        abort_all(m);
        end_frame(m);
        return;
    }

    // If no errors, update tx/rx stats
    if (tx_done)
    {
        m->stats.tx_bytes += m->cfg.frame_len;
        m->tx_pending = 0;
    }
    if (rx_done)
    {
        m->stats.rx_bytes += m->cfg.frame_len;
        m->rx_pending = 0;
        for (uint16_t i = 0; i < m->cfg.frame_len; i++)
        {
            if (m->rx_buf[i] != m->tx_buf[i])
            {
                m->stats.byte_mismatches++;
                m->frame_failed = 1; // TODO: more?
            }
        }
    }

    // End frame if no more data expected
    if (!m->tx_pending && !m->rx_pending)
    {
        end_frame(m);
        return;
    }

    // Fail on timeout
    if ((int32_t)(HAL_GetTick() - m->frame_deadline) >= 0)
    {
        m->stats.timeouts++;
        m->frame_failed = 1;
        abort_all(m);
        end_frame(m);
    }
}

// Immediately end a running link.
void link_manager_abort(link_manager_t* m)
{
    if (m->state == LINK_STATE_RUNNING)
    {
        finish(m, LINK_STATE_DONE);
    }
}
