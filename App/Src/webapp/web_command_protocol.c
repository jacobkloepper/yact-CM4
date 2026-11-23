#include "web_command_protocol.h"
#include "links.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern uint32_t HAL_GetTick(void);

#define CMD_LINE_MAX      200u    /* longest accepted incoming line      */
#define TX_LINE_MAX       400u    /* longest outgoing line (RPMsg ~496)  */
#define RX_SLOTS          4u
#define TX_SLOTS          8u
#define MAX_TOKENS        10u
#define STATUS_PERIOD_MS  500u
#define TX_RETRY_MS       20u

/* ---- receive: assemble lines from arbitrary chunks ------------------------- */

static char     rxq[RX_SLOTS][CMD_LINE_MAX];
static uint8_t  rx_head, rx_tail, rx_count;
static char     rx_line[CMD_LINE_MAX];
static uint16_t rx_len;
static uint8_t  rx_overflow;

void cmd_rx(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        char c = (char)data[i];
        if (c == '\n' || c == '\r') {
            if (rx_len && !rx_overflow && rx_count < RX_SLOTS) {
                memcpy(rxq[rx_head], rx_line, rx_len);
                rxq[rx_head][rx_len] = '\0';
                rx_head = (uint8_t)((rx_head + 1u) % RX_SLOTS);
                rx_count++;
            }
            rx_len = 0;
            rx_overflow = 0;
        } else if (rx_len < CMD_LINE_MAX - 1u) {
            rx_line[rx_len++] = c;
        } else {
            rx_overflow = 1;      /* drop the rest of this over-long line */
        }
    }
}

static int rx_pop(char* out)
{
    if (!rx_count) return 0;
    memcpy(out, rxq[rx_tail], CMD_LINE_MAX);
    rx_tail = (uint8_t)((rx_tail + 1u) % RX_SLOTS);
    rx_count--;
    return 1;
}

/* ---- transmit: small FIFO of formatted lines -------------------------------- */

static char     txq[TX_SLOTS][TX_LINE_MAX];
static uint16_t txq_len[TX_SLOTS];
static uint8_t  tx_head, tx_tail, tx_count;
static uint32_t tx_retry_at;

static int tx_printf(const char *fmt, ...)
{
    if (tx_count == TX_SLOTS) return -1;
    char *slot = txq[tx_head];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(slot, TX_LINE_MAX - 1u, fmt, ap);
    va_end(ap);
    if (n < 0 || (unsigned)n >= TX_LINE_MAX - 1u) return -1;
    slot[n++] = '\n';
    txq_len[tx_head] = (uint16_t)n;
    tx_head = (uint8_t)((tx_head + 1u) % TX_SLOTS);
    tx_count++;
    return 0;
}

/* Send at most one queued line per call, so a stalled transport cannot hold
   up the main loop for long. */
static void tx_pump(void)
{
    if (!tx_count) return;
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - tx_retry_at) < 0) return;
    if (cmd_port_write((const uint8_t *)txq[tx_tail], txq_len[tx_tail]) == 0) {
        tx_tail = (uint8_t)((tx_tail + 1u) % TX_SLOTS);
        tx_count--;
    } else {
        tx_retry_at = now + TX_RETRY_MS;
    }
}

static void ack(const char *cmd, const char *id)
{
    tx_printf("ACK %s link=%s", cmd, id);
}

static void nack(const char *cmd, const char *id, const char *why)
{
    tx_printf("NACK %s link=%s reason=%s", cmd, id, why);
}

/* ---- per-link reporting state ----------------------------------------------- */

static uint8_t  done_sent[LINKS_MAX];       /* 1 = nothing left to report  */
static uint32_t last_status_ms[LINKS_MAX];

static const char *state_name(link_state_t s)
{
    switch (s) {
    case LINK_STATE_IDLE:    return "IDLE";
    case LINK_STATE_RUNNING: return "RUNNING";
    case LINK_STATE_DONE:    return "DONE";
    default:              return "FAILED";
    }
}

static void send_status(unsigned i)
{
    link_manager_t *m = links_mgr(i);
    link_state_t st = link_manager_state(m);
    if (st == LINK_STATE_RUNNING)
        tx_printf("STATUS link=%s state=RUNNING frame=%lu of=%lu", links_id(i),
                  (unsigned long)m->frame_idx, (unsigned long)m->cfg.frame_count);
    else
        tx_printf("STATUS link=%s state=%s", links_id(i), state_name(st));
}

static int send_done(unsigned i)
{
    link_manager_t *m = links_mgr(i);
    link_stats_t s;
    link_manager_get_stats(m, &s);
    return tx_printf(
        "DONE link=%s state=%s tx_bytes=%lu rx_bytes=%lu frames_ok=%lu frames_bad=%lu "
        "byte_mismatches=%lu crc_errors=%lu timeouts=%lu overruns=%lu framing_errors=%lu "
        "noise_errors=%lu parity_errors=%lu nacks=%lu bus_errors=%lu arb_lost=%lu "
        "underruns=%lu start_ms=%lu end_ms=%lu",
        links_id(i), state_name(link_manager_state(m)),
        (unsigned long)s.tx_bytes, (unsigned long)s.rx_bytes,
        (unsigned long)s.frames_ok, (unsigned long)s.frames_bad,
        (unsigned long)s.byte_mismatches, (unsigned long)s.crc_errors,
        (unsigned long)s.timeouts, (unsigned long)s.overruns,
        (unsigned long)s.framing_errors, (unsigned long)s.noise_errors,
        (unsigned long)s.parity_errors, (unsigned long)s.nacks,
        (unsigned long)s.bus_errors, (unsigned long)s.arb_lost,
        (unsigned long)s.underruns, (unsigned long)s.start_ms,
        (unsigned long)s.end_ms);
}

static void report_poll(void)
{
    uint32_t now = HAL_GetTick();
    for (unsigned i = 0; i < links_count(); i++) {
        link_state_t st = link_manager_state(links_mgr(i));
        if (st == LINK_STATE_RUNNING) {
            /* Progress is optional: skip it while the queue is backing up. */
            if (now - last_status_ms[i] >= STATUS_PERIOD_MS && tx_count < TX_SLOTS / 2u) {
                last_status_ms[i] = now;
                send_status(i);
            }
        } else if ((st == LINK_STATE_DONE || st == LINK_STATE_FAILED) && !done_sent[i]) {
            if (send_done(i) == 0)        /* retry next call if the queue was full */
                done_sent[i] = 1;
        }
    }
}

/* ---- command handling -------------------------------------------------------- */

static const char *kv(char **tok, unsigned n, const char *key)
{
    size_t kl = strlen(key);
    for (unsigned i = 1; i < n; i++)
        if (strncmp(tok[i], key, kl) == 0 && tok[i][kl] == '=')
            return tok[i] + kl + 1;
    return NULL;
}

static int kv_uint(char **tok, unsigned n, const char *key,
                   unsigned long lo, unsigned long hi, uint32_t *out)
{
    const char *s = kv(tok, n, key);
    char *end;
    if (!s || !*s) return -1;
    unsigned long v = strtoul(s, &end, 10);
    if (*end != '\0' || v < lo || v > hi) return -1;
    *out = (uint32_t)v;
    return 0;
}

static void cmd_start(unsigned idx, char **tok, unsigned n)
{
    const char *id = links_id(idx);
    link_manager_t *m = links_mgr(idx);
    uint32_t frames, len, timeout;

    if (kv_uint(tok, n, "frames", 1, 100000, &frames) ||
        kv_uint(tok, n, "len", 1, LINK_MAX_FRAME, &len) ||
        kv_uint(tok, n, "timeout", 1, 10000, &timeout)) {
        nack("START", id, "badparam");
        return;
    }
    if (link_manager_state(m) == LINK_STATE_RUNNING) {
        nack("START", id, "busy");
        return;
    }

    link_test_cfg_t cfg = { .frame_len = (uint16_t)len, .frame_count = frames,
                            .frame_timeout_ms = timeout };
    if (link_manager_start(m, &cfg) != LINK_OK) {
        done_sent[idx] = 1;               /* no run, so nothing to report */
        nack("START", id, "initfail");
        return;
    }
    done_sent[idx] = 0;
    last_status_ms[idx] = HAL_GetTick();
    ack("START", id);                     /* always queued before any STATUS */
}

static void cmd_abort(unsigned idx)
{
    const char *id = links_id(idx);
    link_manager_t *m = links_mgr(idx);

    if (link_manager_state(m) != LINK_STATE_RUNNING) {
        nack("ABORT", id, "idle");
        return;
    }
    link_manager_abort(m);                    /* the DONE line follows from report_poll() */
    ack("ABORT", id);
}

static void handle_line(char *line)
{
    char *tok[MAX_TOKENS];
    unsigned n = 0;

    for (char *p = strtok(line, " \t"); p && n < MAX_TOKENS; p = strtok(NULL, " \t"))
        tok[n++] = p;
    if (n == 0) return;

    const char *cmd = tok[0];
    const char *id = kv(tok, n, "link");
    if (!id) id = "-";

    int is_start  = strcmp(cmd, "START")  == 0;
    int is_abort  = strcmp(cmd, "ABORT")  == 0;
    int is_status = strcmp(cmd, "STATUS") == 0;
    if (!is_start && !is_abort && !is_status) {
        nack(cmd, id, "badcmd");
        return;
    }

    int idx = links_index(id);
    if (idx < 0) {
        nack(cmd, id, "badlink");
        return;
    }

    if (is_start)       cmd_start((unsigned)idx, tok, n);
    else if (is_abort)  cmd_abort((unsigned)idx);
    else                send_status((unsigned)idx);
}

/* ---- public ------------------------------------------------------------------ */

void cmd_init(void)
{
    rx_head = rx_tail = rx_count = 0;
    rx_len = 0;
    rx_overflow = 0;
    tx_head = tx_tail = tx_count = 0;
    tx_retry_at = 0;
    memset(done_sent, 1, sizeof done_sent);      /* nothing pending before a START */
    memset(last_status_ms, 0, sizeof last_status_ms);
}

void cmd_poll(void)
{
    char line[CMD_LINE_MAX];

    while (rx_pop(line))
        handle_line(line);
    report_poll();
    tx_pump();
}