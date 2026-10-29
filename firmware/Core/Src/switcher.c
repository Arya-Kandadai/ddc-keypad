/**
 * switcher.c - see switcher.h
 */
#include "switcher.h"

typedef enum {
    ST_IDLE = 0,
    ST_WAIT_SET,      /* Set VCP sent, honouring the 50 ms gap        */
    ST_WAIT_REPLY,    /* Get VCP sent, waiting 40 ms before reading    */
    ST_BACKOFF        /* attempt failed, waiting before a retry        */
} sw_state_t;

typedef struct {
    uint8_t  monitor;
    uint16_t code;
} job_t;

static const switcher_port_t *s_port;
static sw_state_t      s_state;
static uint32_t        s_deadline;
static switch_result_t s_res;
static uint32_t        s_start_ms;

static job_t   s_jobs[SWITCHER_QUEUE_SIZE];
static uint8_t s_jhead, s_jtail, s_jcount;

/* Wrap-safe "has the deadline passed?" (HAL tick wraps after ~49 days). */
/* +1 ms because the tick has 1 ms resolution: "now + 50" could otherwise
 * expire after only 49.x ms of real time and violate the DDC/CI spec. */
#define TICK_MARGIN_MS 1u

static bool due(uint32_t now)
{
    return (int32_t)(now - s_deadline) >= 0;
}

static void finish(switch_status_t status)
{
    uint32_t now = s_port->now_ms(s_port->ctx);
    s_res.status     = status;
    s_res.elapsed_ms = now - s_start_ms;
    s_port->select_bus(s_port->ctx, SWITCHER_NO_BUS);
    s_state = ST_IDLE;
    if (s_port->on_result) {
        s_port->on_result(s_port->ctx, &s_res);
    }
}

static void attempt_failed(switch_status_t why, ddc_status_t ddc_err)
{
    s_res.status  = why;
    if (ddc_err != DDC_OK) {
        s_res.ddc_err = ddc_err;
    }
    if (s_res.attempts >= SWITCHER_MAX_ATTEMPTS) {
        finish(why);
        return;
    }
    s_state    = ST_BACKOFF;
    s_deadline = s_port->now_ms(s_port->ctx) + SWITCHER_BACKOFF_MS * s_res.attempts
                 + TICK_MARGIN_MS;
}

static void send_set(void)
{
    uint8_t pkt[DDC_SET_VCP_LEN];
    size_t  n = ddc_build_set_vcp(pkt, VCP_INPUT_SOURCE, s_res.requested);

    s_res.attempts++;
    s_port->select_bus(s_port->ctx, s_res.monitor);
    if (!s_port->i2c_write(s_port->ctx, pkt, n)) {
        attempt_failed(SW_ERR_NACK, DDC_OK);
        return;
    }
    s_state    = ST_WAIT_SET;
    s_deadline = s_port->now_ms(s_port->ctx) + DDC_SET_DELAY_MS + TICK_MARGIN_MS;
}

static void send_get(void)
{
    uint8_t pkt[DDC_GET_VCP_LEN];
    size_t  n = ddc_build_get_vcp(pkt, VCP_INPUT_SOURCE);

    if (!s_port->i2c_write(s_port->ctx, pkt, n)) {
        attempt_failed(SW_ERR_NACK, DDC_OK);
        return;
    }
    s_state    = ST_WAIT_REPLY;
    s_deadline = s_port->now_ms(s_port->ctx) + DDC_REPLY_DELAY_MS + TICK_MARGIN_MS;
}

static void read_and_verify(void)
{
    uint8_t         rx[DDC_VCP_REPLY_LEN];
    ddc_vcp_reply_t reply;

    if (!s_port->i2c_read(s_port->ctx, rx, sizeof rx)) {
        attempt_failed(SW_ERR_NACK, DDC_OK);
        return;
    }
    ddc_status_t st = ddc_parse_vcp_reply(rx, sizeof rx, VCP_INPUT_SOURCE, &reply);
    if (st != DDC_OK) {
        attempt_failed(SW_ERR_PROTOCOL, st);
        return;
    }
    s_res.actual = reply.present;
    if (reply.present != s_res.requested) {
        attempt_failed(SW_ERR_MISMATCH, DDC_OK);
        return;
    }
    finish(SW_OK);
}

void switcher_init(const switcher_port_t *port)
{
    s_port  = port;
    s_state = ST_IDLE;
    s_jhead = s_jtail = s_jcount = 0;
    s_port->select_bus(s_port->ctx, SWITCHER_NO_BUS);
}

bool switcher_request(uint8_t monitor, uint16_t input_code)
{
    if (s_jcount >= SWITCHER_QUEUE_SIZE) {
        return false;
    }
    s_jobs[s_jhead].monitor = monitor;
    s_jobs[s_jhead].code    = input_code;
    s_jhead = (uint8_t)((s_jhead + 1u) % SWITCHER_QUEUE_SIZE);
    s_jcount++;
    return true;
}

void switcher_poll(void)
{
    uint32_t now = s_port->now_ms(s_port->ctx);

    switch (s_state) {
    case ST_IDLE:
        if (s_jcount == 0) {
            return;
        }
        s_res.monitor   = s_jobs[s_jtail].monitor;
        s_res.requested = s_jobs[s_jtail].code;
        s_res.actual    = 0;
        s_res.status    = SW_OK;
        s_res.ddc_err   = DDC_OK;
        s_res.attempts  = 0;
        s_jtail = (uint8_t)((s_jtail + 1u) % SWITCHER_QUEUE_SIZE);
        s_jcount--;
        s_start_ms = now;
        send_set();
        break;

    case ST_WAIT_SET:
        if (due(now)) {
            send_get();
        }
        break;

    case ST_WAIT_REPLY:
        if (due(now)) {
            read_and_verify();
        }
        break;

    case ST_BACKOFF:
        if (due(now)) {
            send_set();
        }
        break;
    }
}

bool switcher_busy(void)
{
    return s_state != ST_IDLE || s_jcount != 0;
}

const char *switcher_status_str(switch_status_t s)
{
    switch (s) {
    case SW_OK:             return "OK";
    case SW_ERR_NACK:       return "NACK";
    case SW_ERR_PROTOCOL:   return "PROTOCOL";
    case SW_ERR_MISMATCH:   return "MISMATCH";
    case SW_ERR_QUEUE_FULL: return "QUEUE FULL";
    }
    return "?";
}
