/**
 * switcher.h - Non-blocking DDC/CI input-switch job runner.
 *
 * Each job = "set monitor M to input X, then prove it worked":
 *
 *   IDLE --pop job--> send Set VCP 0x60 --50 ms--> send Get VCP 0x60
 *        --40 ms--> read reply --> verify value == requested --> DONE
 *              any failure --> BACKOFF (50 ms * attempt) --> retry
 *              after SWITCHER_MAX_ATTEMPTS failures --> FAILED
 *
 * switcher_poll() never blocks; it just checks the clock and advances the
 * state machine, so the main loop can sleep (WFI) between ticks.
 * All hardware access goes through switcher_port_t (testable on a PC).
 */
#ifndef SWITCHER_H
#define SWITCHER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "ddc.h"

#define SWITCHER_MAX_ATTEMPTS  3u
#define SWITCHER_BACKOFF_MS    50u
#define SWITCHER_QUEUE_SIZE    8u
#define SWITCHER_NO_BUS        0xFFu

typedef enum {
    SW_OK = 0,
    SW_ERR_NACK,        /* display did not acknowledge (absent / busy)   */
    SW_ERR_PROTOCOL,    /* reply failed to parse - see ddc_err            */
    SW_ERR_MISMATCH,    /* reply parsed but input is not what we set      */
    SW_ERR_QUEUE_FULL
} switch_status_t;

typedef struct {
    uint8_t         monitor;
    uint16_t        requested;
    uint16_t        actual;      /* value read back (valid if status OK/MISMATCH) */
    switch_status_t status;
    ddc_status_t    ddc_err;     /* last protocol error seen (DDC_OK if none)    */
    uint8_t         attempts;
    uint32_t        elapsed_ms;
} switch_result_t;

typedef struct {
    void     *ctx;
    /* Route the shared I2C bus to one monitor (SWITCHER_NO_BUS = none). */
    void     (*select_bus)(void *ctx, uint8_t monitor);
    /* Write/read to 7-bit address DDC_I2C_ADDR_7BIT. Return false on NACK. */
    bool     (*i2c_write)(void *ctx, const uint8_t *buf, size_t len);
    bool     (*i2c_read)(void *ctx, uint8_t *buf, size_t len);
    uint32_t (*now_ms)(void *ctx);
    /* Called once per job when it finishes (success or failure). */
    void     (*on_result)(void *ctx, const switch_result_t *result);
} switcher_port_t;

void switcher_init(const switcher_port_t *port);
bool switcher_request(uint8_t monitor, uint16_t input_code);
void switcher_poll(void);
bool switcher_busy(void);

const char *switcher_status_str(switch_status_t s);

#endif /* SWITCHER_H */
