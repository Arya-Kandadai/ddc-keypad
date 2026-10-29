/**
 * sw_i2c.h - Bit-banged (software) I2C master on two open-drain GPIOs.
 *
 * Why: the DDC/CI bus only needs ~100 kHz, a handful of bytes per command,
 * and must tolerate slow displays (clock stretching). A software master
 * works on any two GPIOs, so each monitor gets its own bus (they all use
 * address 0x37, so they can't share one). It also avoids a stall seen with
 * the STM32C0 I2C peripheral model in the Wokwi simulator.
 *
 * Lines are open-drain: "release" = drive 1 (high-Z, pulled up),
 * "pull" = drive 0. External 4.7 k pull-ups are needed on real hardware.
 */
#ifndef SW_I2C_H
#define SW_I2C_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef struct {
    void     *ctx;                /* passed to the pin callbacks, e.g. which pins */
    void     (*scl_write)(void *ctx, bool high);
    void     (*sda_write)(void *ctx, bool high);
    bool     (*scl_read)(void *ctx);
    bool     (*sda_read)(void *ctx);
    void     (*delay_us)(uint32_t us);
    uint32_t half_period_us;      /* 5 -> ~100 kHz                       */
    uint32_t stretch_timeout_us;  /* max time a slave may hold SCL low   */
} sw_i2c_t;

/* Free the bus if a slave is stuck mid-byte (9 clocks + STOP). */
void sw_i2c_recover(const sw_i2c_t *bus);

/* Full transactions to a 7-bit address. Return false on any NACK/timeout. */
bool sw_i2c_write(const sw_i2c_t *bus, uint8_t addr7, const uint8_t *buf, size_t len);
bool sw_i2c_read(const sw_i2c_t *bus, uint8_t addr7, uint8_t *buf, size_t len);

#endif /* SW_I2C_H */
