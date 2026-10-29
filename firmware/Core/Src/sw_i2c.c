/**
 * sw_i2c.c - see sw_i2c.h
 */
#include "sw_i2c.h"

static void half(const sw_i2c_t *b)
{
    b->delay_us(b->half_period_us);
}

/* Release SCL and wait for it to actually go high (the slave may stretch
 * the clock by holding it low). */
static bool scl_high(const sw_i2c_t *b)
{
    b->scl_write(b->ctx, true);
    uint32_t waited = 0;
    while (!b->scl_read(b->ctx)) {
        if (waited >= b->stretch_timeout_us) {
            return false;
        }
        b->delay_us(1);
        waited++;
    }
    return true;
}

static bool start(const sw_i2c_t *b)
{
    b->sda_write(b->ctx, true);
    if (!scl_high(b)) return false;
    half(b);
    b->sda_write(b->ctx, false);          /* SDA falls while SCL high = START */
    half(b);
    b->scl_write(b->ctx, false);
    half(b);
    return true;
}

static void stop(const sw_i2c_t *b)
{
    b->sda_write(b->ctx, false);
    half(b);
    scl_high(b);
    half(b);
    b->sda_write(b->ctx, true);           /* SDA rises while SCL high = STOP */
    half(b);
}

/* Returns true if the slave ACKed (pulled SDA low on the 9th clock). */
static bool write_byte(const sw_i2c_t *b, uint8_t v)
{
    for (int i = 7; i >= 0; i--) {
        b->sda_write(b->ctx, (v >> i) & 1u);
        half(b);
        if (!scl_high(b)) return false;
        half(b);
        b->scl_write(b->ctx, false);
    }
    b->sda_write(b->ctx, true);           /* release SDA for the ACK bit */
    half(b);
    if (!scl_high(b)) return false;
    half(b);
    bool ack = !b->sda_read(b->ctx);
    b->scl_write(b->ctx, false);
    return ack;
}

static uint8_t read_byte(const sw_i2c_t *b, bool send_ack)
{
    uint8_t v = 0;
    b->sda_write(b->ctx, true);
    for (int i = 0; i < 8; i++) {
        half(b);
        scl_high(b);
        half(b);
        v = (uint8_t)((v << 1) | (b->sda_read(b->ctx) ? 1u : 0u));
        b->scl_write(b->ctx, false);
    }
    b->sda_write(b->ctx, !send_ack);      /* ACK = pull low, NACK = release */
    half(b);
    scl_high(b);
    half(b);
    b->scl_write(b->ctx, false);
    b->sda_write(b->ctx, true);
    return v;
}

void sw_i2c_recover(const sw_i2c_t *b)
{
    b->sda_write(b->ctx, true);
    for (int i = 0; i < 9 && !b->sda_read(b->ctx); i++) {
        b->scl_write(b->ctx, false);
        half(b);
        scl_high(b);
        half(b);
    }
    stop(b);
}

bool sw_i2c_write(const sw_i2c_t *b, uint8_t addr7, const uint8_t *buf, size_t len)
{
    if (!start(b)) {
        sw_i2c_recover(b);
        return false;
    }
    bool ok = write_byte(b, (uint8_t)(addr7 << 1));
    for (size_t i = 0; ok && i < len; i++) {
        ok = write_byte(b, buf[i]);
    }
    stop(b);
    return ok;
}

bool sw_i2c_read(const sw_i2c_t *b, uint8_t addr7, uint8_t *buf, size_t len)
{
    if (!start(b)) {
        sw_i2c_recover(b);
        return false;
    }
    bool ok = write_byte(b, (uint8_t)((addr7 << 1) | 1u));
    for (size_t i = 0; ok && i < len; i++) {
        buf[i] = read_byte(b, i + 1 < len);   /* NACK the last byte */
    }
    stop(b);
    return ok;
}
