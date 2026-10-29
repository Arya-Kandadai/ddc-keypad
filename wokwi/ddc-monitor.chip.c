/**
 * ddc-monitor.chip.c - Wokwi custom chip: a monitor's DDC/CI interface.
 *
 * Behaves like the I2C side of a real monitor at address 0x37:
 *   - validates header, length and checksum of every host packet
 *   - Set VCP 0x60 (input source) changes the displayed input
 *   - Get VCP replies with the current value (unsupported codes -> result 1)
 *   - enforces DDC/CI timing: NACKs writes for 50 ms after a Set VCP, and
 *     returns a "null message" if the host reads a reply sooner than 40 ms
 *   - "Fault injection" slider: random NACKs and corrupted reply checksums,
 *     to exercise the firmware's retry logic
 *
 * Attributes (set in diagram.json "attrs"):
 *   monitorId     number shown on screen (default 1)
 *   initialInput  starting input code (default 15 = 0x0F = DP1)
 */
#include "wokwi-api.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DDC_ADDR          0x37
#define ADDR_DISPLAY_W    0x6E
#define ADDR_HOST_SRC     0x51
#define ADDR_VHOST        0x50
#define VCP_INPUT_SOURCE  0x60
#define SET_BUSY_NS       (50ull * 1000000ull)
#define REPLY_DELAY_NS    (40ull * 1000000ull)

#define FB_W 128
#define FB_H 64

typedef struct {
  uint32_t error_attr;
  uint32_t monitor_id;

  buffer_t fb;
  uint32_t fb_w, fb_h;
  uint8_t  px[FB_W * FB_H * 4];

  /* host -> display */
  uint8_t  rx[32];
  uint8_t  rx_len;
  bool     rx_overflow;
  bool     reading;

  /* display -> host */
  uint8_t  reply[11];
  bool     have_reply;
  uint64_t reply_ready_ns;
  const uint8_t *serve;
  uint8_t  serve_len;
  uint8_t  serve_idx;
  bool     corrupt_reply;

  uint64_t busy_until_ns;
  uint16_t input;

  uint32_t rx_count, err_count, nack_count;
  char     last_event[24];
  uint32_t rng;
} chip_state_t;

static const uint8_t NULL_MSG[3] = { ADDR_DISPLAY_W, 0x80, 0xBE };

/* ---------------------------------------------------------------- font */
/* Classic 5x7 font, ASCII 0x20..0x5A, column-major, bit 0 = top row. */
static const uint8_t FONT5X7[][5] = {
  {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5F,0x00,0x00}, {0x00,0x07,0x00,0x07,0x00}, /* sp ! " */
  {0x14,0x7F,0x14,0x7F,0x14}, {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62}, /* # $ % */
  {0x36,0x49,0x55,0x22,0x50}, {0x00,0x05,0x03,0x00,0x00}, {0x00,0x1C,0x22,0x41,0x00}, /* & ' ( */
  {0x00,0x41,0x22,0x1C,0x00}, {0x08,0x2A,0x1C,0x2A,0x08}, {0x08,0x08,0x3E,0x08,0x08}, /* ) * + */
  {0x00,0x50,0x30,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08}, {0x00,0x60,0x60,0x00,0x00}, /* , - . */
  {0x20,0x10,0x08,0x04,0x02}, {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00}, /* / 0 1 */
  {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31}, {0x18,0x14,0x12,0x7F,0x10}, /* 2 3 4 */
  {0x27,0x45,0x45,0x45,0x39}, {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03}, /* 5 6 7 */
  {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E}, {0x00,0x36,0x36,0x00,0x00}, /* 8 9 : */
  {0x00,0x56,0x36,0x00,0x00}, {0x00,0x08,0x14,0x22,0x41}, {0x14,0x14,0x14,0x14,0x14}, /* ; < = */
  {0x41,0x22,0x14,0x08,0x00}, {0x02,0x01,0x51,0x09,0x06}, {0x32,0x49,0x79,0x41,0x3E}, /* > ? @ */
  {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22}, /* A B C */
  {0x7F,0x41,0x41,0x22,0x1C}, {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x01,0x01}, /* D E F */
  {0x3E,0x41,0x41,0x51,0x32}, {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00}, /* G H I */
  {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41}, {0x7F,0x40,0x40,0x40,0x40}, /* J K L */
  {0x7F,0x02,0x04,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E}, /* M N O */
  {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46}, /* P Q R */
  {0x46,0x49,0x49,0x49,0x31}, {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F}, /* S T U */
  {0x1F,0x20,0x40,0x20,0x1F}, {0x7F,0x20,0x18,0x20,0x7F}, {0x63,0x14,0x08,0x14,0x63}, /* V W X */
  {0x03,0x04,0x78,0x04,0x03}, {0x61,0x51,0x49,0x45,0x43},                              /* Y Z   */
};

/* ------------------------------------------------------------- drawing */
static void fill(chip_state_t *c, uint8_t r, uint8_t g, uint8_t b)
{
  for (uint32_t i = 0; i < FB_W * FB_H; i++) {
    c->px[i * 4 + 0] = r;
    c->px[i * 4 + 1] = g;
    c->px[i * 4 + 2] = b;
    c->px[i * 4 + 3] = 0xFF;
  }
}

static void put_px(chip_state_t *c, int x, int y, uint8_t r, uint8_t g, uint8_t b)
{
  if (x < 0 || y < 0 || x >= FB_W || y >= FB_H) return;
  uint8_t *p = &c->px[(y * FB_W + x) * 4];
  p[0] = r; p[1] = g; p[2] = b; p[3] = 0xFF;
}

static void draw_text(chip_state_t *c, int x, int y, const char *s, int scale,
                      uint8_t r, uint8_t g, uint8_t b)
{
  for (; *s; s++, x += 6 * scale) {
    char ch = *s;
    if (ch >= 'a' && ch <= 'z') ch = (char)(ch - 'a' + 'A');
    if (ch < 0x20 || ch > 0x5A) ch = '?';
    const uint8_t *glyph = FONT5X7[ch - 0x20];
    for (int col = 0; col < 5; col++) {
      for (int row = 0; row < 7; row++) {
        if (glyph[col] & (1u << row)) {
          for (int dx = 0; dx < scale; dx++)
            for (int dy = 0; dy < scale; dy++)
              put_px(c, x + col * scale + dx, y + row * scale + dy, r, g, b);
        }
      }
    }
  }
}

static const char *input_name(uint16_t code)
{
  switch (code) {
    case 0x0F: return "DP 1";
    case 0x10: return "DP 2";
    case 0x11: return "HDMI 1";
    case 0x12: return "HDMI 2";
    case 0x1B: return "USB-C";
    default:   return NULL;
  }
}

static void render(chip_state_t *c)
{
  /* background colour per input so a switch is obvious at a glance */
  switch (c->input) {
    case 0x0F: fill(c, 0x10, 0x30, 0x90); break;   /* DP1   blue   */
    case 0x10: fill(c, 0x50, 0x20, 0x80); break;   /* DP2   purple */
    case 0x11: fill(c, 0x10, 0x60, 0x20); break;   /* HDMI1 green  */
    case 0x12: fill(c, 0x00, 0x55, 0x60); break;   /* HDMI2 teal   */
    case 0x1B: fill(c, 0x90, 0x40, 0x00); break;   /* USB-C orange */
    default:   fill(c, 0x30, 0x30, 0x30); break;
  }

  char line[32];
  snprintf(line, sizeof line, "MONITOR %u", (unsigned)c->monitor_id);
  draw_text(c, 2, 2, line, 1, 0xFF, 0xFF, 0xFF);

  const char *name = input_name(c->input);
  draw_text(c, 2, 14, name ? name : "???", 2, 0xFF, 0xFF, 0xFF);

  snprintf(line, sizeof line, "VCP60=0X%02X", c->input);
  draw_text(c, 2, 33, line, 1, 0xE0, 0xE0, 0xE0);
  draw_text(c, 2, 44, c->last_event, 1, 0xFF, 0xFF, 0x80);

  snprintf(line, sizeof line, "RX%u ERR%u NACK%u",
           (unsigned)c->rx_count, (unsigned)c->err_count, (unsigned)c->nack_count);
  draw_text(c, 2, 55, line, 1, 0xC0, 0xC0, 0xC0);

  buffer_write(c->fb, 0, c->px, sizeof c->px);
}

/* ------------------------------------------------------------- helpers */
static uint32_t roll_percent(chip_state_t *c)
{
  c->rng = c->rng * 1103515245u + 12345u;
  return ((c->rng >> 16) & 0x7FFF) % 100u;
}

static bool inject_fault(chip_state_t *c)
{
  uint32_t rate = attr_read(c->error_attr);       /* 0..50 % per transaction */
  return rate > 0 && roll_percent(c) < rate;
}

static uint8_t xor_sum(uint8_t seed, const uint8_t *b, size_t n)
{
  for (size_t i = 0; i < n; i++) seed ^= b[i];
  return seed;
}

static void event(chip_state_t *c, const char *msg, bool is_error)
{
  snprintf(c->last_event, sizeof c->last_event, "%s", msg);
  if (is_error) c->err_count++;
}

/* ------------------------------------------------------- DDC/CI logic */
static void build_vcp_reply(chip_state_t *c, uint8_t vcp, uint64_t now)
{
  bool supported = (vcp == VCP_INPUT_SOURCE);
  uint16_t value = supported ? c->input : 0;
  uint8_t *r = c->reply;
  r[0] = ADDR_DISPLAY_W;
  r[1] = 0x88;                      /* 8 data bytes follow */
  r[2] = 0x02;                      /* Get VCP Feature reply */
  r[3] = supported ? 0x00 : 0x01;   /* result: 0 ok, 1 unsupported */
  r[4] = vcp;
  r[5] = 0x00;                      /* type: set parameter */
  r[6] = 0x00;                      /* max value hi/lo */
  r[7] = 0x1B;
  r[8] = (uint8_t)(value >> 8);
  r[9] = (uint8_t)(value & 0xFF);
  r[10] = xor_sum(ADDR_VHOST, r, 10);
  c->have_reply = true;
  c->reply_ready_ns = now + REPLY_DELAY_NS;
}

static void process_packet(chip_state_t *c)
{
  uint64_t now = get_sim_nanos();
  const uint8_t *p = c->rx;
  char msg[24];

  c->rx_count++;

  if (c->rx_overflow || c->rx_len < 3 || p[0] != ADDR_HOST_SRC || !(p[1] & 0x80)) {
    event(c, "BAD HEADER", true);
    return;
  }
  uint8_t n = p[1] & 0x7F;
  if (c->rx_len != n + 3) {
    event(c, "BAD LENGTH", true);
    return;
  }
  if (xor_sum(ADDR_DISPLAY_W, p, n + 2) != p[n + 2]) {
    event(c, "BAD CHECKSUM", true);
    return;
  }

  const uint8_t *d = &p[2];
  if (d[0] == 0x03 && n == 4) {                         /* Set VCP Feature */
    uint8_t  vcp   = d[1];
    uint16_t value = (uint16_t)((d[2] << 8) | d[3]);
    c->have_reply = false;
    c->busy_until_ns = now + SET_BUSY_NS;               /* "switching..." */
    if (vcp != VCP_INPUT_SOURCE) {
      snprintf(msg, sizeof msg, "SET %02X IGNORED", vcp);
      event(c, msg, false);
    } else if (!input_name(value)) {
      snprintf(msg, sizeof msg, "BAD INPUT %02X", value & 0xFF);
      event(c, msg, true);
    } else {
      c->input = value;
      snprintf(msg, sizeof msg, "SET %s", input_name(value));
      event(c, msg, false);
    }
  } else if (d[0] == 0x01 && n == 2) {                  /* Get VCP Feature */
    build_vcp_reply(c, d[1], now);
    snprintf(msg, sizeof msg, "GET %02X", d[1]);
    event(c, msg, false);
  } else {
    snprintf(msg, sizeof msg, "UNKNOWN CMD %02X", d[0]);
    event(c, msg, true);
  }
}

/* --------------------------------------------------------- I2C events */
static bool on_i2c_connect(void *user_data, uint32_t address, bool read)
{
  chip_state_t *c = user_data;
  (void)address;

  uint64_t now = get_sim_nanos();

  if (!read && now < c->busy_until_ns) {
    c->nack_count++;
    event(c, "BUSY: NACK", false);
    render(c);
    return false;
  }
  if (inject_fault(c)) {
    c->nack_count++;
    event(c, "INJECTED NACK", false);
    render(c);
    return false;
  }

  c->reading = read;
  if (read) {
    if (c->have_reply && now >= c->reply_ready_ns) {
      c->serve = c->reply;
      c->serve_len = sizeof c->reply;
      c->corrupt_reply = inject_fault(c);
    } else {
      c->serve = NULL_MSG;          /* not ready yet (or nothing asked) */
      c->serve_len = sizeof NULL_MSG;
      c->corrupt_reply = false;
      event(c, "READ TOO EARLY", false);
    }
    c->serve_idx = 0;
  } else {
    c->rx_len = 0;
    c->rx_overflow = false;
  }
  return true;
}

static uint8_t on_i2c_read(void *user_data)
{
  chip_state_t *c = user_data;
  if (c->serve_idx >= c->serve_len) {
    return 0xFF;                    /* past the end: idle bus */
  }
  uint8_t b = c->serve[c->serve_idx];
  if (c->corrupt_reply && c->serve_idx == c->serve_len - 1) {
    b ^= 0x5A;                      /* break the checksum */
    event(c, "INJECTED BAD CHK", false);
  }
  c->serve_idx++;
  return b;
}

static bool on_i2c_write(void *user_data, uint8_t data)
{
  chip_state_t *c = user_data;
  if (c->rx_len < sizeof c->rx) {
    c->rx[c->rx_len++] = data;
  } else {
    c->rx_overflow = true;
  }
  return true;
}

static void on_i2c_disconnect(void *user_data)
{
  chip_state_t *c = user_data;
  if (!c->reading && c->rx_len > 0) {
    process_packet(c);
    c->rx_len = 0;
  }
  render(c);
}

/* ---------------------------------------------------------------- init */
void chip_init(void)
{
  chip_state_t *c = calloc(1, sizeof(chip_state_t));

  c->error_attr = attr_init("errorRate", 0);
  c->monitor_id = attr_read(attr_init("monitorId", 1));
  c->input      = (uint16_t)attr_read(attr_init("initialInput", 0x0F));
  c->rng        = 0xC0FFEEu + c->monitor_id;
  snprintf(c->last_event, sizeof c->last_event, "READY");

  const i2c_config_t i2c = {
    .user_data  = c,
    .address    = DDC_ADDR,
    .scl        = pin_init("SCL", INPUT_PULLUP),
    .sda        = pin_init("SDA", INPUT_PULLUP),
    .connect    = on_i2c_connect,
    .read       = on_i2c_read,
    .write      = on_i2c_write,
    .disconnect = on_i2c_disconnect,
  };
  i2c_init(&i2c);

  c->fb = framebuffer_init(&c->fb_w, &c->fb_h);
  render(c);
}
