/**
 * ddc.h - DDC/CI (VESA Display Data Channel Command Interface) packet layer.
 *
 * Pure C, no hardware dependencies: builds and parses the byte streams that
 * travel over the monitor's I2C bus. The I2C transfer itself is done by the
 * caller (see switcher.h for the transport interface).
 *
 * Wire format (host -> display), written to 7-bit address 0x37:
 *   [0x51 src] [0x80 | n] [n data bytes] [checksum]
 *   checksum = 0x6E ^ (XOR of every byte before it)     (0x6E = 0x37 << 1)
 *
 * Wire format (display -> host), read from address 0x37:
 *   [0x6E src] [0x80 | n] [n data bytes] [checksum]
 *   checksum = 0x50 ^ (XOR of every byte before it)     (0x50 = "virtual host")
 */
#ifndef DDC_H
#define DDC_H

#include <stdint.h>
#include <stddef.h>

#define DDC_I2C_ADDR_7BIT     0x37u   /* DDC/CI display address            */
#define DDC_ADDR_DISPLAY_W    0x6Eu   /* 0x37 << 1, used in checksums      */
#define DDC_ADDR_HOST_SRC     0x51u   /* source address byte from host     */
#define DDC_ADDR_VHOST        0x50u   /* checksum seed for display replies */
#define DDC_LEN_FLAG          0x80u

/* Command opcodes (first data byte) */
#define DDC_CMD_GET_VCP       0x01u
#define DDC_CMD_GET_VCP_REPLY 0x02u
#define DDC_CMD_SET_VCP       0x03u

/* MCCS VCP feature codes */
#define VCP_INPUT_SOURCE      0x60u

/* Common MCCS input-source values (vendors vary - check your monitor) */
#define INPUT_DP1             0x0Fu
#define INPUT_DP2             0x10u
#define INPUT_HDMI1           0x11u
#define INPUT_HDMI2           0x12u
#define INPUT_USBC            0x1Bu

/* Spec timing: host must wait this long after a write before the next
 * transaction (set) or before reading the reply (get). */
#define DDC_SET_DELAY_MS      50u
#define DDC_REPLY_DELAY_MS    40u

#define DDC_SET_VCP_LEN       7u      /* bytes written for Set VCP          */
#define DDC_GET_VCP_LEN       5u      /* bytes written for Get VCP request  */
#define DDC_VCP_REPLY_LEN     11u     /* bytes read for Get VCP reply       */

typedef enum {
    DDC_OK = 0,
    DDC_ERR_NULL_MSG,        /* display replied with "null message" (not ready) */
    DDC_ERR_BAD_SOURCE,      /* first byte was not 0x6E                       */
    DDC_ERR_BAD_LENGTH,      /* length byte missing flag or wrong size         */
    DDC_ERR_BAD_CHECKSUM,
    DDC_ERR_BAD_REPLY,       /* not a Get VCP reply, or wrong opcode echoed     */
    DDC_ERR_UNSUPPORTED,     /* display says VCP code is unsupported            */
    DDC_ERR_ARG
} ddc_status_t;

typedef struct {
    uint8_t  opcode;     /* VCP code the reply is for          */
    uint8_t  type;       /* 0 = set parameter, 1 = momentary   */
    uint16_t max;        /* maximum value                       */
    uint16_t present;    /* current value                       */
} ddc_vcp_reply_t;

/** XOR checksum over len bytes, starting from seed. */
uint8_t ddc_checksum(uint8_t seed, const uint8_t *buf, size_t len);

/** Build a Set VCP Feature packet. buf must hold DDC_SET_VCP_LEN bytes.
 *  Returns number of bytes to write. */
size_t ddc_build_set_vcp(uint8_t *buf, uint8_t vcp_code, uint16_t value);

/** Build a Get VCP Feature request. buf must hold DDC_GET_VCP_LEN bytes. */
size_t ddc_build_get_vcp(uint8_t *buf, uint8_t vcp_code);

/** Parse a Get VCP Feature reply (DDC_VCP_REPLY_LEN bytes read from 0x37). */
ddc_status_t ddc_parse_vcp_reply(const uint8_t *buf, size_t len,
                                 uint8_t expected_vcp, ddc_vcp_reply_t *out);

const char *ddc_status_str(ddc_status_t s);
const char *ddc_input_name(uint16_t input_code);

#endif /* DDC_H */
