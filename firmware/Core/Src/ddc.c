/**
 * ddc.c - DDC/CI packet build/parse. See ddc.h for the wire format.
 */
#include "ddc.h"

uint8_t ddc_checksum(uint8_t seed, const uint8_t *buf, size_t len)
{
    uint8_t c = seed;
    for (size_t i = 0; i < len; i++) {
        c ^= buf[i];
    }
    return c;
}

size_t ddc_build_set_vcp(uint8_t *buf, uint8_t vcp_code, uint16_t value)
{
    if (buf == NULL) {
        return 0;
    }
    buf[0] = DDC_ADDR_HOST_SRC;
    buf[1] = DDC_LEN_FLAG | 4u;
    buf[2] = DDC_CMD_SET_VCP;
    buf[3] = vcp_code;
    buf[4] = (uint8_t)(value >> 8);
    buf[5] = (uint8_t)(value & 0xFFu);
    buf[6] = ddc_checksum(DDC_ADDR_DISPLAY_W, buf, 6);
    return DDC_SET_VCP_LEN;
}

size_t ddc_build_get_vcp(uint8_t *buf, uint8_t vcp_code)
{
    if (buf == NULL) {
        return 0;
    }
    buf[0] = DDC_ADDR_HOST_SRC;
    buf[1] = DDC_LEN_FLAG | 2u;
    buf[2] = DDC_CMD_GET_VCP;
    buf[3] = vcp_code;
    buf[4] = ddc_checksum(DDC_ADDR_DISPLAY_W, buf, 4);
    return DDC_GET_VCP_LEN;
}

ddc_status_t ddc_parse_vcp_reply(const uint8_t *buf, size_t len,
                                 uint8_t expected_vcp, ddc_vcp_reply_t *out)
{
    if (buf == NULL || out == NULL || len < 3) {
        return DDC_ERR_ARG;
    }
    if (buf[0] != DDC_ADDR_DISPLAY_W) {
        return DDC_ERR_BAD_SOURCE;
    }
    if ((buf[1] & DDC_LEN_FLAG) == 0) {
        return DDC_ERR_BAD_LENGTH;
    }

    size_t n = buf[1] & 0x7Fu;
    if (n == 0) {
        /* Null message: 6E 80 BE. Display is busy / has nothing to say. */
        return DDC_ERR_NULL_MSG;
    }
    if (n != 8 || len < 2 + n + 1) {
        return DDC_ERR_BAD_LENGTH;
    }
    if (ddc_checksum(DDC_ADDR_VHOST, buf, 2 + n) != buf[2 + n]) {
        return DDC_ERR_BAD_CHECKSUM;
    }

    const uint8_t *d = &buf[2];
    if (d[0] != DDC_CMD_GET_VCP_REPLY) {
        return DDC_ERR_BAD_REPLY;
    }
    if (d[1] != 0x00u) {           /* result code: 0 = no error */
        return DDC_ERR_UNSUPPORTED;
    }
    if (d[2] != expected_vcp) {
        return DDC_ERR_BAD_REPLY;
    }

    out->opcode  = d[2];
    out->type    = d[3];
    out->max     = (uint16_t)((d[4] << 8) | d[5]);
    out->present = (uint16_t)((d[6] << 8) | d[7]);
    return DDC_OK;
}

const char *ddc_status_str(ddc_status_t s)
{
    switch (s) {
    case DDC_OK:               return "ok";
    case DDC_ERR_NULL_MSG:     return "null msg (display busy)";
    case DDC_ERR_BAD_SOURCE:   return "bad source byte";
    case DDC_ERR_BAD_LENGTH:   return "bad length";
    case DDC_ERR_BAD_CHECKSUM: return "bad checksum";
    case DDC_ERR_BAD_REPLY:    return "unexpected reply";
    case DDC_ERR_UNSUPPORTED:  return "VCP unsupported";
    case DDC_ERR_ARG:          return "bad argument";
    }
    return "?";
}

const char *ddc_input_name(uint16_t input_code)
{
    switch (input_code) {
    case INPUT_DP1:   return "DP1";
    case INPUT_DP2:   return "DP2";
    case INPUT_HDMI1: return "HDMI1";
    case INPUT_HDMI2: return "HDMI2";
    case INPUT_USBC:  return "USB-C";
    default:          return "unknown";
    }
}
