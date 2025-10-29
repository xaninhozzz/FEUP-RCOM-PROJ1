#include "utils_stuff.h"
#include "macros.h"
#include <stddef.h>

/* Stuff bytes: ESC + (byte ^ STUFF_XOR) for FLAG or ESC */
int stuff_buffer(const uint8_t *in, int inlen, uint8_t *out, int outcap)
{
    if (!in || !out || inlen < 0) return -1;
    int pos = 0;
    for (int i = 0; i < inlen; ++i) {
        uint8_t b = in[i];
        if (b == FLAG || b == ESC) {
            if (pos + 2 > outcap) return -1;
            out[pos++] = ESC;
            out[pos++] = (uint8_t)(b ^ STUFF_XOR);
        } else {
            if (pos + 1 > outcap) return -1;
            out[pos++] = b;
        }
    }
    return pos;
}

/* Un-stuff bytes: ESC followed by (byte ^ STUFF_XOR) */
int unstuff_buffer(const uint8_t *in, int inlen, uint8_t *out, int outcap)
{
    if (!in || !out || inlen < 0) return -1;
    int pos = 0;
    for (int i = 0; i < inlen; ++i) {
        uint8_t b = in[i];
        if (b == ESC) {
            /* ESC must be followed by another byte */
            if (i + 1 >= inlen) return -1; /* malformed */
            uint8_t next = in[++i];
            uint8_t orig = (uint8_t)(next ^ STUFF_XOR);
            if (pos + 1 > outcap) return -1;
            out[pos++] = orig;
        } else {
            if (pos + 1 > outcap) return -1;
            out[pos++] = b;
        }
    }
    return pos;
}

uint8_t compute_bcc2(const uint8_t *buf, int len)
{
    uint8_t bcc = 0x00;
    if (!buf || len <= 0) return bcc;
    for (int i = 0; i < len; ++i) bcc ^= buf[i];
    return bcc;
}