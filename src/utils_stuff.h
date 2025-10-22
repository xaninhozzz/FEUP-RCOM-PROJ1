#ifndef UTILS_STUFF_H
#define UTILS_STUFF_H

#include <stdint.h>

/*
 * Stuffing helpers
 *
 * stuff_buffer:
 *   - in: input bytes (payload + bcc2, un-stuffed)
 *   - inlen: input length
 *   - out: output buffer (must be large enough)
 *   - outcap: capacity of out
 *   - returns number of bytes written to out, or -1 on overflow
 *
 * unstuff_buffer:
 *   - in: input bytes (stuffed payload)
 *   - inlen: length of input
 *   - out: output buffer (un-stuffed)
 *   - outcap: capacity of out
 *   - returns number of bytes written to out, or -1 on malformed/overflow
 *
 * compute_bcc2:
 *   - XOR of all bytes in buffer (0 if len==0)
 */
int stuff_buffer(const uint8_t *in, int inlen, uint8_t *out, int outcap);
int unstuff_buffer(const uint8_t *in, int inlen, uint8_t *out, int outcap);
uint8_t compute_bcc2(const uint8_t *buf, int len);

#endif /* UTILS_STUFF_H */