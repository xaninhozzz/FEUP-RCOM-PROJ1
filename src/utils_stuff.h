#ifndef UTILS_STUFF_H
#define UTILS_STUFF_H

#include <stdint.h>

int stuff_buffer(const uint8_t *in, int inlen, uint8_t *out, int outcap);
int unstuff_buffer(const uint8_t *in, int inlen, uint8_t *out, int outcap);
uint8_t compute_bcc2(const uint8_t *buf, int len);

#endif // UTILS_STUFF_H 