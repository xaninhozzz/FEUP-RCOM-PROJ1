#ifndef _STATE_H_
#define _STATE_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    STATE_START,        // Waiting for first FLAG
    STATE_FLAG_RCV,     // FLAG received, expecting address
    STATE_A_RCV,        // Address field received
    STATE_C_RCV,        // Control field received
    STATE_BCC1_OK,      // BCC1 verified, entering data
    STATE_DATA,         // Reading raw (stuffed) data bytes (info frame)
    STATE_STOP,         // End FLAG received, frame complete
    STATE_ERROR         // Invalid frame detected
} State;


State getIState(State state, uint8_t byte, uint8_t addressField, uint8_t *frameNumber, uint8_t *buffer, int *index, int maxSize);

State getSOrUState(State state, uint8_t byte, uint8_t addressField, uint8_t *controlField);


#endif // _STATE_H_