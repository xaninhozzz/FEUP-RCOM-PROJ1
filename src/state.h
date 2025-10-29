#ifndef _STATE_H_
#define _STATE_H_

#include <stdint.h>
#include <stdbool.h>

// ---------------------------------------------------------
// FLAG (0x7E), ESC (0x7D), etc. 
// Address field (A) and Control field (C) macros are assumed available.

// ---------------------------------------------------------
// ENUM: State
// ---------------------------------------------------------
typedef enum {
    STATE_START,        // Waiting for first FLAG
    STATE_FLAG_RCV,     // FLAG received, expecting address
    STATE_A_RCV,        // Address field received
    STATE_C_RCV,        // Control field received
    STATE_BCC1_OK,      // BCC1 verified, entering data
    STATE_DATA,         // Reading raw (stuffed) data bytes (info frame)
    STATE_STOP,         // End FLAG received, frame complete
    STATE_ERROR           // Invalid frame detected
} State;

// ---------------------------------------------------------
// FUNCTIONS
// ---------------------------------------------------------

/**
 * @brief Advances the state machine for parsing I frames (frames with data).
 * 
 * @param state         Current FSM state.
 * @param byte          New byte received from serial input.
 * @param addressField  Expected address (A) field.
 * @param frameNumber   Output: frame number (0 or 1) if found.
 * @param buffer        Buffer to store raw frame data (stuffed bytes).
 * @param index         Pointer to current index in buffer (will be updated).
 * @param maxSize       Maximum allowed buffer size.
 * @return              Updated FSM state.
 */
State getIState(State state, uint8_t byte, uint8_t addressField,
                      uint8_t *frameNumber, uint8_t *buffer, int *index, int maxSize);


/**
 * @brief Advances the state machine for parsing S or U frames (no data).
 * 
 * @param state         Current FSM state.
 * @param byte          New byte received from serial input.
 * @param addressField  Expected address (A) field.
 * @param controlField  Output: control byte read from frame.
 * @return              Updated FSM state.
 */
State getSOrUState(State state, uint8_t byte, uint8_t addressField, uint8_t *controlField);


#endif // _STATE_H_