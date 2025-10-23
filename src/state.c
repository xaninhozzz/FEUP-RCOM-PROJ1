#include "state.h"
#include "special_bytes.h"
#include <stdio.h>   // optional, for debugging/logging

// ---------------------------------------------------------
// Helper: check if a control field corresponds to a valid S or U frame
// ---------------------------------------------------------
bool isValidSOrUFrameControl(uint8_t byte) {
    return (byte == SET)  ||
           (byte == UA)   ||
           (byte == DISC) ||
           (byte == RR(0)) ||
           (byte == RR(1)) ||
           (byte == REJ(0)) ||
           (byte == REJ(1));
}

// ---------------------------------------------------------
// Helper: return true if current state is within data reading
// ---------------------------------------------------------
bool isDataState(State state) {
    return state == STATE_DATA;
}

// ---------------------------------------------------------
// FSM: for Supervisory or Unnumbered frames (SET, UA, DISC, RR, REJ)
// ---------------------------------------------------------
State nextSOrUFrameState(State state, uint8_t byte, uint8_t addressField, uint8_t *controlField) {
    switch (state) {

        case STATE_START:
            if (byte == FLAG)
                state = STATE_FLAG_RCV;
            break;

        case STATE_FLAG_RCV:
            if (byte == addressField)
                state = STATE_A_RCV;
            else if (byte != FLAG)
                state = STATE_START;
            break;

        case STATE_A_RCV:
            if (isValidSOrUFrameControl(byte)) {
                *controlField = byte;
                state = STATE_C_RCV;
            } else if (byte == FLAG) {
                state = STATE_FLAG_RCV;
            } else {
                state = STATE_START;
            }
            break;

        case STATE_C_RCV:
            if (byte == (addressField ^ *controlField))
                state = STATE_BCC1_OK;
            else if (byte == FLAG)
                state = STATE_FLAG_RCV;
            else
                state = STATE_START;
            break;

        case STATE_BCC1_OK:
            if (byte == FLAG)
                state = STATE_STOP;
            else
                state = STATE_START;
            break;

        default:
            break;
    }
    return state;
}

// ---------------------------------------------------------
// FSM: for Information (I) frames (data-carrying)
// the BCC2 (XOR over unstuffed payload) is verified in llread()
// ---------------------------------------------------------
State nextIFrameState(State state, uint8_t byte, uint8_t addressField,
                      uint8_t *frameNumber, uint8_t *buffer, int *index, int maxSize)
{
    switch (state) {

        case STATE_START:
            if (byte == FLAG)
                state = STATE_FLAG_RCV;
            break;

        case STATE_FLAG_RCV:
            if (byte == addressField)
                state = STATE_A_RCV;
            else if (byte != FLAG)
                state = STATE_START;
            break;

        case STATE_A_RCV:
            if (byte == C(0)) {
                *frameNumber = 0;
                state = STATE_C_RCV;
            } else if (byte == C(1)) {
                *frameNumber = 1;
                state = STATE_C_RCV;
            } else if (byte == FLAG) {
                state = STATE_FLAG_RCV;
            } else {
                state = STATE_START;
            }
            break;

        case STATE_C_RCV:
            if (byte == (addressField ^ C(*frameNumber)))
                state = STATE_BCC1_OK;
            else if (byte == FLAG)
                state = STATE_FLAG_RCV;
            else
                state = STATE_START;
            break;

        case STATE_BCC1_OK:
            // Transition into DATA only when we get the first byte after header
            if (byte == FLAG) {
                // empty data field — should not happen, restart
                state = STATE_FLAG_RCV;
            } else {
                // store the first byte of the (stuffed) data stream
                if (*index < maxSize) {
                    buffer[*index] = byte;
                    (*index)++;
                    state = STATE_DATA;
                } else {
                    state = STATE_BAD;
                }
            }
            break;

        case STATE_DATA:
            if (byte == FLAG) {
                state = STATE_STOP; // end of frame
            } else {
                if (*index < maxSize) {
                    buffer[*index] = byte;
                    (*index)++;
                } else {
                    state = STATE_BAD; // buffer overflow
                }
            }
            break;

        default:
            break;
    }

    return state;
}