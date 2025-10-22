#include "link_layer.h"
#include "serial_port.h"
#include "state.h"
#include "special_bytes.h"
#include "utils_stuff.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <errno.h>
#include <stdlib.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

////////////////////////////////////////////////
// Globals / configuration saved from llopen
////////////////////////////////////////////////

static int fd = -1;
static LinkLayerRole role;
static int ll_timeout = 3;          // seconds
static int ll_nRetransmissions = 3; // attempts

// statistics
static int stat_retransmissions = 0;
static int stat_frames_sent = 0;
static int stat_frames_recv = 0;

////////////////////////////////////////////////
// Helper: read single byte with timeout (seconds).
// Returns 1 if byte read, 0 on timeout, -1 on error.
////////////////////////////////////////////////
static int read_byte_timeout(unsigned char *out, int timeout_seconds)
{
    if (fd < 0) return -1;

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);

    struct timeval tv;
    tv.tv_sec = timeout_seconds;
    tv.tv_usec = 0;

    int rv = select(fd + 1, &rfds, NULL, NULL, &tv);
    if (rv == -1) {
        return -1; // select error
    } else if (rv == 0) {
        return 0; // timeout
    } else {
        ssize_t r = read(fd, out, 1);
        if (r == 1) return 1;
        if (r == 0) return 0; // EOF
        return -1;
    }
}

////////////////////////////////////////////////
// Helper: send buffer using existing helper; fallback to write if helper fails
////////////////////////////////////////////////
static int send_bytes(const unsigned char *buf, int len)
{
    // try existing helper
    int w = writeBytesSerialPort((unsigned char*)buf, len);
    if (w == len) {
        stat_frames_sent++;
        return w;
    }
    // fallback direct write
    ssize_t r = write(fd, buf, len);
    if (r > 0) {
        stat_frames_sent++;
        return (int)r;
    }
    return -1;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////

int llopen(LinkLayer connectionParameters)
{
    fd = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate);
    if (fd < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    // save config
    role = connectionParameters.role;
    ll_timeout = connectionParameters.timeout > 0 ? connectionParameters.timeout : ll_timeout;
    ll_nRetransmissions = connectionParameters.nRetransmissions > 0 ? connectionParameters.nRetransmissions : ll_nRetransmissions;

    // Transmitter role
    if (role == LlTx)
    {
        printf("[TX] Sending SET frame...\n");

        unsigned char frame[5] = {
            FLAG,
            A_TX,
            SET,
            (unsigned char)(A_TX ^ SET),
            FLAG
        };

        int attempts = 0;
        while (attempts <= ll_nRetransmissions) {
            if (send_bytes(frame, 5) != 5) {
                perror("send SET");
                closeSerialPort();
                return -1;
            }

            // wait for UA (with timeout)
            State state = STATE_START;
            unsigned char byte = 0, control = 0;
            int keep_waiting = 1;
            while (keep_waiting) {
                int r = read_byte_timeout(&byte, ll_timeout);
                if (r == 1) {
                    state = nextSOrUFrameState(state, byte, A_RX, &control);
                    if (state == STATE_STOP) {
                        if (control == UA) {
                            printf("[TX] Connection established successfully!\n");
                            return 0;
                        } else {
                            // Unexpected control, ignore and continue waiting until timeout
                            state = STATE_START;
                        }
                    }
                    // continue reading until timeout or stop
                } else if (r == 0) {
                    // timeout => break to retransmit
                    break;
                } else {
                    // error
                    perror("read");
                    closeSerialPort();
                    return -1;
                }
            }

            // timeout or no valid UA, retransmit
            attempts++;
            stat_retransmissions++;
            if (attempts > ll_nRetransmissions) {
                fprintf(stderr, "[TX] SET retries exhausted\n");
                closeSerialPort();
                return -1;
            } else {
                printf("[TX] Retransmitting SET (attempt %d/%d)...\n", attempts, ll_nRetransmissions);
            }
        }

        // exhausted
        closeSerialPort();
        return -1;
    }

    // Receiver role
    else if (role == LlRx)
    {
        printf("[RX] Waiting for SET frame...\n");

        State state = STATE_START;
        unsigned char byte = 0, control = 0;

        // wait indefinitely for SET
        while (1) {
            ssize_t r = read(fd, &byte, 1);
            if (r == 1) {
                state = nextSOrUFrameState(state, byte, A_TX, &control);
                if (state == STATE_STOP) {
                    if (control == SET) {
                        unsigned char ua[5] = {
                            FLAG,
                            A_RX,
                            UA,
                            (unsigned char)(A_RX ^ UA),
                            FLAG
                        };
                        if (send_bytes(ua, 5) != 5) {
                            perror("send UA");
                            closeSerialPort();
                            return -1;
                        }
                        printf("[RX] Connection established successfully!\n");
                        return 0;
                    } else {
                        // ignore and continue
                        state = STATE_START;
                    }
                }
            } else if (r == 0) {
                // EOF on serial? treat as error
                perror("serial EOF");
                closeSerialPort();
                return -1;
            } else {
                perror("read");
                closeSerialPort();
                return -1;
            }
        }
    }

    return -1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////

int llwrite(const unsigned char *buf, int bufSize)
{
    static int frameNumber = 0;
    if (bufSize > MAX_PAYLOAD_SIZE) return -1;

    // compute BCC2
    unsigned char bcc2 = 0x00;
    for (int i = 0; i < bufSize; i++) bcc2 ^= buf[i];

    // build unstuffed payload: data || bcc2
    unsigned char tmp[MAX_PAYLOAD_SIZE + 1];
    memcpy(tmp, buf, bufSize);
    tmp[bufSize] = bcc2;

    // stuff payload + bcc2
    unsigned char stuffed[ (MAX_PAYLOAD_SIZE + 1) * 2 + 10 ]; // safe cap
    int stuffed_len = stuff_buffer(tmp, bufSize + 1, stuffed, sizeof(stuffed));
    if (stuffed_len < 0) return -1;

    // build frame
    int frame_len = 0;
    unsigned char *frame = (unsigned char*)malloc(4 + stuffed_len + 1);
    if (!frame) return -1;
    frame[frame_len++] = FLAG;
    frame[frame_len++] = A_TX;
    unsigned char Cbyte = C(frameNumber);
    frame[frame_len++] = Cbyte;
    frame[frame_len++] = (unsigned char)(A_TX ^ Cbyte);
    memcpy(frame + frame_len, stuffed, stuffed_len);
    frame_len += stuffed_len;
    frame[frame_len++] = FLAG;

    int attempts = 0;
    while (attempts <= ll_nRetransmissions) {
        if (send_bytes(frame, frame_len) != frame_len) {
            free(frame);
            return -1;
        }

        // wait for RR/REJ with timeout
        State state = STATE_START;
        unsigned char byte = 0, control = 0;
        int done = 0;
        while (!done) {
            int r = read_byte_timeout(&byte, ll_timeout);
            if (r == 1) {
                state = nextSOrUFrameState(state, byte, A_RX, &control);
                if (state == STATE_STOP) {
                    // check control type
                    // REJ?
                    if ((control & 0x7F) == 0x01) {
                        // REJ -> retransmit immediately (but count as retransmission)
                        stat_retransmissions++;
                        attempts++;
                        if (attempts > ll_nRetransmissions) {
                            free(frame);
                            return -1;
                        } else {
                            printf("[TX] Received REJ, retransmitting (attempt %d/%d)\n", attempts, ll_nRetransmissions);
                            done = 1; // break to outer loop and resend
                        }
                    }
                    // RR?
                    else if ((control & 0x7F) == 0x05) {
                        // extract r (MSB)
                        int rbit = (control & 0x80) ? 1 : 0;
                        int expected_ack = (frameNumber ^ 1);
                        if (rbit == expected_ack) {
                            // success
                            frameNumber = frameNumber ^ 1;
                            free(frame);
                            return bufSize;
                        } else {
                            // unexpected ack, ignore and keep waiting until timeout or valid ack
                            state = STATE_START;
                        }
                    } else {
                        // other control, ignore
                        state = STATE_START;
                    }
                }
                // else continue reading until timeout
            } else if (r == 0) {
                // timeout - retransmit
                attempts++;
                stat_retransmissions++;
                if (attempts > ll_nRetransmissions) {
                    free(frame);
                    return -1;
                } else {
                    printf("[TX] Timeout waiting RR, retransmitting (attempt %d/%d)\n", attempts, ll_nRetransmissions);
                    done = 1; // break to outer to resend
                }
            } else {
                // error
                free(frame);
                return -1;
            }
        }
    }

    free(frame);
    return -1;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    static int expectedFrame = 0; // receiver expects 0 first
    State state = STATE_START;
    unsigned char byte;
    unsigned char frameNumberRx = 0;
    unsigned char stuffed_buf[MAX_PAYLOAD_SIZE * 2 + 16];
    int idx = 0;

    while (1) {
        ssize_t r = read(fd, &byte, 1);
        if (r == 1) {
            // state = nextIFrameState(state, byte, A_RX, &frameNumberRx, stuffed_buf, &idx, sizeof(stuffed_buf));
            state = nextIFrameState(state, byte, A_TX, &frameNumberRx, stuffed_buf, &idx, sizeof(stuffed_buf));

            if (state == STATE_STOP) {
                stat_frames_recv++;

                // stuffed_buf[0..idx-1] contains stuffed payload + stuffed bcc2
                int un_len;
                unsigned char unstuffed[MAX_PAYLOAD_SIZE + 2];
                un_len = unstuff_buffer(stuffed_buf, idx, unstuffed, sizeof(unstuffed));
                if (un_len <= 0) {
                    // malformed/stuffing error => send REJ and continue
                    unsigned char rej[5] = {
                        FLAG,
                        A_RX,
                        REJ(expectedFrame),
                        (unsigned char)(A_RX ^ REJ(expectedFrame)),
                        FLAG
                    };
                    send_bytes(rej, 5);
                    // reset FSM
                    state = STATE_START;
                    idx = 0;
                    continue;
                }

                if (un_len < 1) {
                    // nothing to deliver -> ignore
                    state = STATE_START;
                    idx = 0;
                    continue;
                }

                int payload_len = un_len - 1;
                unsigned char received_bcc2 = unstuffed[un_len - 1];
                unsigned char calc_bcc2 = compute_bcc2(unstuffed, payload_len);

                if (calc_bcc2 != received_bcc2) {
                    // BCC2 mismatch -> REJ
                    unsigned char rej[5] = {
                        FLAG,
                        A_RX,
                        REJ(expectedFrame),
                        (unsigned char)(A_RX ^ REJ(expectedFrame)),
                        FLAG
                    };
                    send_bytes(rej, 5);
                    // reset and continue
                    state = STATE_START;
                    idx = 0;
                    continue;
                }

                // good frame
                if (frameNumberRx == expectedFrame) {
                    // deliver payload
                    for (int i = 0; i < payload_len; ++i) packet[i] = unstuffed[i];
                    // advance expected
                    expectedFrame = expectedFrame ^ 1;

                    // send RR with new expected (i.e., next expected)
                    unsigned char rr[5] = {
                        FLAG,
                        A_RX,
                        RR(expectedFrame),
                        (unsigned char)(A_RX ^ RR(expectedFrame)),
                        FLAG
                    };
                    send_bytes(rr, 5);

                    // reset state and idx
                    state = STATE_START;
                    idx = 0;

                    return payload_len;
                } else {
                    // duplicate frame - send RR for current expected and continue (don't deliver)
                    unsigned char rr[5] = {
                        FLAG,
                        A_RX,
                        RR(expectedFrame),
                        (unsigned char)(A_RX ^ RR(expectedFrame)),
                        FLAG
                    };
                    send_bytes(rr, 5);
                    state = STATE_START;
                    idx = 0;
                    continue;
                }
            }
            // if isDataState, do not append here; FSM already appended into stuffed_buf
        } else if (r == 0) {
            // EOF on serial
            return -1;
        } else {
            // error
            return -1;
        }
    }
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose()
{
    unsigned char disc_tx[5] = { FLAG, A_TX, DISC, (unsigned char)(A_TX^DISC), FLAG };
    unsigned char disc_rx[5] = { FLAG, A_RX, DISC, (unsigned char)(A_RX^DISC), FLAG };
    unsigned char ua_tx[5]   = { FLAG, A_TX, UA, (unsigned char)(A_TX^UA), FLAG };

    State state;
    unsigned char byte = 0, control = 0;

    if (role == LlTx) {
        // send DISC and wait for DISC from remote, retransmit on timeout
        int attempts = 0;
        while (attempts <= ll_nRetransmissions) {
            if (send_bytes(disc_tx, 5) != 5) {
                return -1;
            }
            // wait for DISC (from receiver)
            state = STATE_START;
            int got_disc = 0;
            while (1) {
                int r = read_byte_timeout(&byte, ll_timeout);
                if (r == 1) {
                    state = nextSOrUFrameState(state, byte, A_RX, &control);
                    if (state == STATE_STOP) {
                        if (control == DISC) {
                            got_disc = 1;
                            break;
                        } else {
                            state = STATE_START; // ignore other frames
                        }
                    }
                } else if (r == 0) {
                    // timeout
                    break;
                } else {
                    // error
                    closeSerialPort();
                    return -1;
                }
            }

            if (got_disc) {
                // send UA and finish
                if (send_bytes(ua_tx, 5) != 5) {
                    closeSerialPort();
                    return -1;
                }
                break;
            } else {
                attempts++;
                stat_retransmissions++;
                if (attempts > ll_nRetransmissions) {
                    closeSerialPort();
                    return -1;
                }
            }
        }
    } else { // Receiver role
        // wait for DISC from transmitter
        state = STATE_START;
        while (1) {
            ssize_t r = read(fd, &byte, 1);
            if (r == 1) {
                state = nextSOrUFrameState(state, byte, A_TX, &control);
                if (state == STATE_STOP && control == DISC) {
                    // send DISC back
                    if (send_bytes(disc_rx, 5) != 5) {
                        closeSerialPort();
                        return -1;
                    }
                    // now wait for UA
                    state = STATE_START;
                    while (1) {
                        ssize_t r2 = read(fd, &byte, 1);
                        if (r2 == 1) {
                            state = nextSOrUFrameState(state, byte, A_RX, &control);
                            if (state == STATE_STOP && control == UA) {
                                // finished
                                goto finish_close;
                            }
                        } else {
                            // error
                            closeSerialPort();
                            return -1;
                        }
                    }
                }
            } else {
                closeSerialPort();
                return -1;
            }
        }
    }

finish_close:
    // print statistics
    printf("[llclose] frames sent: %d, frames received: %d, retransmissions: %d\n",
           stat_frames_sent, stat_frames_recv, stat_retransmissions);

    closeSerialPort();
    return 0;
}