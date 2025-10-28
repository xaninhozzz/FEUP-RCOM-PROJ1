// Link layer protocol implementation (refactored to per-frame alarms)
// - One alarm per frame (SET / I / DISC), stopped on valid reply
// - No read_byte_timeout(); reads are plain read()/readByteSerialPort()
// - Retransmissions on SIGALRM expiry
//
// Depends on your existing project headers & helpers/macros.

#include "link_layer.h"
#include "serial_port.h"
#include "state.h"
#include "macros.h"
#include "utils_stuff.h"
#include "alarm.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <errno.h>
#include <stdlib.h>
#include <signal.h>

#define _POSIX_SOURCE 1 // POSIX compliant source

////////////////////////////////////////////////
// Globals / configuration saved from llopen
////////////////////////////////////////////////

static int fd = -1;
static LinkLayerRole role;
static int timeout;           // seconds
static int nRetransmissions;  // max retries

// statistics
static int stat_retransmissions = 0;
static int stat_frames_sent     = 0;
static int stat_rej_received    = 0;
static int stat_timeouts        = 0;

static struct timeval t_start = {0}, t_end = {0};

// Timer state comes from alarm.c via alarmActive

////////////////////////////////////////////////
// Helpers
////////////////////////////////////////////////

// send buffer using existing helper
static int send_bytes(const unsigned char *buf, int len)
{
    if (!buf || len <= 0) return -1;
    int total = 0;

    while (total < len) {
        int remaining = len - total;
        int w = writeBytesSerialPort((unsigned char*)buf + total, remaining);
        if (w < 0) return -1;
        total += w;
    }

    stat_frames_sent++;
    return total;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    fd = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate);
    if (fd < 0) {
        perror("openSerialPort");
        return -1;
    }

    role = connectionParameters.role;
    timeout = connectionParameters.timeout;
    nRetransmissions = connectionParameters.nRetransmissions;

    // ensure alarm is initialized once
    alarm_init();

    // Transmitter
    if (role == LlTx)
    {
        printf("[TX] Sending SET frame...\n");
        unsigned char frame[5] = { FLAG, A_TX, SET, (unsigned char)(A_TX ^ SET), FLAG };

        int attempts = 0;
        unsigned char byte = 0, control = 0;
        State state;

        while (attempts < nRetransmissions) {
            if (send_bytes(frame, 5) != 5) {
                perror("send SET");
                closeSerialPort();
                return -1;
            }

            printf("[TX] Waiting for UA (attempt %d/%d)...\n", attempts + 1, nRetransmissions);
            alarm_start((unsigned int)timeout);

            state = STATE_START;
            while (alarmActive) {
                int r = readByteSerialPort( &byte);
                if (r == 1) {
                    state = getSOrUState(state, byte, A_RX, &control);
                    if (state == STATE_STOP && control == UA) {
                        alarm_stop();
                        printf("[TX] Connection established successfully!\n");
                        gettimeofday(&t_start, NULL);
                        return 0;
                    }
                } else if (r < 0 && errno != EAGAIN && errno != EINTR) {
                    perror("read UA");
                    alarm_stop();
                    closeSerialPort();
                    return -1;
                }
                // else: r==0 (no byte now) or EINTR/EAGAIN → keep looping until alarm or UA
            }

            // timeout
            stat_timeouts++;
            stat_retransmissions++;
            printf("[TX] Timeout waiting for UA — retransmitting SET...\n");
            attempts++;
        }

        fprintf(stderr, "[TX] SET retries exhausted (%d attempts)\n", attempts);
        closeSerialPort();
        return -1;
    }

    // Receiver
    else if (role == LlRx)
    {
        printf("[RX] Waiting for SET frame...\n");
        State state = STATE_START;
        unsigned char byte = 0, control = 0;

        while (1) {
            int r = readByteSerialPort(&byte);
            if (r == 1) {
                state = getSOrUState(state, byte, A_TX, &control);
                if (state == STATE_STOP && control == SET) {
                    unsigned char ua[5] = { FLAG, A_RX, UA, (unsigned char)(A_RX ^ UA), FLAG };
                    if (send_bytes(ua, 5) != 5) {
                        perror("send UA");
                        closeSerialPort();
                        return -1;
                    }
                    printf("[RX] Connection established successfully!\n");
                    gettimeofday(&t_start, NULL);
                    return 0;
                }
            } else if (r <= 0) {
                perror("read SET (RX)");
                closeSerialPort();
                return -1;
            }
        }
    }

    return -1;
}

////////////////////////////////////////////////
// LLWRITE (Transmitter only sends I-frames)
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    static int frameNumber = 0;
    if (bufSize > MAX_PAYLOAD_SIZE) return -1;

    // Build BCC2: XOR of payload
    unsigned char bcc2 = 0x00;
    for (int i = 0; i < bufSize; i++) bcc2 ^= buf[i];

    // tmp = [payload || bcc2]
    unsigned char tmp[MAX_PAYLOAD_SIZE + 1];
    memcpy(tmp, buf, bufSize);
    tmp[bufSize] = bcc2;

    // Stuff payload+bcc2
    unsigned char stuffed[(MAX_PAYLOAD_SIZE + 1) * 2 + 10];
    int stuffed_len = stuff_buffer(tmp, bufSize + 1, stuffed, sizeof(stuffed));
    if (stuffed_len < 0) return -1;

    // Build I-frame: F A C BCC1 [stuffed payload+bcc2] F
    unsigned char *frame = malloc(4 + stuffed_len + 1);
    if (!frame) return -1;

    int frame_len = 0;
    frame[frame_len++] = FLAG;
    frame[frame_len++] = A_TX;
    unsigned char Cbyte = C(frameNumber);
    frame[frame_len++] = Cbyte;
    frame[frame_len++] = (unsigned char)(A_TX ^ Cbyte);
    memcpy(frame + frame_len, stuffed, stuffed_len);
    frame_len += stuffed_len;
    frame[frame_len++] = FLAG;

    int attempts = 0;
    int rej_retries = 0;

    while (attempts < nRetransmissions) {

        if (send_bytes(frame, frame_len) != frame_len) {
            attempts++;
            stat_retransmissions++;
            printf("[TX] Send error, retransmitting (%d/%d)\n", attempts, nRetransmissions);
            continue;
        }

        State state = STATE_START;
        unsigned char byte = 0, control = 0;

        // Start per-frame timer waiting for RR/REJ
        alarm_start((unsigned int)timeout);

        while (alarmActive) {
            ssize_t r = read(fd, &byte, 1);
            if (r == 1) {
                state = getSOrUState(state, byte, A_RX, &control);
                if (state == STATE_STOP) {
                    // REJ?
                    if ((control & 0x7F) == 0x01) {
                        // REJ received: stop timer, retransmit immediately
                        alarm_stop();
                        stat_rej_received++;
                        stat_retransmissions++;
                        rej_retries++;
                        if (rej_retries > nRetransmissions) {
                            free(frame);
                            return -1;
                        }
                        printf("[TX] REJ received, retransmitting (REJ %d/%d)\n", rej_retries, nRetransmissions);
                        // break inner loop → resend same frame (attempt not incremented here to keep same semantics)
                        goto retransmit_same_frame;
                    }
                    // RR?
                    else if ((control & 0x7F) == 0x05) {
                        int rbit = (control & 0x80) ? 1 : 0;
                        if (rbit == (frameNumber ^ 1)) {
                            alarm_stop();
                            frameNumber ^= 1;
                            free(frame);
                            return bufSize;
                        }
                        // else: RR for current Ns (duplicate) → keep waiting
                    }
                }
            } else if (r < 0 && errno != EAGAIN && errno != EINTR) {
                // hard read error
                alarm_stop();
                printf("[TX] Read error while waiting RR/REJ\n");
                free(frame);
                return -1;
            }
        }

        // timeout
        stat_timeouts++;
        attempts++;
        stat_retransmissions++;
        rej_retries = 0;
        printf("[TX] Timeout waiting RR/REJ, retransmitting (%d/%d)\n", attempts, nRetransmissions);

retransmit_same_frame:
        ; // label target needs a statement
    }

    free(frame);
    return -1;
}

////////////////////////////////////////////////
// LLREAD (Receiver parses I-frames and replies)
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    static int expectedFrame = 0;
    State state = STATE_START;
    unsigned char byte;
    unsigned char frameNumberRx = 0;

    unsigned char stuffed_buf[MAX_PAYLOAD_SIZE * 2 + 16];
    int idx = 0;

    while (1) {
        ssize_t r = read(fd, &byte, 1);
        if (r == 1) {
            state = getIState(state, byte, A_TX, &frameNumberRx, stuffed_buf, &idx, sizeof(stuffed_buf));
            if (state == STATE_STOP) {

                unsigned char unstuffed[MAX_PAYLOAD_SIZE + 2];
                int un_len = unstuff_buffer(stuffed_buf, idx, unstuffed, sizeof(unstuffed));
                if (un_len <= 0) { state = STATE_START; idx = 0; continue; }

                int payload_len = un_len - 1;
                unsigned char recv_bcc2 = unstuffed[un_len - 1];

                // Recompute BCC2 on payload
                unsigned char calc_bcc2 = compute_bcc2(unstuffed, payload_len);

                if (calc_bcc2 != recv_bcc2) {
                    // Data error → REJ(expectedFrame)
                    unsigned char resp[5] = { FLAG, A_RX, REJ(expectedFrame), (unsigned char)(A_RX ^ REJ(expectedFrame)), FLAG };
                    send_bytes(resp, 5);
                    state = STATE_START; idx = 0;
                    continue;
                }

                // Header OK & data OK
                if (frameNumberRx == expectedFrame) {
                    // Accept payload
                    memcpy(packet, unstuffed, payload_len);
                    expectedFrame ^= 1;
                    unsigned char rr[5] = { FLAG, A_RX, RR(expectedFrame), (unsigned char)(A_RX ^ RR(expectedFrame)), FLAG };
                    send_bytes(rr, 5);
                    return payload_len;
                } else {
                    // Duplicate → re-ACK last expected
                    unsigned char rr_dup[5] = { FLAG, A_RX, RR(expectedFrame), (unsigned char)(A_RX ^ RR(expectedFrame)), FLAG };
                    send_bytes(rr_dup, 5);
                    state = STATE_START; idx = 0;
                    continue;
                }
            }
        } else if (r <= 0) {
            return -1; // hard read error or EOF
        }
    }
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose()
{
    unsigned char disc_tx[5] = { FLAG, A_TX, DISC, (unsigned char)(A_TX ^ DISC), FLAG };
    unsigned char disc_rx[5] = { FLAG, A_RX, DISC, (unsigned char)(A_RX ^ DISC), FLAG };
    unsigned char ua_tx[5]   = { FLAG, A_RX, UA,   (unsigned char)(A_RX ^ UA),   FLAG };

    State state;
    unsigned char byte = 0, control = 0;

    // ensure alarm is initialized (safe if already done)
    alarm_init();

    if (role == LlTx) {
        int attempts = 0;

        while (attempts < nRetransmissions) {
            if (send_bytes(disc_tx, 5) != 5) {
                closeSerialPort();
                return -1;
            }

            state = STATE_START;
            int got_disc = 0;

            alarm_start((unsigned int)timeout);

            while (alarmActive) {
                ssize_t r = read(fd, &byte, 1);
                if (r == 1) {
                    state = getSOrUState(state, byte, A_RX, &control);
                    if (state == STATE_STOP && control == DISC) { got_disc = 1; break; }
                } else if (r < 0 && errno != EAGAIN && errno != EINTR) {
                    alarm_stop();
                    closeSerialPort();
                    return -1;
                }
            }

            if (got_disc) {
                alarm_stop();
                send_bytes(ua_tx, 5);
                goto finish_close;
            } else {
                // timeout
                stat_timeouts++;
                attempts++;
                stat_retransmissions++;
                printf("[TX] Timeout waiting DISC, retransmitting (%d/%d)\n", attempts, nRetransmissions);
            }
        }

        // exceeded attempts
        closeSerialPort();
        return -1;
    }
    else { // Receiver
        state = STATE_START;

        // Wait DISC from Tx
        while (1) {
            ssize_t r = read(fd, &byte, 1);
            if (r == 1) {
                state = getSOrUState(state, byte, A_TX, &control);
                if (state == STATE_STOP && control == DISC) {
                    // Send DISC and wait UA with alarm+retries
                    int attempts = 0;
                    while (attempts < nRetransmissions) {
                        send_bytes(disc_rx, 5);

                        state = STATE_START;
                        int got_ua = 0;

                        alarm_start((unsigned int)timeout);

                        while (alarmActive) {
                            ssize_t r2 = read(fd, &byte, 1);
                            if (r2 == 1) {
                                state = getSOrUState(state, byte, A_RX, &control);
                                if (state == STATE_STOP && control == UA) { got_ua = 1; break; }
                            } else if (r2 < 0 && errno != EAGAIN && errno != EINTR) {
                                alarm_stop();
                                closeSerialPort();
                                return -1;
                            }
                        }

                        if (got_ua) {
                            alarm_stop();
                            goto finish_close;
                        }

                        // timeout: try again
                        stat_timeouts++;
                        attempts++;
                        stat_retransmissions++;
                        printf("[RX] Timeout waiting UA, retransmitting DISC (%d/%d)\n", attempts, nRetransmissions);
                    }

                    // exceeded attempts from RX side
                    closeSerialPort();
                    return -1;
                }
            } else {
                closeSerialPort();
                return -1;
            }
        }
    }

finish_close:
    gettimeofday(&t_end, NULL);
    double secs = (t_end.tv_sec - t_start.tv_sec) + (t_end.tv_usec - t_start.tv_usec)/1e6;

    printf("[llclose] number of frames sent: %d, retransmissions: %d, duration: %.3f s, timeouts: %d, number of REJ: %d\n",
           stat_frames_sent, stat_retransmissions,
           secs > 0 ? secs : 0.0, stat_timeouts, stat_rej_received
    );

    closeSerialPort();
    return 0;
}
