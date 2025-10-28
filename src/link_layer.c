// Link layer protocol implementation
#include "link_layer.h"
#include "serial_port.h"
#include "state.h"
#include "special_bytes.h"
#include "utils_stuff.h"
#include "alarm.h"

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
static int stat_rej_received = 0;
static int stat_timeouts = 0;
static struct timeval t_start = {0}, t_end = {0};

////////////////////////////////////////////////
// Helper: read single byte with timeout (seconds).
// Returns 1 if byte read, 0 on timeout, -1 on error.
////////////////////////////////////////////////

static int read_byte_timeout(unsigned char *out, int timeout_seconds)
{
    if (fd < 0) {
        return -1;
    }

    // Initialize alarm once
    static int alarm_inited = 0;
    if (!alarm_inited) {
        alarm_init();
        alarm_inited = 1;
    }

    // Arm alarm
    alarm_start((unsigned int)timeout_seconds);

    errno = 0;
    ssize_t r = read(fd, out, 1);
    int saved_errno = errno;

    // Disarm alarm
    alarm_stop();

    if (r == 1) {
        return 1;
    }
    if (r == 0) {
        // EOF treated as timeout
        return 0;
    }
    if (r == -1) {
        if (saved_errno == EINTR || saved_errno == EAGAIN ||
            saved_errno == EWOULDBLOCK || saved_errno == EIO) {
            return 0;
        }
        errno = saved_errno;
        return -1;
    }
    return -1;
}


////////////////////////////////////////////////
// Helper: send buffer using existing helper; fallback to write if helper fails
////////////////////////////////////////////////
static int send_bytes(const unsigned char *buf, int len)
{
    if (!buf || len <= 0) return -1;
    int total = 0;

    while (total < len) {
        int remaining = len - total;
        int w = writeBytesSerialPort((unsigned char*)buf + total, remaining);
        if (w < 0) return -1;
        if (w == 0) {
            ssize_t r = write(fd, buf + total, remaining);
            if (r <= 0) return -1;
            total += (int)r;
        } else {
            total += w;
        }
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
    ll_timeout = connectionParameters.timeout > 0 ? connectionParameters.timeout : ll_timeout;
    ll_nRetransmissions = connectionParameters.nRetransmissions > 0 ? connectionParameters.nRetransmissions : ll_nRetransmissions;

    // Transmitter
    if (role == LlTx)
    {
        printf("[TX] Sending SET frame...\n");
        unsigned char frame[5] = { FLAG, A_TX, SET, (unsigned char)(A_TX ^ SET), FLAG };

        int attempts = 0;
        while (attempts <= ll_nRetransmissions) {
            if (send_bytes(frame, 5) != 5) {
                perror("send SET");
                closeSerialPort();
                return -1;
            }

            State state = STATE_START;
            unsigned char byte = 0, control = 0;
            int gotUA = 0;

            while (!gotUA) {
                int r = read_byte_timeout(&byte, ll_timeout);
                if (r == 1) {
                    state = nextSOrUFrameState(state, byte, A_RX, &control);
                    if (state == STATE_STOP && control == UA) {
                        printf("[TX] Connection established successfully!\n");
                        // mark session start time
                        gettimeofday(&t_start, NULL);
                        return 0;
                    }
                } else if (r == 0) {
                    stat_timeouts++;               // count timeout
                    // timeout => break to retransmit
                    break;
                } else {
                    perror("read");
                    closeSerialPort();
                    return -1;
                }
            }

            attempts++;
            stat_retransmissions++;
            if (attempts > ll_nRetransmissions) {
                fprintf(stderr, "[TX] SET retries exhausted\n");
                closeSerialPort();
                return -1;
            }
            printf("[TX] Timeout, retransmitting SET (attempt %d/%d)...\n", attempts, ll_nRetransmissions);
        }
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
            ssize_t r = read(fd, &byte, 1);
            if (r == 1) {
                state = nextSOrUFrameState(state, byte, A_TX, &control);
                if (state == STATE_STOP && control == SET) {
                    unsigned char ua[5] = { FLAG, A_RX, UA, (unsigned char)(A_RX ^ UA), FLAG };
                    if (send_bytes(ua, 5) != 5) {
                        perror("send UA");
                        closeSerialPort();
                        return -1;
                    }
                    printf("[RX] Connection established successfully!\n");
                    // mark session start time
                    gettimeofday(&t_start, NULL);
                    return 0;
                }
            } else if (r <= 0) {
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

    unsigned char bcc2 = 0x00;
    for (int i = 0; i < bufSize; i++) bcc2 ^= buf[i];

    unsigned char tmp[MAX_PAYLOAD_SIZE + 1];
    memcpy(tmp, buf, bufSize);
    tmp[bufSize] = bcc2;

    unsigned char stuffed[(MAX_PAYLOAD_SIZE + 1) * 2 + 10];
    int stuffed_len = stuff_buffer(tmp, bufSize + 1, stuffed, sizeof(stuffed));
    if (stuffed_len < 0) return -1;

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

    while (attempts <= ll_nRetransmissions) {
        if (send_bytes(frame, frame_len) != frame_len) {
                attempts++;
                stat_retransmissions++;
                if (attempts > ll_nRetransmissions) {
                    free(frame);
                    return -1;
                }
                printf("[TX] Send error, retransmitting (%d/%d)\n", attempts, ll_nRetransmissions);
                continue;
        }

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
                        stat_rej_received++;        // count REJ received
                        stat_retransmissions++;
                        rej_retries++;
                        if (rej_retries > ll_nRetransmissions) {
                            free(frame);
                            return -1;
                        }
                        printf("[TX] REJ received, retransmitting (REJ %d/%d)\n", rej_retries, ll_nRetransmissions);
                        done = 1;
                    } else if ((control & 0x7F) == 0x05) { // RR
                        int rbit = (control & 0x80) ? 1 : 0;
                        if (rbit == (frameNumber ^ 1)) {
                            frameNumber ^= 1;
                            free(frame);
                            return bufSize;
                        }
                    }
                }
            } else if (r == 0) {
                attempts++;
                stat_retransmissions++;
                rej_retries = 0;
                stat_timeouts++;                     // count timeout
                if (attempts > ll_nRetransmissions) {
                    free(frame);
                    return -1;
                }
                printf("[TX] Timeout waiting RR, retransmitting (%d/%d)\n", attempts, ll_nRetransmissions);
                done = 1;
            } else {
                printf("[TX] Read error while waiting RR/REJ\n");
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
    static int expectedFrame = 0;
    State state = STATE_START;
    unsigned char byte;
    unsigned char frameNumberRx = 0;
    unsigned char stuffed_buf[MAX_PAYLOAD_SIZE * 2 + 16];
    int idx = 0;

    while (1) {
        ssize_t r = read(fd, &byte, 1);
        if (r == 1) {
            state = nextIFrameState(state, byte, A_TX, &frameNumberRx, stuffed_buf, &idx, sizeof(stuffed_buf));
            if (state == STATE_STOP) {

                unsigned char unstuffed[MAX_PAYLOAD_SIZE + 2];
                int un_len = unstuff_buffer(stuffed_buf, idx, unstuffed, sizeof(unstuffed));
                if (un_len <= 0) continue;

                int payload_len = un_len - 1;
                unsigned char recv_bcc2 = unstuffed[un_len - 1];
                unsigned char calc_bcc2 = compute_bcc2(unstuffed, payload_len);

                if (calc_bcc2 != recv_bcc2) {
                    unsigned char resp[5] = { FLAG, A_RX, REJ(expectedFrame), (unsigned char)(A_RX ^ REJ(expectedFrame)), FLAG };
                    send_bytes(resp, 5);
                    state = STATE_START; idx = 0;
                    continue;
                }

                if (frameNumberRx == expectedFrame) {
                    memcpy(packet, unstuffed, payload_len);
                    expectedFrame ^= 1;
                    unsigned char rr[5] = { FLAG, A_RX, RR(expectedFrame), (unsigned char)(A_RX ^ RR(expectedFrame)), FLAG };
                    send_bytes(rr, 5);
                    return payload_len;
                } else {
                    unsigned char rr_dup[5] = { FLAG, A_RX, RR(expectedFrame), (unsigned char)(A_RX ^ RR(expectedFrame)), FLAG };
                    send_bytes(rr_dup, 5);
                    state = STATE_START; idx = 0;
                    continue;
                }
            }
        } else if (r <= 0) {
            return -1;
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
    unsigned char ua_tx[5]   = { FLAG, A_RX, UA, (unsigned char)(A_RX ^ UA), FLAG };

    State state;
    unsigned char byte = 0, control = 0;

    if (role == LlTx) {
        int attempts = 0;
        while (attempts <= ll_nRetransmissions) {
            if (send_bytes(disc_tx, 5) != 5) return -1;

            state = STATE_START;
            int got_disc = 0;
            while (1) {
                int r = read_byte_timeout(&byte, ll_timeout);
                if (r == 1) {
                    state = nextSOrUFrameState(state, byte, A_RX, &control);
                    if (state == STATE_STOP && control == DISC) { got_disc = 1; break; }
                } else if (r == 0) break; // timeout
                else { closeSerialPort(); return -1; }
            }

            if (got_disc) {
                send_bytes(ua_tx, 5);
                break;
            } else {
                attempts++;
                stat_retransmissions++;
                if (attempts > ll_nRetransmissions) { closeSerialPort(); return -1; }
            }
        }
    } else {
        state = STATE_START;
        while (1) {
            ssize_t r = read(fd, &byte, 1);
            if (r == 1) {
                state = nextSOrUFrameState(state, byte, A_TX, &control);
                if (state == STATE_STOP && control == DISC) {
                    int attempts = 0;
                    while (attempts <= ll_nRetransmissions) {
                        send_bytes(disc_rx, 5);
                        state = STATE_START;
                        int got_ua = 0;
                        while (1) {
                            int r2 = read_byte_timeout(&byte, ll_timeout);
                            if (r2 == 1) {
                                state = nextSOrUFrameState(state, byte, A_RX, &control);
                                if (state == STATE_STOP && control == UA) { got_ua = 1; break; }
                            } else if (r2 == 0) break;
                            else { closeSerialPort(); return -1; }
                        }
                        if (got_ua) goto finish_close;
                        attempts++;
                        stat_retransmissions++;
                        if (attempts > ll_nRetransmissions) { closeSerialPort(); return -1; }
                    }
                }
            } else { closeSerialPort(); return -1; }
        }
    }

finish_close:
    gettimeofday(&t_end, NULL);
    // compute seconds correctly: microseconds divided by 1e6
    double secs = (t_end.tv_sec - t_start.tv_sec) + (t_end.tv_usec - t_start.tv_usec)/1e6;

    printf("[llclose] number of frames sent: %d, retransmissions: %d, duration: %.3f s, timeouts: %d, number of REJ: %d\n",
           stat_frames_sent, stat_retransmissions,
           secs > 0 ? secs : 0.0, stat_timeouts, stat_rej_received
           );

    closeSerialPort();
    return 0;
}   
