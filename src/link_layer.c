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
#include "statistics.h"

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

static LinkLayerRole role;
static int timeout;           // seconds
static int nRetransmissions;  // max retries


static ll_statistics stats;    

static struct timeval tv_tmp = {0}; // helper for timestamps


////////////////////////////////////////////////
// Helpers
////////////////////////////////////////////////

static int send_bytes(const unsigned char *buf, int len){
    if (!buf || len <= 0) return -1;
    int total = 0;

    while (total < len) {
        int remaining = len - total;
        int w = writeBytesSerialPort((unsigned char*)buf + total, remaining);
        if (w < 0) return -1;
        total += w;
    }

    stats.frames_sent++;
    return total;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters) {

    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0) {
        perror("openSerialPort");
        return -1;
    }

    role = connectionParameters.role;
    timeout = connectionParameters.timeout;
    nRetransmissions = connectionParameters.nRetransmissions;

    // ensure alarm is initialized once
    alarm_init();
    // reset statistics for a new session
    stats_init(&stats);

    // Transmitter
    if (role == LlTx) {
        printf("[TX] Sending SET frame...\n");
        unsigned char frame[5] = { FLAG, A_TX, SET, (unsigned char)(A_TX ^ SET), FLAG };

        int attempts = 0;
        unsigned char byte = 0, control = 0;
        State state;

        while (attempts < nRetransmissions) {
            // Send SET frame
            stats.SET_sent++;
            if (send_bytes(frame, 5) != 5) {
                perror("send SET");
                closeSerialPort();
                return -1;
            }

            printf("[TX] Waiting for UA (attempt %d/%d)...\n", attempts + 1, nRetransmissions);
            alarm_start((unsigned int)timeout);
            state = STATE_START;

            // Wait for UA or timeout
            while (alarmActive) {
                int r = readByteSerialPort( &byte);
                if (r == 1) {
                    state = getSOrUState(state, byte, A_RX, &control);
                    // Valid UA frame received
                    if (state == STATE_STOP && control == UA) {
                        alarm_stop();
                        printf("[TX] Connection established successfully!\n");
                        // mark start time
                        gettimeofday(&tv_tmp, NULL);
                        stats.start_time = tv_tmp.tv_sec + tv_tmp.tv_usec / 1e6;
                        stats.frames_received++; // UA received
                        stats.UA_received++;
                        return 0;
                    }
                } 
                else if (r < 0) {
                    // Expected errors, nothing to read or alarm interruption, continue
                    if (errno == EAGAIN || errno == EINTR) {
                        continue;
                    }
                    // Actual error, send error to app layer
                    perror("[TX] read UA");
                    alarm_stop();
                    closeSerialPort();
                    return -1;
                }
                // else r==0, no data yet
            }

            // Timeout, will trigger retransmission

            stats.timeouts++;
            stats.retransmissions++;
            printf("[TX] Timeout waiting for UA — retransmitting SET...\n");
            attempts++;
        }

        // Retransmissions exhausted
        fprintf(stderr, "[TX] SET retries exhausted (%d attempts)\n", attempts);
        closeSerialPort();
        return -1;
    }

    // Receiver
    else if (role == LlRx){
        printf("[RX] Waiting for SET frame...\n");
        State state = STATE_START;
        unsigned char byte = 0, control = 0;

        while (1) {
            int r = readByteSerialPort(&byte);
            if (r == 1) {
                state = getSOrUState(state, byte, A_TX, &control);
                // Valid SET frame received
                if (state == STATE_STOP && control == SET) {
                    stats.frames_received++;
                    stats.SET_received++;
                    // Send UA
                    unsigned char ua[5] = { FLAG, A_RX, UA, (unsigned char)(A_RX ^ UA), FLAG };
                    stats.UA_sent++;
                    if (send_bytes(ua, 5) != 5) {
                        perror("send UA");
                        closeSerialPort();
                        return -1;
                    }
                    printf("[RX] Connection established successfully!\n");
                    // mark start time
                    gettimeofday(&tv_tmp, NULL);
                    stats.start_time = tv_tmp.tv_sec + tv_tmp.tv_usec / 1e6;
                    return 0;
                }
            }
            else if (r < 0) {
                if (errno == EAGAIN || errno == EINTR) {
                    continue;
                }
                perror("read SET (RX)");
                closeSerialPort();
                return -1;
            }
            // else r==0, no data yet
        }
    }

    return -1;
}

////////////////////////////////////////////////
// LLWRITE (Transmitter only sends I-frames)
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize) {
    // N(s)
    static int frameNumber = 0;
    if (bufSize > MAX_PAYLOAD_SIZE) return -1;

    // Build BCC2, XOR of payload
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
    unsigned char frame[stuffed_len + 5];

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

        // Send I frames
        stats.I_frames_sent++;
        if (send_bytes(frame, frame_len) != frame_len) {
            attempts++;
            stats.retransmissions++;
            printf("[TX] Send error, retransmitting (%d/%d)\n", attempts, nRetransmissions);
            continue;
        }

        State state = STATE_START;
        unsigned char byte = 0, control = 0;

        // Start timer, waiting for RR/REJ
        alarm_start((unsigned int)timeout);

        while (alarmActive) {
            ssize_t r = readByteSerialPort(&byte);
            if (r == 1) {
                state = getSOrUState(state, byte, A_RX, &control);
                // S/U parsing: FSM will self-recover to START/FLAG_RCV as needed
                if (state == STATE_STOP) {
                    // REJ received: stop timer, retransmit
                    if ((control & 0x7F) == 0x01) {
                        alarm_stop();
                        stats.REJ_received++;
                        stats.retransmissions++;
                        stats.frames_received++;
                        rej_retries++;
                        if (rej_retries > nRetransmissions) {
                            return -1;
                        }
                        printf("[TX] REJ received, retransmitting (REJ %d/%d)\n", rej_retries, nRetransmissions);
                        // Jump directly to retransmission, so it doesnt count as an attempt/timeout
                        goto retransmit_same_frame;
                    }
                    // RR received
                    else if ((control & 0x7F) == 0x05) {
                        // N(r)
                        int rbit = (control & 0x80) ? 1 : 0;
                        stats.RR_received++;
                        stats.frames_received++;
                        // Toggle N(s) to receive next frame
                        if (rbit == (frameNumber ^ 1)) {
                            alarm_stop();
                            frameNumber ^= 1;
                            return bufSize;
                        }
                        // else: RR for current Ns (duplicate), ignore
                    }
                }
            } 

            else if (r < 0) {
                if (errno == EAGAIN || errno == EINTR) {
                    continue;
                }
                alarm_stop();
                printf("[TX] Read error while waiting RR/REJ\n");
                return -1;
            }
            // else r==0, no data yet

        }

        // Timeout, retransmit
        stats.timeouts++;
        attempts++;
        stats.retransmissions++;
        rej_retries = 0;
        printf("[TX] Timeout waiting RR/REJ, retransmitting (%d/%d)\n", attempts, nRetransmissions);

    retransmit_same_frame:
        ; 
    }

    return -1;
}

////////////////////////////////////////////////
// LLREAD (Receiver parses I-frames and replies)
////////////////////////////////////////////////
int llread(unsigned char *packet){
    static int expectedFrame = 0;
    State state = STATE_START;
    unsigned char byte;
    unsigned char frameNumberRx = 0;

    unsigned char stuffed_buf[MAX_PAYLOAD_SIZE * 2 + 16];
    int idx = 0;

    while (1) {
        ssize_t r = readByteSerialPort(&byte);
        if (r == 1) {
            state = getIState(state, byte, A_TX, &frameNumberRx, stuffed_buf, &idx, sizeof(stuffed_buf));
            
            // Discard buffer if bad header/wrong BCC1
            if (state == STATE_ERROR) {
                state = STATE_START;
                idx = 0;
                continue;
            }           

            if (state == STATE_STOP) {
                stats.frames_received++;

                unsigned char unstuffed[MAX_PAYLOAD_SIZE + 2];
                int un_len = unstuff_buffer(stuffed_buf, idx, unstuffed, sizeof(unstuffed));
                
                if (un_len <= 0) { 
                    state = STATE_START;
                    idx = 0; 
                    continue; 
                }

                int payload_len = un_len - 1;
                unsigned char recv_bcc2 = unstuffed[un_len - 1];

                // Recompute BCC2 on payload
                unsigned char calc_bcc2 = compute_bcc2(unstuffed, payload_len);

                if (calc_bcc2 != recv_bcc2) {
                    // Data error so REJ(expectedFrame)
                    unsigned char resp[5] = { FLAG, A_RX, REJ(expectedFrame), (unsigned char)(A_RX ^ REJ(expectedFrame)), FLAG };
                    stats.REJ_sent++;
                    send_bytes(resp, 5);
                    state = STATE_START; idx = 0;
                    continue;
                }

                // Header OK & data OK
                if (frameNumberRx == expectedFrame) {
                    // Accept payload
                    memcpy(packet, unstuffed, payload_len);
                    stats.I_frames_received++;
                    expectedFrame ^= 1;
                    unsigned char rr[5] = { FLAG, A_RX, RR(expectedFrame), (unsigned char)(A_RX ^ RR(expectedFrame)), FLAG };
                    stats.RR_sent++;
                    send_bytes(rr, 5);
                    return payload_len;
                }
                else {
                    // Duplicate so re ACK last expected
                    unsigned char rr_dup[5] = { FLAG, A_RX, RR(expectedFrame), (unsigned char)(A_RX ^ RR(expectedFrame)), FLAG };
                    stats.RR_sent++;
                    send_bytes(rr_dup, 5);
                    state = STATE_START;
                    idx = 0;
                    continue;
                }
            }
        } 
        else if (r <= 0) {
            return -1;
        }
    }
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose() {
    unsigned char disc_tx[5] = { FLAG, A_TX, DISC, (unsigned char)(A_TX ^ DISC), FLAG };
    unsigned char disc_rx[5] = { FLAG, A_RX, DISC, (unsigned char)(A_RX ^ DISC), FLAG };
    unsigned char ua_tx[5]   = { FLAG, A_RX, UA,   (unsigned char)(A_RX ^ UA),   FLAG };

    State state;
    unsigned char byte = 0, control = 0;

    alarm_init();

    //Transmitter
    if (role == LlTx) {
        int attempts = 0;

        // Send DISC and wait for DISC from receiver
        while (attempts < nRetransmissions) {
            stats.DISC_sent++;
            if (send_bytes(disc_tx, 5) != 5) {
                closeSerialPort();
                return -1;
            }

            state = STATE_START;
            int got_disc = 0;

            alarm_start((unsigned int)timeout);

            while (alarmActive) {
                ssize_t r = readByteSerialPort(&byte);
                if (r == 1) {
                    state = getSOrUState(state, byte, A_RX, &control);

                    if (state == STATE_STOP && control == DISC) {
                        // Valid DISC received
                        got_disc = 1;
                        stats.frames_received++;
                        stats.DISC_received++;
                        break;
                    }
                }
                else if (r < 0 && errno != EAGAIN && errno != EINTR) {
                    alarm_stop();
                    closeSerialPort();
                    return -1;
                }
            }

            if (got_disc) {
                alarm_stop();
                stats.UA_sent++;
                // Send UA to confirm closing
                send_bytes(ua_tx, 5);
                goto finish_close;
            }
            else {
                // Timeout
                stats.timeouts++;
                attempts++;
                stats.retransmissions++;
                printf("[TX] Timeout waiting DISC, retransmitting (%d/%d)\n", attempts, nRetransmissions);
            }
        }

        // Exceeded attempts
        closeSerialPort();
        return -1;
    }

    // Receiver
    else { 
        state = STATE_START;

        // Wait DISC from tx
        while (1) {
            ssize_t r = readByteSerialPort(&byte);

            if (r == 1) {
                state = getSOrUState(state, byte, A_TX, &control);

                // DISC received
                if (state == STATE_STOP && control == DISC) {
                    stats.frames_received++;
                    stats.DISC_received++;

                    // Send DISC and wait for UA from tx
                    int attempts = 0;
                    while (attempts < nRetransmissions) {
                        stats.DISC_sent++;
                        send_bytes(disc_rx, 5);
                        state = STATE_START;
                        int got_ua = 0;

                        alarm_start((unsigned int)timeout);

                        while (alarmActive) {
                            ssize_t r2 = readByteSerialPort(&byte);
                            
                            if (r2 == 1) {
                                state = getSOrUState(state, byte, A_RX, &control);
                                if (state == STATE_STOP && control == UA) { 
                                    // Valid UA received, terminate connection
                                    got_ua = 1;
                                    stats.frames_received++;
                                    stats.UA_received++;
                                    break;
                                 }
                            }
                            else if (r2 < 0 && errno != EAGAIN && errno != EINTR) {
                                alarm_stop();
                                closeSerialPort();
                                return -1;
                            }
                        }

                        if (got_ua) {
                            alarm_stop();
                            goto finish_close;
                        }

                        // Timeout
                        stats.timeouts++;
                        attempts++;
                        stats.retransmissions++;
                        printf("[RX] Timeout waiting UA, retransmitting DISC (%d/%d)\n", attempts, nRetransmissions);
                    }

                    // Attempts exceeded
                    closeSerialPort();
                    return -1;
                }
            } 
            
            else {
                closeSerialPort();
                return -1;
            }
        }
    }

finish_close:
    gettimeofday(&tv_tmp, NULL);
    stats.end_time = tv_tmp.tv_sec + tv_tmp.tv_usec / 1e6;
    stats_update_duration(&stats);

    print_statistics_for_role(&stats, role == LlTx ? 1 : 0);

    closeSerialPort();
    return 0;
}
