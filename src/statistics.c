#include "statistics.h"
#include <string.h>

void stats_init(ll_statistics *s) {
    if (!s) return;
    memset(s, 0, sizeof(*s));
}

void stats_update_duration(ll_statistics *s) {
    if (!s) return;
    s->duration = (s->end_time >= s->start_time) ? (s->end_time - s->start_time) : 0.0;
}

void print_statistics(const ll_statistics *s) {
    if (!s) return;
    printf("\n");
    printf("************ STATISTICS ************\n");
    printf("\n");
    printf("Total Frames Sent: %u\n", s->frames_sent);
    printf("Total Frames Received: %u\n", s->frames_received);
    printf("Total SET Frames Sent: %u\n", s->SET_sent);
    printf("Total SET Frames Received: %u\n", s->SET_received);
    printf("Total UA Frames Sent: %u\n", s->UA_sent);
    printf("Total UA Frames Received: %u\n", s->UA_received);
    printf("Total RR Frames Sent: %u\n", s->RR_sent);
    printf("Total RR Frames Received: %u\n", s->RR_received);
    printf("Total REJ Frames Sent: %u\n", s->REJ_sent);
    printf("Total REJ Frames Received: %u\n", s->REJ_received);
    printf("Total I Frames Sent: %u\n", s->I_frames_sent);
    printf("Total I Frames Received: %u\n", s->I_frames_received);
    printf("Total DISC Frames Sent: %u\n", s->DISC_sent);
    printf("Total DISC Frames Received: %u\n", s->DISC_received);
    printf("Total Timeouts: %u\n", s->timeouts);
    printf("Total Retransmissions: %u\n", s->retransmissions);
    if (s->duration > 0.0) {
        printf("Transmission duration: %.3f s\n", s->duration);
    }
    printf("\n");
}

void print_statistics_for_role(const ll_statistics *s, int is_tx) {
    if (!s) return;
    printf("\n");
    printf("************ STATISTICS (%s) ************\n", is_tx ? "Transmitter" : "Receiver");
    printf("\n");
    printf("Sent: ");
    if (is_tx) {
        printf("SET: %u, I: %u, DISC: %u, UA: %u\n", s->SET_sent, s->I_frames_sent, s->DISC_sent, s->UA_sent);
    } else {
        printf("UA: %u, RR: %u, REJ: %u, DISC: %u\n", s->UA_sent, s->RR_sent, s->REJ_sent, s->DISC_sent);
    }
    printf("Received: ");
    if (is_tx) {
        printf("UA: %u, RR: %u, REJ: %u, DISC: %u\n", s->UA_received, s->RR_received, s->REJ_received, s->DISC_received);
    } else {
        printf("SET: %u, I: %u, DISC: %u, UA: %u\n", s->SET_received, s->I_frames_received, s->DISC_received, s->UA_received);
    }

    printf("\nTotals\n");
    printf("  Total Frames Sent: %u\n", s->frames_sent);
    printf("  Total Frames Received: %u\n", s->frames_received);
    printf("  Timeouts: %u\n", s->timeouts);
    printf("  Retransmissions: %u\n", s->retransmissions);
    if (s->duration > 0.0) {
        printf("  Duration: %.3f s\n", s->duration);
    }
    printf("\n");
}
