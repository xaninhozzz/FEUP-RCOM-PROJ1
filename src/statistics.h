#ifndef STATISTICS_H
#define STATISTICS_H

#include <stdio.h>

typedef struct {
    unsigned int frames_sent;           
    unsigned int frames_received;       

    unsigned int SET_sent;
    unsigned int SET_received;
    unsigned int UA_sent;
    unsigned int UA_received;
    unsigned int RR_sent;
    unsigned int RR_received;
    unsigned int REJ_sent;
    unsigned int REJ_received;
    unsigned int I_frames_sent;         
    unsigned int I_frames_received;
    unsigned int DISC_sent;
    unsigned int DISC_received;

    unsigned int timeouts;              
    unsigned int retransmissions;       

    double start_time;                      
    double end_time;                        
    double duration;                        
} ll_statistics;

void stats_init(ll_statistics *s);
void stats_update_duration(ll_statistics *s);
void print_statistics(const ll_statistics *s);
// is_tx: 1 if transmitter, 0 if receiver
void print_statistics_for_role(const ll_statistics *s, int is_tx);

#endif /* STATISTICS_H */
