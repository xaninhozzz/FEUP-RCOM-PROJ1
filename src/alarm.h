#ifndef _ALARM_H_
#define _ALARM_H_

// Global alarm state (simple ints, like the classroom example)
extern int alarmActive;   // 1 when alarm is active, 0 otherwise
extern int alarmCount;    // counts how many times the alarm expired

// Flag set by SIGALRM handler to signal timeout to waiting loops
#include <signal.h>
extern volatile sig_atomic_t alarm_flag;

// Initializes the alarm system (registers signal handler)
void alarm_init(void);

// Starts the alarm (timeout in seconds)
void alarm_start(unsigned int seconds);

// Stops the alarm
void alarm_stop(void);

#endif // _ALARM_H_
