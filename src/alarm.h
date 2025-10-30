#ifndef _ALARM_H_
#define _ALARM_H_

// Global alarm state (updated by SIGALRM handler)
extern int alarmActive;   // 1 when alarm is active, 0 otherwise
extern int alarmCount;    // counts how many times the alarm expired

// Initializes the alarm system (registers signal handler)
void alarm_init(void);

// Starts the alarm
void alarm_start(unsigned int seconds);

// Stops the alarm
void alarm_stop(void);

#endif // _ALARM_H_
