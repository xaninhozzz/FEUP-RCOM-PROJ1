#ifndef _ALARM_H_
#define _ALARM_H_

// Boolean constants for TRUE and FALSE
#define FALSE 0
#define TRUE 1

// Extern declarations for global variables to track alarm state and count
extern int alarm_enabled; // Indicates whether the alarm is currently enabled
extern int alarm_count;   // Counts how many times the alarm has been triggered

// Function declaration for the alarm signal handler
// This function will handle the signal when the alarm goes off
void alarm_handler(int signal);

#endif // _ALARM_H_
