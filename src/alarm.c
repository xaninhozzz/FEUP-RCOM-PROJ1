#define _POSIX_SOURCE 1

#include "alarm.h"
#include <signal.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

// Global variables (use volatile int per project constraints)
int alarmActive = 0;
int alarmCount = 0;

// Internal signal handler
static void alarmHandler(int sig)
{
    alarmActive = 0;
    alarmCount++;
    // Keep handler minimal (async-signal-safe)
}

// Initializes the alarm system
void alarm_init(void)
{
    // Configure SIGALRM handler similarly to the classroom example
    struct sigaction act;
    memset(&act, 0, sizeof(act));
    sigemptyset(&act.sa_mask);
    act.sa_handler = &alarmHandler; // no SA_RESTART; read() will be interrupted
    act.sa_flags = 0;
    if (sigaction(SIGALRM, &act, NULL) == -1)
    {
        perror("sigaction");
    }
    alarmActive = 0;
    alarmCount = 0;
}

// Starts (or restarts) the alarm with timeout in seconds
void alarm_start(unsigned int seconds)
{
    alarm(seconds);
    alarmActive = 1;
}

// Stops any active alarm
void alarm_stop(void)
{
    alarm(0);
    alarmActive = 0;
}

// To remove act error:
// Include string.h and do memset(&act, 0, sizeof act)
// Call sigemptyset(&act.sa_mask)
