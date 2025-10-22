#include <unistd.h>
#include <signal.h>
#include <stdio.h>
#include "alarm.h"

// Global variable to track if the alarm is enabled
int alarm_enabled = FALSE;
// Counter for the number of times the alarm has been triggered
int alarm_count = 0;

// Alarm function handler
// This function is called when the alarm signal is received
void alarm_handler(int signal)
{
    alarm_enabled = FALSE; // Disable the alarm
    alarm_count++;         // Increment the alarm trigger count
}