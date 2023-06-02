
/**
 * See https://en.cppreference.com/w/c/thread
 */
#include "motor.h"
#include "sensors.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wiringPi.h>

void wait_delay(unsigned int target_wait_ms, unsigned int last_millis)
{
    unsigned int now = millis();
    int diff = now - last_millis;
    if (diff >= target_wait_ms) {
        fprintf(stderr, "wait_delay unable to keep up, your delay is too short compared to execution time");
        // just get a switch for LINUX to handle the thread
        delay(1);
        return;
    }
    int approx_wait_time = target_wait_ms - diff;
    delay(approx_wait_time);
}

int pins[2] = { MOTOR_L, MOTOR_R };
int pinc = 2;

void shutdown(void)
{
    if (ask_stop() < 0) {
        fprintf(stderr, "\n\t[ERROR] Stopping sensor thread cleanly didn't work\n");
    }
    cleanup(pins, pinc);
}
void sigint_handler(int signum)
{
    fprintf(stderr, "\n\tCleaning up after Ctrl+C\n");
    shutdown();
    exit(-1);
}

int startup(void)
{
    // LINUX specific
    signal(SIGINT, sigint_handler);
    fprintf(stderr, "MAIN PROGRAM\n");
    if (setup(pins, pinc) < -0) {
        fprintf(stderr, "Error setting up pins\n");
        return -10;
    }
    if (start_sensors() < 0) {
        fprintf(stderr, "Error starting sensors\n");
        return -20;
    }
    return 0;
}

int to_int(char* var, int default_value)
{
    return (var ? atoi(var) : default_value);
}
int get_default_var(const char* envvar, int default_value)
{
    char* envvar_val = getenv(envvar);
    return to_int(envvar_val, default_value);
}
