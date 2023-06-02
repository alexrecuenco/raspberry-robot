#ifndef MOTOR_H
#define MOTOR_H

#include "wiringPins.h"
#include <math.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <wiringPi.h>

#define CLOCK_SPEED 19200000 // Hz
#define PWM_CLOCK 192 // Every how many clock ticks we tick one up
#define PWM_RANGE 2000 // how many tick ups to reset the count
// TO PREVENTI OVERFLOWS
#define TEMP_RATIO_CLOCK_PWM (CLOCK_SPEED / PWM_CLOCK) // Hzs
#define FREQUENCY (TEMP_RATIO_CLOCK_PWM / PWM_RANGE) // Hzs
// 19.2e6/192/2000 = 50Hz
// Minimum tick is 20ms, 20ms/2000 = 10 mu s
// Total width is 20 000 mu s
#define MIN_TICK_US (1000000 / TEMP_RATIO_CLOCK_PWM) // mu s
#define SPEC_MIN_US 1300 // mu s
#define SPEC_MAX_US 1700 // mu s
#define SAFETY_MIN_US 0 // mu s
#define SAFETY_MAX_US 10000 // mu s

typedef enum {
    FORWARD = 1,
    BACKWARD = -1,
} Direction;

extern void cleanup(int* pins, int pinc);

extern int duty_cycle(int target_us);

extern int set_to(int pin, int target_us);

extern int set_wheel_moving(int speed);
extern int set_wheel_turning(int speed);

/**
 * Setup system
 */
extern int setup(int* pins, int pinc);
/**
 * Once calibrated
 */
extern int set_speed(int pin, int speed, Direction direction);
extern int get_speed(int pin);

#endif
