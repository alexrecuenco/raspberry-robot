#include "motor.h"
#include "wiringPins.h"
#include <math.h>
#include <signal.h>
#include <stdarg.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <wiringPi.h>

void cleanup(int* pins, int pinc)
{
    for (int i = 0; i < pinc; i++) {
        pinMode(pins[i], INPUT);
    }
}

int duty_cycle(int target_us)
{
    if (target_us > SAFETY_MAX_US) {
        fprintf(stderr, "[WARN] TARGET outside safety range\n");
        target_us = SAFETY_MAX_US;
    } else if (target_us < SAFETY_MIN_US) {
        fprintf(stderr, "[WARN] TARGET outside safety range\n");
        target_us = SAFETY_MIN_US;
    }

    if (target_us % MIN_TICK_US != 0) {
        fprintf(stderr, "[WARN] TARGET not a multiple of MIN_TICK :(%d), approximating\n", MIN_TICK_US);
        return (int)round(target_us / (double)MIN_TICK_US);
    }
    return target_us / MIN_TICK_US;
}

int set_to_internal(int pin, int target_us)
{
    int pwm_cycle_count = duty_cycle(target_us);
    fprintf(stderr, "Setting PIN: %d (PGIO: %d) duty cycle to: %d us (%d)\n", pin, wpiPinToGpio(pin), target_us, pwm_cycle_count);
    pwmWrite(pin, pwm_cycle_count);
    return 0;
}

int set_to(int pin, int target_us)
{
    return set_to_internal(pin, target_us);
}

int set_wheel_moving(int speed)
{
    if (set_speed(MOTOR_L, speed, FORWARD) < 0)
        return -128;
    if (set_speed(MOTOR_R, speed, FORWARD) < 0)
        return -127;
    return 0;
}

int set_wheel_turning(int speed)
{
    if (set_speed(MOTOR_L, speed, BACKWARD) < 0)
        return -120;
    if (set_speed(MOTOR_R, speed, FORWARD) < 0)
        return -119;
    return 0;
}
/**
 * Setup system
 */
int setup(int* pins, int pinc)
{
    // TODO: Consider using wiringPiSetupGpio
    // wiringPiSetupGpio
    if (wiringPiSetup() < 0) {
        return -115;
    }

    fprintf(stderr, "PWM_CLOCK: %d\n", PWM_CLOCK);
    fprintf(stderr, "PWM_RANGE: %d\n", PWM_RANGE);
    fprintf(stderr, "FREQUENCY: %d\n", FREQUENCY);
    fprintf(stderr, "MIN_TICK_US: %d\n", MIN_TICK_US);
    fprintf(stderr, "SPEC_MIN_US: %d\n", SPEC_MIN_US);
    fprintf(stderr, "SPEC_MAX_US: %d\n", SPEC_MAX_US);

    /**
     * The Raspberry Pi PWM clock has a base frequency of 19.2 MHz.
     * This frequency, divided by the argument to pwmSetClock(),
     * is the frequency at which the PWM counter is incremented.
     * When the counter reaches a value equal to the specified range,
     * it resets to zero.
     *
     * https://raspberrypi.stackexchange.com/a/9725
     *
     * While the counter is less than the specified duty cycle,
     * the output is high,
     * otherwise the output is low.
     */
    for (int i = 0; i < pinc; i++) {
        pinMode(pins[i], PWM_OUTPUT);
        set_to(pins[i], 0);
    }

    pwmSetMode(PWM_MODE_MS);
    pwmSetClock(PWM_CLOCK);
    pwmSetRange(PWM_RANGE);
    // If we had 19.2e6/192/2000  we get 50Hz
    // Minimum tick is then 20ms, 20ms/4000 = 5 mu s
    // Total width is 20 000 mu s

    /**
     * Specs of servo:
     * Operational pulse control signal range: 500 μs to 2530 μs +/-20 μs
     * Approximate angular responses to pulse widths:
     * (620 μs, 1520 μs, 2420 μs) maps to (0°, 90°, 180°) with angles increasing from clockwise to counterclockwise
     */
    /**
     * Spec of continuous servo
     * 1300 us to 1700 us
     */
    return 0;
}
atomic_int motor_l_speed = 0;
atomic_int motor_r_speed = 0;

int get_speed(int pin)
{
    if (pin == MOTOR_L) {
        return motor_l_speed;
    } else if (pin == MOTOR_R) {
        // The right one is looking the opposite way
        return motor_r_speed;
    }
}
/**
 * Once calibrated, we set a number between 0 and 200, it will be rounded to the closest 5
 * CALIBRATION WOULD BE AROUND 1500 us according to spec
 * TODO: Create wrapper set_speed that in a loop modifies the speed to match what we expect
 */
int set_speed(int pin, int speed, Direction direction)
{
    int forward_speed = direction * speed;
    if (pin == MOTOR_L) {
        motor_l_speed = forward_speed;
    } else if (pin == MOTOR_R) {
        motor_r_speed = forward_speed;
        // The right one is looking the opposite way
    }

    // LEFT MOTOR:
    // Clockwise => going back
    // Counter-clockwise => going forward
    // RIGHT MOTOR:
    // Clockwise => going forward
    // Counter-clockwise => going back
    int counter_clockwise_speed;
    if (pin == MOTOR_L) {
        counter_clockwise_speed = forward_speed;
    } else if (pin == MOTOR_R) {
        counter_clockwise_speed = -forward_speed;
    }

    if (counter_clockwise_speed == 0) {
        return set_to(pin, 0);
    }

    int zero = 1500;
    int output = zero;
    if (counter_clockwise_speed > 200) {
        counter_clockwise_speed = 200;
    } else if (counter_clockwise_speed < -200) {
        counter_clockwise_speed = -200;
    }
    output = zero + counter_clockwise_speed;

    return set_to(pin, output);
}
