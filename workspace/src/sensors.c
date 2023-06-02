
/**
 * See https://en.cppreference.com/w/c/thread
 */
#include "sensors.h"
#include "helper.h"
#include "motor.h"
#include "wiringPi.h"
#include <stdatomic.h>
#include <stdbool.h>
#include <threads.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

atomic_bool stop = 0;
atomic_int counter_l = 0;
atomic_int counter_r = 0;

// mm measurements
atomic_int motion_len_l = 1000000;
atomic_int motion_len_r = 1000000;

/**
 *
CALIBRATION

distance_cm, 0, 1
1, 753, 640
2, 513, 861
3, 870, 870
4, 870, 870
5, 870, 778
6, 870, 669
7, 870, 583
8, 870, 508
9, 870, 423
10, 810, 434
11, 740, 367
12, 690, 348
13, 636, 320
14, 597, 291
15, 565, 270
16, 530, 261
17, 503, 237
18, 475, 221
19, 453, 200
20, 442, 180
21, 413, 180
22, 395, 180
23, 392, 180
24, 378, 180
25, 366, 150
26, 355, 150
27, 344, 150
28, 330, 150
29, 324, 140
30, 308, 140
35, 266, 100
40, 243, 108
45, 229, 100
50, 205, 100
70, 200, 100
90, 200, 100
100, 200, 100

FULL NOISE FROM 120

NOISE
idx, min, max
0, 320, 420
1, 0, 330


10000000000,


MAX/MIN of wheels turning
idx, min, max
2, 161, 695
3, 50, 550,
*/

#define SWITCH_POINT_L 380
#define SWITCH_POINT_R 380

/* Doing this for 15cm */
#define OBSTACLE_PROXIMITY_L 600
#define OBSTACLE_PROXIMITY_R 600
#define IGNORE_PROXIMITY_L 200
#define IGNORE_PROXIMITY_R 200

bool should_print_sensor(void)
{
    return get_default_var("DEBUG_SENSORS", 0) != 0;
}

thrd_t t;
bool debug_print = false;

int sensorThread(void* arg)
{
    if (debug_print) {
        fprintf(stdout, "adc0, adc1, adc2, adc3\n");
    }

    // TODO: Use https://en.cppreference.com/w/c/chrono/timespec and https://en.cppreference.com/w/c/chrono/timespec_get to get precise delay increments
    while (!stop) {
        unsigned int last_time = millis();

        int adc0 = analogRead(100);
        int adc1 = analogRead(101);
        int adc2 = analogRead(102);
        int adc3 = analogRead(103);

        writeMotionCount(MOTION_SENSOR_L, adc0);
        writeMotionCount(MOTION_SENSOR_R, adc1);
        writeWheelCount(SENSOR_L, adc2);
        writeWheelCount(SENSOR_R, adc3);
        if (debug_print) {
            fprintf(stdout, "%d, %d, %d, %d\n", adc0, adc1, adc2, adc3);
        }
        wait_delay(10, last_time);
    }
    // for speed 80 -> 1v : 1.36s -> 68ms for round(1.36/20*1000)
    return 0;
}

int motion_sensor(MotionSensor pin)
{
    switch (pin) {
    case MOTION_SENSOR_L:
        return motion_len_l;
    case MOTION_SENSOR_R:
        return motion_len_r;
    default:
        return -1;
    }
}

int moving_update(double measure, double previous_moving_average, double weight)
{
    return (weight - 1.0) * previous_moving_average / weight + 1.0 / weight * measure;
}
#define EXP_AVR_WEIGHT 3.0
bool should_sensor_count(double* moving_count, bool* is_high, double switch_point, int measure)
{
    (*moving_count) = moving_update(measure, *moving_count, EXP_AVR_WEIGHT);
    bool new_is_high = (*moving_count) > switch_point;
    if (new_is_high != *is_high) {
        *is_high = new_is_high;
        return true;
    }
    return false;
}

int writeWheelCount(WheelSensor pin, int measure)
{
    static double moving_count_l = 0;
    static double moving_count_r = 0;
    static bool is_high_l = true;
    static bool is_high_r = false;
    switch (pin) {
    case SENSOR_L:
        if (should_sensor_count(&moving_count_l, &is_high_l, SWITCH_POINT_L, measure)) {
            counter_l++;
        }
        return 0;
    case SENSOR_R:
        if (should_sensor_count(&moving_count_r, &is_high_r, SWITCH_POINT_R, measure)) {
            counter_r++;
        }
        return 0;
    default:
        return -1;
    }
}

#define MOTION_EXP_AVR_WEIGHT 10
bool is_object_nearby(double* moving_count, double switch_point, int measure)
{
    (*moving_count) = moving_update(measure, *moving_count, MOTION_EXP_AVR_WEIGHT);
    bool is_nearby = (*moving_count) > switch_point;
    return is_nearby;
}

int writeMotionCount(MotionSensor pin, int measure)
{
    static double motion_sensor_l = 0;
    static double motion_sensor_r = 0;

    switch (pin) {
    case MOTION_SENSOR_L:
        if (measure < IGNORE_PROXIMITY_L) {
            // ignore
            return 0;
        }
        if (is_object_nearby(&motion_sensor_l, OBSTACLE_PROXIMITY_L, measure)) {
            // for now we just check if higher/lower, we would need to switch this with a proper measurmeent
            motion_len_l = 0;
        } else {
            motion_len_l = 100000;
        }

        return 0;
    case MOTION_SENSOR_R:
        if (measure < IGNORE_PROXIMITY_R) {
            // ignore
            return 0;
        }
        if (is_object_nearby(&motion_sensor_r, OBSTACLE_PROXIMITY_R, measure)) {
            // for now we just check if higher/lower, we would need to switch this with a proper measurmeent
            motion_len_r = 0;
        } else {
            motion_len_r = 100000;
        }

        return 0;
    default:
        return -1;
    }
}

int wheelCounter(WheelSensor pin)
{
    switch (pin) {
    case SENSOR_L:
        return counter_l;
    case SENSOR_R:
        return counter_r;
    default:
        return -1;
    }
}

bool has_obstacle(int d_mm)
{
    return ((motion_sensor(MOTION_SENSOR_L) < d_mm) || motion_sensor(MOTION_SENSOR_R) < d_mm);
}

int reset_count(WheelSensor pin)
{
    switch (pin) {
    case SENSOR_L:
        return atomic_exchange(&counter_l, 0);
    case SENSOR_R:
        return atomic_exchange(&counter_r, 0);
    }
    return -1;
}

int start_sensors(void)
{
    debug_print = should_print_sensor();
    // Here we set the speed we expect on channel 0. Channels can be either 0 or 1?
    if (wiringPiSPISetup(0, 500000) < 0) {
        return -1;
    }
    // The pin base is just a random variable number used internally in wiringpi to identify the nodes you create
    // When you call analogRead it will do a search through them, looking for pinBase+pinId to find your pin
    // pinId in the mcp3004 case is a number between 0 and 3
    // WARNING: On read, if a pin is outside of range, it just returns zero as a value.
    // The channel can be either 0 or 1, any other value gets casted down
    if (mcp3004Setup(100, 0) == FALSE) {
        return -1;
    }
    if (t != NULL) {
        fprintf(stderr, "Check your code, you are likely breaking something by trying to create this twice\n");
        return -1;
    }
    // We are now just creating a fake one
    int rs = thrd_create(&t, sensorThread, NULL);
    if (rs != thrd_success) {
        return -1;
    }
    fprintf(stderr, "Started sensors\n");

    return 0;
}

int ask_stop(void)
{
    fprintf(stderr, "Asked for sensor stop\n");
    stop = 1;
    if (t == NULL)
        return -1;

    if (thrd_join(t, NULL) != thrd_success) {
        return -1;
    }
    t = NULL;
    return 0;
}
