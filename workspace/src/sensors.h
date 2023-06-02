#ifndef SENSORS_H
#define SENSORS_H

#include "wiringPins.h"
#include <stdatomic.h>
#include <stdbool.h>

#define SENSOR_PIN_L MOTOR_L
#define SENSOR_PIN_R MOTOR_R

// You can also do enum EnumAlphabet {A, B, C}... but then you refer to it always as enum EnumAlphabet
// Here we are declaring an anonymous enum, and setting Sensor as a global alias
typedef enum {
    SENSOR_L = SENSOR_PIN_L,
    SENSOR_R = SENSOR_PIN_R
} WheelSensor;
typedef enum {
    MOTION_SENSOR_L = SENSOR_PIN_L,
    MOTION_SENSOR_R = SENSOR_PIN_R
} MotionSensor;

extern int wheelCounter(WheelSensor pin);
extern int reset_count(WheelSensor pin);

extern int motion_sensor(MotionSensor pin);

extern bool has_obstacle(int d_mm);

extern int start_sensors(void);
/**
 * Ask stop requests the thread to stop
 * It is not guaranteed it will end, doing it correctly is hard.
 */
extern int ask_stop(void);

#endif
