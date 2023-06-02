#ifndef CONTROL_H
#define CONTROL_H

#include "sensors.h"
#include <stdbool.h>

#define IGNORE_ANGLE 10000

typedef struct {
    double x;
    double y;
    double theta;
} Point;

extern double dist(Point p1, Point p2);

extern void copy_point(Point p1, Point* p2);

extern int debug_point(Point p, const char* idx);

extern double peek_distance_counter(WheelSensor pin, int* errorCode);

extern double distance_atomic_count(WheelSensor pin, int* errorCode);

extern int atomic_update_point(Point p_init, Point* result);

extern int peek_update_point(Point p_init, Point* result);

extern int reset_motion(void);

extern int move_from_to(Point from, Point to, int speed, Point* result);

extern int go_around(int speed, Point init, Point* result);

#endif
