#include "control.h"
#include "helper.h"
#include "motor.h"
#include <signal.h>
#include <stdbool.h>

int get_distances(double* left_out, double* right_out)
{
    int err_left = 0;
    int err_right = 0;
    *left_out = distance_atomic_count(SENSOR_L, &err_left);
    *right_out = distance_atomic_count(SENSOR_R, &err_right);

    if ((err_left < 0) || (err_right < 0)) {
        fprintf(stderr, "Error when counting sleep, err_left %d, err_right %d\n", err_left, err_right);

        return -5;
    }
    return 0;
}

int run(int speed_increment, int integration_time, int max_speed, int direction)
{
    int speed = speed_increment;
    fprintf(stderr, "speed_tick, speed_left, speed_right\n");
    while (speed < max_speed) {
        double left = 0;
        double right = 0;
        if (get_distances(&left, &right) < 0) {
            return -1;
        };
        left = right = 0;
        int true_speed = direction * speed;
        if (set_wheel_moving(true_speed) < 0) {
            return -2;
        }

        int init = millis();

        delay(integration_time);
        if (set_wheel_moving(0) < 0) {
            return -3;
        }
        if (get_distances(&left, &right) < 0) {
            return -4;
        };

        int end = millis();

        delay(1);
        double speed_l = left / (end - init);
        double speed_r = right / (end - init);
        fprintf(stderr, "%d, %f, %f\n", true_speed, speed_l, speed_r);
        speed += speed_increment;
    }
    return 0;
}

int main(int argc, char* argv[])
{
    int speed_increment = MIN_TICK_US;

    if (argc >= 2) {
        speed_increment = atoi(argv[1]);
    }
    int integration_time = 5;
    if (argc >= 3) {
        integration_time = atoi(argv[2]);
    }
    int speed_total = 100;
    if (argc >= 4) {
        speed_total = atoi(argv[3]);
    }

    fprintf(stderr, "settings tick, integration_time, max_speed %d, %d, %d\n", speed_increment, integration_time, speed_total);

    if (startup() < 0) {
        return -6;
    }

    int result = run(speed_increment, integration_time, speed_total, 1);
    if (result >= 0) {
        result = run(speed_increment, integration_time, speed_total, -1);
    }
    shutdown();
    return -result;
}
