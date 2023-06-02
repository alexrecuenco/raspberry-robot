#include "../src/control.h"
#include "../src/helper.h"
#include "../src/motor.h"
#include "../src/sensors.h"

#include <signal.h>
#include <stdbool.h>

int select(int speed)
{
    int input;
    fprintf(stderr, "Enter a number:\n\t1:\tmove forward\n\t2:\tmove backwards\n\t3:\tturn left\n\t4:\tturn right\n");
    scanf("%d", &input);
    switch (input) {
    case 1:
        if (set_wheel_moving(speed) < 0) {
            return -127;
        }
        return 0;
    case 2:
        if (set_wheel_moving(-speed) < 0) {
            return -126;
        }
        return 0;
    case 3:
        if (set_wheel_turning(speed) < 0) {
            return -125;
        }

        return 0;
    case 4:
        if (set_wheel_turning(-speed) < 0) {
            return -124;
        }

        return 0;
    default:
        return -255;
    }
}

int run(int speed, int time_s)
{
    bool cont = true;
    while (cont) {
        Point p_init = { 0.0, 0.0, 0.0 };
        Point p_out;
        if (set_wheel_moving(0) < 0) {
            return -128;
        }

        int result = select(speed);
        if (result < 0) {
            return result;
        }

        int i = 0;
        while (i++ < time_s * 10) {
            delay(100);
            peek_update_point(p_init, &p_out);
            debug_point(p_out, "debug-temp");
        }
        atomic_update_point(p_init, &p_out);
        debug_point(p_out, "p_out");
    }

    return 0;
}

int main(int argc, char* argv[])
{
    int speed = 20;

    if (argc >= 2) {
        speed = atoi(argv[1]);
    }
    int time = 2;
    if (argc >= 3) {
        time = atoi(argv[2]);
    }

    if (startup() < 0) {
        return -10;
    }

    int result = run(speed, time);
    shutdown();
    return -result;
}
