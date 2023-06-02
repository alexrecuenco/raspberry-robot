#include "control.h"
#include "helper.h"
#include "motor.h"
#include "sensors.h"
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wiringPi.h>

// TODO atoi replacement.

// atoi returns 0 hen the input is not a number, which is problematic behaviour

int get_default_speed(void)
{
    return get_default_var("SPEED", 30);
}

int get_n_tries(void)
{
    return get_default_var("N_TRIES", 1);
}

int get_init_point(Point* init)
{
    // Read the values of the environment variables
    int x = get_default_var("X_INIT", 0);
    int y = get_default_var("Y_INIT", 0);
    int theta = get_default_var("THETA_INIT", 0);

    init->x = (float)x;
    init->y = (float)y;
    init->theta = (float)theta;

    return 0;
}

int get_destination(int argc, char* argv[], Point* result)
{
    int x = 500;
    int y = 0;
    int theta = 0;
    int last_arg_parsed = 0;
    if (argc >= 2) {
        last_arg_parsed = 1;
        char* arg = argv[1];
        if (strcmp(arg, "turnleft") == 0) {
            x = 0;
            y = 0;
            theta = 90;
            if (argc >= 3) {
                last_arg_parsed = 2;
                theta = atoi(argv[2]);
            }
        } else if (strcmp(arg, "turnright") == 0) {
            x = 0;
            y = 0;
            theta = -90;
            if (argc >= 3) {
                last_arg_parsed = 2;
                theta = -atoi(argv[2]);
            }
        } else if (strcmp(arg, "move") == 0) {
            x = 100;
            y = 100;
            theta = 90;
            if (argc >= 5) {
                x = atoi(argv[2]);
                y = atoi(argv[3]);
                theta = atoi(argv[4]);
                last_arg_parsed = 4;
            }
        } else if (strcmp(arg, "movereckless") == 0) {
            x = 100;
            y = 100;
            // gynormous angle so that it is ignored
            theta = 10 * IGNORE_ANGLE;
            if (argc >= 4) {
                x = atoi(argv[2]);
                y = atoi(argv[3]);
                last_arg_parsed = 3;
            }
        } else {
            fprintf(stderr, "DEBUG move forward %d");
            x = atoi(argv[1]);
        }
    }
    result->x = (double)x;
    result->y = (double)y;
    result->theta = (double)theta;
    return last_arg_parsed;
}

/**
 *  Some chatgpt solution
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

int main() {
    char input[] = "123abc";
    char *endptr;
    errno = 0;

    long int result = strtol(input, &endptr, 10);

    if (errno == ERANGE) {
        printf("Number out of range.\n");
    }
    else if (*endptr != '\0') {
        printf("Invalid character: %c\n", *endptr);
    }
    else {
        printf("Result: %ld\n", result);
    }

    return 0;
}

** Then, convert to int using <limits.h> to get what is the INT_MIN and INT_MAX, and exit if the value can't be represented in int

#include <limits.h>
#include <stdio.h>

int main() {
    long int l = 1234567890;
    int i;

    if (l >= INT_MIN && l <= INT_MAX) {
        i = (int) l;
        printf("Converted to int: %d\n", i);
    }
    else {
        printf("Value is out of range for int.\n");
    }

    return 0;
}

*/

int execute_move_protocol(Point p_init, Point p_target, Point* p_result)
{
    int speed = get_default_speed();
    int n_tries = get_n_tries();
    Point p_now;
    copy_point(p_init, &p_now);
    int result = 0;
    Point p_out;
    copy_point(p_init, &p_out);

    while (n_tries > 0) {
        fprintf(stderr, "EXECUTING TRY %d\n", n_tries);
        n_tries--;
        result = move_from_to(p_now, p_target, speed, &p_out);
        fprintf(stderr, "RESULT MOVE? %d\n", result);
        int result_go_around = INTERRUPT;

        if (result == INTERRUPT) {
            fprintf(stderr, "[WARN] Executing interrupt\n");
            Point p_temp;

            int result = go_around(speed, p_out, &p_temp);
            if (result == UNKNOWN_ERROR) {
                fprintf(stderr, "[ERROR] Unkown error going around\n");
                return result;
            }
            if (result == INTERRUPT) {
                fprintf(stderr, "[WARN] Interrupt during interrupt\n");
            }

            copy_point(p_temp, &p_out);
        }

        if (result == UNKNOWN_ERROR) {
            fprintf(stderr, "Unkown error");
            return result;
        }

        debug_point(p_out, "p_out");
        debug_point(p_target, "p_target");
        double distance = dist(p_out, p_target);

        if (result == CONTROL_OK) {
            fprintf(stderr, "CONTROL_OK. d=%f\n", distance);
            break;
        }

        copy_point(p_out, &p_now);
    }
    copy_point(p_out, p_result);
    return result;
}

int run(int argc, char* argv[])
{
    int last_parsed = 0;
    int total_parsed = 0;
    Point p_init = { 0.0, 0.0, 0.0 };
    get_init_point(&p_init);

    int result = 0;
    int nruns = 0;
    while ((argc - total_parsed > 0) && (result >= 0)) {
        Point p_target;
        nruns++;
        fprintf(stderr, "Getting destination\n");
        int parsed = get_destination(argc - total_parsed, argv + total_parsed, &p_target);
        if (parsed < 2 && nruns > 1) {
            fprintf(stderr, "no more destinations\n");
            break;
        } else {
            total_parsed += parsed;
        }
        Point p_out;
        result = execute_move_protocol(p_init, p_target, &p_out);
        copy_point(p_out, &p_init);
    }

    return result;
}

int main(int argc, char* argv[])
{
    if (startup() < 0) {
        return 10;
    }

    int result = run(argc, argv);
    shutdown();
    return -result;
}
