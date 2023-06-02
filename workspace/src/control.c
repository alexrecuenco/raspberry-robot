#include "control.h"
#include "helper.h"
#include "motor.h"
#include "sensors.h"
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <wiringPi.h>

#define CONTROL_MS_CLOCK 40

/**
 * CHAT GPT
 * ./main 2>&1 >/dev/null | grep -Eo 'Point [0-9]+\.idx: \(x, y, theta\) [0-9\.]+, [0-9\.]+, [0-9\.]+' | awk -F'[ ,( )]' '{print $9, $10, $11}'
 * FIX
 * ./main 2>&1 >/dev/null | grep -Eo 'Point .+' | awk -F'[ ,( )]' '{print $9, $10, $11}'

*/
typedef bool (*is_interrupt)(void);

typedef int (*action)(double, int, is_interrupt, Point);

typedef struct actionStruct {
    action f;
    double param;
    int speed;
    is_interrupt is_interrupt;
    struct actionStruct* interrupt;
} actionNode;

double dist(Point p1, Point p2)
{
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return sqrt(dx * dx + dy * dy);
}

double angle_diff(Point p1, Point p2)
{
    double dtheta = p2.theta - p1.theta;
    return dtheta;
}

double angle_to(Point p1, Point p2)
{
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double angle_rads = atan2(dy, dx);
    double angle = angle_rads / PI * 180.0;
    return angle;
}

double distance_sign(WheelSensor pin)
{
    return get_speed(pin) >= 0 ? 1.0 : -1.0;
}

void copy_point(Point p1, Point* p2)
{
    p2->x = p1.x;
    p2->y = p1.y;
    p2->theta = p1.theta;
}

int debug_action(actionNode a, const char* idx)
{
    fprintf(stderr, "[DEBUG] Action\t %s:\t \n\tf->%p, \n\tparam->%f, \n\tspeed->%d, \n\tis_interrpt->%p, \n\tinterrupt->%p\n", idx, a.f, a.param, a.speed, a.is_interrupt, a.interrupt);
    return 0;
}

int debug_point(Point p, const char* idx)
{
    fprintf(stderr, "[DEBUG] Point %s: (x, y, theta) %f, %f, %f\n", idx, p.x, p.y, p.theta);
    return 0;
}

int wait_target(Point p_init, bool (*has_reached)(Point, Point, double), double param, int delay_ms, bool (*interrupt)(void))
{
    int error = CONTROL_OK;
    bool reach = false;
    while (!reach) {
        int last_time = millis();
        Point now_point;
        error = peek_update_point(p_init, &now_point);
        if (error == UNKNOWN_ERROR) {
            break;
        }
        reach = has_reached(p_init, now_point, param);
        if (reach) {
            break;
        }
        if (interrupt()) {
            error = INTERRUPT;
            break;
        }
        wait_delay(delay_ms, last_time);
    }
    return error;
}

bool has_displaced(Point p_in, Point p_now, double d)
{
    return dist(p_now, p_in) >= d;
}

bool has_turned(Point p_in, Point p_now, double angle)
{
    double angle_moved = angle_diff(p_in, p_now);

    if (angle < 0) {
        return angle_moved < angle;
    }
    return angle_moved > angle;
}

int displacement(double (*counter)(WheelSensor, int*), Point p_init, Point* result)
{
    int errL = 0;
    int errR = 0;
    double countL = counter(SENSOR_L, &errL);
    double countR = counter(SENSOR_R, &errR);
    if ((errL < 0) || (errR < 0)) {
        fprintf(stderr, "Broken sensor reads. erros %d, %d. Reads L:  %f; R: %f \n", errL, errR, countL, countR);
        return UNKNOWN_ERROR;
    }
    copy_point(p_init, result);

    // fprintf(stderr, "\t\t Reads L:  %f; R: %f, d %f, angle %f \n", countL, countR, d, angle);
    integrate_move_point(result, countL, countR);
    return CONTROL_OK;
}

void integrate_move_point(Point* p, double countL, double countR)
{
    const int N_STEPS = 1000;
    const double dl = countL / (double)N_STEPS;
    const double dr = countR / (double)N_STEPS;
    const double distance_wheels_mm = 115.0;

    double x = p->x;
    double y = p->y;
    double angle_radians = p->theta * PI / 180.0;

    for (int i = 0; i < N_STEPS; i++) {
        double d = (dl + dr) / 2;
        double dx = d * cos(angle_radians);
        double dy = d * sin(angle_radians);
        x += dx;
        y += dy;
        angle_radians += (dr - dl) / distance_wheels_mm; // dl - dr is with theta clockwise, dr - dl when counter-clockwise
    }
    p->x = x;
    p->y = y;
    p->theta = angle_radians / PI * 180.0;
}

void move_angle(Point* p, double angle)
{
    p->theta += angle;
}

double simplify_angle(double angle)
{
    angle = fmod(angle, 360.0);
    if (angle > 180.0) {
        angle = angle - 360.0;
    } else if (angle < -180.0) {
        angle = angle + 360.0;
    }
    return angle;
}

int move_internal(int (*move_robot)(int), int speed, bool (*interrupt)(void), Point p, bool (*has_reached)(Point, Point, double), double param)
{
    if (move_robot(speed) < 0) {
        return UNKNOWN_ERROR;
    }

    int result = wait_target(p, has_reached, param, CONTROL_MS_CLOCK, interrupt);

    return result;
}

const int countsPerLap = 20;
// 6.6cm wheel diameter measured experimentally
const double perimeter_wheel_mm = 66.0 * PI;
const double circle_circumference_mm = PI * 100.0;

double internal_calculate_distance(WheelSensor pin, int wheel_count, int* errorCode, bool debug)
{
    if (wheel_count < 0) {
        // error reading
        *errorCode = UNKNOWN_ERROR;
    } else {
        *errorCode = 0;
    }
    double distance_moved = perimeter_wheel_mm * ((double)wheel_count / (double)countsPerLap);
    double sign = distance_sign(pin);
    if (debug) {
        fprintf(stderr, "[DEBUG] Atomic count [pin:%d], %d, \t d %f, sign %f\n", pin, wheel_count, distance_moved, sign);
    }
    return sign * distance_moved;
}

double peek_distance_counter(WheelSensor pin, int* errorCode)
{
    int wheel_count = wheelCounter(pin);
    return internal_calculate_distance(pin, wheel_count, errorCode, false);
}

double distance_atomic_count(WheelSensor pin, int* errorCode)
{
    int wheel_count = reset_count(pin);
    return internal_calculate_distance(pin, wheel_count, errorCode, true);
}

int atomic_update_point(Point p_init, Point* result)
{
    return displacement(distance_atomic_count, p_init, result);
}

int peek_update_point(Point p_init, Point* result)
{
    return displacement(peek_distance_counter, p_init, result);
}

int move_forward(double d_mm, int speed, bool (*interrupt)(void), Point p)
{
    if (d_mm == 0) {
        return CONTROL_OK;
    }

    return move_internal(set_wheel_moving, speed, interrupt, p, has_displaced, d_mm);
}

int turn_left(double degrees, int speed, bool (*interrupt)(void), Point p)
{
    if (degrees < 0) {
        fprintf(stderr, "Degrees must be positive calling turn left. %f", degrees);
        return UNKNOWN_ERROR;
    }
    return turn_in_place(degrees, speed, interrupt, p);
}

int turn_right(double degrees, int speed, bool (*interrupt)(void), Point p)
{
    if (degrees < 0) {
        fprintf(stderr, "Degrees must be positive calling turn right. %f", degrees);
        return UNKNOWN_ERROR;
    }
    return turn_in_place(-degrees, -speed, interrupt, p);
}

int turn_in_place(double degrees, int speed, bool (*interrupt)(void), Point p)
{
    if (degrees == 0) {
        return CONTROL_OK;
    }
    if (fabs(degrees) >= IGNORE_ANGLE) {
        fprintf(stderr, "[WARN] THAT ANGLE IS TOO BIG, IGNORING (angle %f)", degrees);
        return CONTROL_OK;
    }
    return move_internal(set_wheel_turning, speed, interrupt, p, has_turned, degrees);
}

int reset_motion(void)
{
    if (set_wheel_moving(0) < 0) {
        return UNKNOWN_ERROR;
    }

    reset_count(SENSOR_L);
    reset_count(SENSOR_R);
    return CONTROL_OK;
}

int run_actions(int count, actionNode actions[], Point p_init, Point* result)
{
    int i = 0;
    Point p;
    copy_point(p_init, &p);
    fprintf(stderr, "\n\n[DEBUG] RUNNING %d ACTIONS\n\n", count);

    int errorCode = CONTROL_OK;

    while ((i < count) && (errorCode >= 0)) {
        actionNode action = actions[i];
        debug_action(action, "input");

        errorCode = action.f(action.param, action.speed, action.is_interrupt, p);
        fprintf(stderr, "[DEBUG] executed action i: %d; returned code: %d\n", i, errorCode);

        if (errorCode == UNKNOWN_ERROR) {
            fprintf(stderr, "Unknown Error when executing step %d\n", i);
            return UNKNOWN_ERROR;
        }
        if (errorCode == INTERRUPT) {
            actionNode* interrupt = action.interrupt;
            if (interrupt == NULL) {
                break;
            }
            debug_action(*interrupt, "interrupt");
            errorCode = interrupt->f(interrupt->param, interrupt->speed, interrupt->is_interrupt, p_init);
        }

        if (errorCode == RETRY) {
            continue;
        }

        i++;

        // Update p, using temporary variable p_out
        Point p_temp;
        atomic_update_point(p, &p_temp);
        copy_point(p_temp, &p);
        p.theta = simplify_angle(p.theta);

        if (reset_motion() < 0) {
            fprintf(stderr, "Error when resetting motion between actions\n");
            return UNKNOWN_ERROR;
        }
    }

    copy_point(p, result);
    debug_point(*result, "action.result");
    fprintf(stderr, "\n\n");

    return errorCode;
}

bool is_obstacle_interrupt()
{
    return has_obstacle(1500);
}

bool null_interrupt()
{
    return false;
}

int wait_obstacle(int time_to_wait_ms, bool (*obstacle_detector)(int), int d_mm, int delay_ms)
{
    int no_obstacle_last_time_ms = millis();
    int last_obstacle_ms = no_obstacle_last_time_ms;
    while ((last_obstacle_ms - no_obstacle_last_time_ms) < time_to_wait_ms) {
        bool obstacle = obstacle_detector(d_mm);
        if (obstacle) {
            last_obstacle_ms = millis();
            fprintf(stderr, "no_obstacle_last_time_ms %d, time_to_wait_ms %d, last_obstacle_ms %d\n", no_obstacle_last_time_ms, time_to_wait_ms, last_obstacle_ms);
        } else {
            return RETRY;
        }
        delay(delay_ms);
    }
    fprintf(stderr, "stopped. last  %d; last no obsttacle %d. Time waited %d \n", last_obstacle_ms, no_obstacle_last_time_ms, time_to_wait_ms);
    return INTERRUPT;
}

int wait_interrupt_action(double d_obstacle_mm, int speed_ignore, is_interrupt interrupt, Point to)
{
    if (set_wheel_moving(0) < 0) {
        return UNKNOWN_ERROR;
    }
    return wait_obstacle(1 * 1000, has_obstacle, (int)round(d_obstacle_mm) /* obstacle distance mm */, 40 /* delay checks ms*/);
}

void move_action_factory(actionNode* a, double d_mm, int speed, actionNode* interrupt)
{
    a->f = move_forward;
    a->speed = speed;
    a->param = d_mm;
    a->is_interrupt = is_obstacle_interrupt;
    a->interrupt = interrupt;
}
void interrupt_action_factory(actionNode* a)
{
    a->f = wait_interrupt_action;
    a->param = 1500.0; // distance measuring
    a->speed = 0;
    a->is_interrupt = null_interrupt;
    a->interrupt = NULL;
}

void turn_action_factory(actionNode* a, double degrees, int speed)
{
    a->is_interrupt = null_interrupt;
    a->speed = speed;
    a->interrupt = NULL;
    if (degrees < 0) {
        a->f = turn_right;
        a->param = -degrees;
    } else {
        a->param = degrees;
        a->f = turn_left;
    }
}

int move_from_to(Point from, Point to, int speed, Point* result)
{
    fprintf(stderr, "GOING TO %f, %f, %f\n", to.x, to.y, to.theta);
    fprintf(stderr, "GOING FROM %f, %f, %f\n", from.x, from.y, from.theta);

    double distance = dist(from, to);
    double orientation_angle = angle_to(from, to);
    double first_angle = orientation_angle - from.theta;
    double final_angle_turn = to.theta - orientation_angle;
    fprintf(stderr, "Trayectory change angle %f, move d %f, angle final %f\n", first_angle, distance, final_angle_turn);

    actionNode move_interupt;
    actionNode turn_to_go;
    actionNode turn_end;
    actionNode move;
    interrupt_action_factory(&move_interupt);
    turn_action_factory(&turn_to_go, first_angle, speed);
    move_action_factory(&move, distance, speed, &move_interupt);
    turn_action_factory(&turn_end, final_angle_turn, speed);

    actionNode actions[3] = { turn_to_go, move, turn_end };
    int run_result = run_actions(3, actions, from, result);

    return run_result;
}

int go_around(int speed, Point init, Point* result)
{
    // NOTA: La rutina de movimientos fijos de esquiva de obstáculos será la siguiente:
    // (1) el robot está parado,
    // (2) a continuación se acercará al obstáculo hasta una distancia de 15 cm de él,
    // (3) girará a la izquierda 90 grados,
    // (4) avanzará 40 cm,
    // (5) girará a la derecha 90 grados,
    // (7) avanzará 50 cm y considerará finalizada la rutina de esquiva.
    actionNode move_interupt;
    interrupt_action_factory(&move_interupt);

    actionNode action_0;
    // (3) girará a la izquierda 90 grados,
    turn_action_factory(&action_0, 90, speed);
    actionNode action_1;
    // (4) avanzará 40 cm,
    move_action_factory(&action_1, 400, speed, &move_interupt);
    actionNode action_2;
    // (5) girará a la derecha 90 grados,
    turn_action_factory(&action_2, -90, speed);
    actionNode action_3;
    // (7) avanzará 50 cm y considerará finalizada la rutina de esquiva.
    move_action_factory(&action_3, 500, speed, &move_interupt);
    actionNode actions[4] = {
        action_0,
        action_1,
        action_2,
        action_3,
    };
    return run_actions(4, actions, init, result);
}
