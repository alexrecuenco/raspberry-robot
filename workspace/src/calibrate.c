#include "helper.h"
#include "motor.h"
#include <signal.h>

void calibration_pulse(int pin1, int pin2, int left_us, int right_us)
{
    // Documentation for continuous servo
    // servo around 1500 μs pulses, a microcontroller program can be written that repeatedly sends:
    // ● 1485 μs pulses every 20 ms for 2 seconds
    // ● 1515 μs pulses every 20 ms for 2 seconds.
    while (1) {
        set_to(pin1, left_us);
        set_to(pin2, left_us);
        delay(2000);
        set_to(pin1, right_us);
        set_to(pin2, right_us);
        delay(2000);
    }
}

void ioduty(int pin1, int pin2, int min, int max)
{
    int input = 0;

    do {
        fprintf(stderr, "Enter a number for %d: ", wpiPinToGpio(pin1));
        scanf("%d", &input);

        if ((input < min || input > max) && input != 0) {
            fprintf(stderr, "Set duty cycle outside spec range\n");
            break;
        }
        set_to(pin1, input);

        fprintf(stderr, "Enter a number for %d: ", wpiPinToGpio(pin2));
        scanf("%d", &input);

        if ((input < min || input > max) && input != 0) {
            fprintf(stderr, "Set duty cycle outside spec range\n");
            break;
        }
        set_to(pin2, input);
    } while (1);
}
void calibrate(int pin1, int pin2)
{
    fprintf(stderr, "Calibrating PINs for continous servo GPIO %d, GPIO %d\n", wpiPinToGpio(pin1), wpiPinToGpio(pin2));
    ioduty(pin1, pin2, SPEC_MIN_US, SPEC_MAX_US);
    calibration_pulse(pin1, pin2, 1490, 1510);
}

int run(int argc, char* argv)
{
    calibrate(MOTOR_L, MOTOR_R);
}

int main(int argc, char* argv[])
{
    if (startup() < 0) {
        return -10;
    }

    int result = run(argc, argv);
    shutdown();
    return -result;
}
