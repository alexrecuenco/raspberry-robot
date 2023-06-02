from gpiozero import Servo

MOTOR_R_PIN = 18
MOTOR_L_PIN = 13

LEFT = 1.3  # ms
RIGHT = 1.7  # ms
ZERO = 1.5  # ms
# class SpecialServo(Servo):
#     def __init__(self, pin=None, initial_value=0, min_ms=1 / 1000, max_ms=2 / 1000, center_ms, frame_width=20 / 1000, pin_factory=None,):
#         min_pulse_width=min_ms/1000
#         max_pulse_width=max_ms/1000
#         super().__init__(pin, initial_value, min_pulse_width, max_pulse_width, frame_width, pin_factory)

#     pass


def main():
    servoR = Servo(
        MOTOR_R_PIN,
        initial_value=None,
        min_pulse_width=LEFT / 1000,
        max_pulse_width=RIGHT / 1000,
        frame_width=20 / 1000,
    )
    servoL = Servo(
        MOTOR_L_PIN,
        initial_value=None,
        min_pulse_width=LEFT / 1000,
        max_pulse_width=RIGHT / 1000,
        frame_width=20 / 1000,
    )

    def run():
        servoL.value = -0.4
        servoR.value = -0.4


if __name__ == "__main__":
    main()
