# from gpiozero import LED, Button
# from signal import pause
import contextlib
from time import sleep

import RPi.GPIO as GPIO  # Import Raspberry Pi GPIO library
from flask import Flask

HARDCODED_BUTTON = 19
HARDCODED_LED = 26

# led = LED(26)
# button = Button(19)


# button.when_pressed = led.on
# button.when_released = led.off


@contextlib.contextmanager
def cleanup():
    try:
        yield
    finally:
        GPIO.cleanup()


def on():
    GPIO.output(HARDCODED_LED, 1)


def off():
    GPIO.output(HARDCODED_LED, 0)


def button_callback(channel):
    if not GPIO.input(HARDCODED_BUTTON):
        print("rising", GPIO.input(HARDCODED_BUTTON))
        on()
    else:
        print("FALLING")
        GPIO.output(HARDCODED_LED, 0)
        off()

    print("release was pushed!", channel)


# pause()
def main():
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(HARDCODED_LED, GPIO.OUT)
    GPIO.setup(HARDCODED_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    GPIO.add_event_detect(HARDCODED_BUTTON, GPIO.BOTH, callback=button_callback)
    on()
    sleep(1)
    off()
    app.run()


app = Flask(__name__)


@app.route("/")
def hello():
    return "Hello, World!"


@app.route("/on")
def turnOn():
    on()
    return "On"


@app.route("/off")
def turnOff():
    off()
    return "off"


if __name__ == "__main__":
    with cleanup():
        main()
