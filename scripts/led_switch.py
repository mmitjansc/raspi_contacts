#!/usr/bin/env python

import RPi.GPIO as GPIO
import time

led = 12

try:

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(led, GPIO.OUT)

    while True:
        GPIO.output(led, GPIO.HIGH)
        time.sleep(0.3)
        GPIO.output(led, GPIO.LOW)
        time.sleep(0.3)

finally:
    GPIO.cleanup()
    print("GPIOs cleaned up.")