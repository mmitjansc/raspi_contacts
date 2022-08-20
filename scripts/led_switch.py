#!/usr/bin/env python

import RPi.GPIO as GPIO
import time

led = 12

try:

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(led, GPIO.OUT)
    # GPIO.setup(switch, GPIO.IN)

    for i in range(10):
        GPIO.output(led, GPIO.HIGH)
        time.sleep(0.2)
        GPIO.output(led, GPIO.LOW)
        time.sleep(0.2)

finally:
    GPIO.cleanup()