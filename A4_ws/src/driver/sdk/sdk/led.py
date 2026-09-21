#!/usr/bin/python3
# coding=utf8
import Jetson.GPIO as GPIO

LED_PIN = 24  # pin number corresponding to LED

mode = GPIO.getmode()
if mode == 1 or mode is None:  # whether the pin code is set
    GPIO.setmode(GPIO.BCM)  # set as BCM code

GPIO.setwarnings(False)  # close alarm print

GPIO.setup(LED_PIN, GPIO.OUT)  # set pin as output mode

def on():
    GPIO.output(LED_PIN, 0)

def off():
    GPIO.output(LED_PIN, 1)

def set(new_state):
    GPIO.output(LED_PIN, new_state)

if __name__ == "__main__":
    import time
    while True:
        on()
        time.sleep(1)
        off()
        time.sleep(1)
