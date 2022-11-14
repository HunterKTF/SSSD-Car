#!/usr/bin/env python3
'''
This is a blinker test.

The code uses Pin 3 (GPIO 2) to make a LED blink.
'''

# import standard libraries
import RPi.GPIO as gpio
import time

# set gpio mode
gpio.setmode(gpio.BOARD)
gpio.setup(3, gpio.OUT)

# initialize loop
while True:
  gpio.output(3, gpio.HIGH)
  time.sleep(1)
  gpio.output(3, gpio.LOW)
  time.sleep(1)