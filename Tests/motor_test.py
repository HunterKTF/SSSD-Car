#!/usr/bin/env python3

'''
Motor test using the Raspberry Pi 3B+.

This script uses the L295N motor dirver and requires 4 PWM
signals. The PWM pins in the board are GPIO 13, 19, 12 and
18. This test uses each individually.

'''

# Import standard libraries
import RPi.GPIO as GPIO
from time import sleep

# Import developer libraries
from pin_map import *

# Declare motor PWM pins
# 
# frequency = 1000   # Set PWM frequency in Hz

# Set gpio mode
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)



# Set pins for Rear Left motor:
GPIO.setup(PWM0_0, GPIO.OUT)	# ENA
GPIO.setup(GPIO_2, GPIO.OUT)	# IN2
GPIO.setup(GPIO_3, GPIO.OUT)	# IN1

# Set pins for Rear Right motor:
GPIO.setup(PWM0_1, GPIO.OUT)	# ENB
GPIO.setup(GPIO_4, GPIO.OUT)	# IN3
GPIO.setup(GPIO_14, GPIO.OUT)	# IN4

# Set pins for Front Right motor:
GPIO.setup(PWM1_0, GPIO.OUT)	# ENB
GPIO.setup(GPIO_17, GPIO.OUT)	# IN1
GPIO.setup(GPIO_27, GPIO.OUT)	# IN2

# Set pins for Front Left motor:
GPIO.setup(PWM1_1, GPIO.OUT)	# ENB
GPIO.setup(GPIO_22, GPIO.OUT)	# IN1
GPIO.setup(GPIO_23, GPIO.OUT)	# IN2



# Set config for forward movement [RL]
pi_pwm_0_0 = GPIO.PWM(PWM0_0, 1000)
GPIO.output(GPIO_2, GPIO.HIGH)
GPIO.output(GPIO_3, GPIO.LOW)

# Set config for forward movement [RR]
pi_pwm_0_1 = GPIO.PWM(PWM0_1, 1000)
GPIO.output(GPIO_14, GPIO.HIGH)
GPIO.output(GPIO_4, GPIO.LOW)

# Set config for forward movement [FR]
pi_pwm_1_0 = GPIO.PWM(PWM1_0, 1000)
GPIO.output(GPIO_17, GPIO.LOW)
GPIO.output(GPIO_27, GPIO.HIGH)

# Set config for forward movement [FL]
pi_pwm_1_1 = GPIO.PWM(PWM1_1, 1000)
GPIO.output(GPIO_22, GPIO.LOW)
GPIO.output(GPIO_23, GPIO.HIGH)



cycle = 50
pi_pwm_0_0.start(cycle)
pi_pwm_0_1.start(cycle)
pi_pwm_1_0.start(cycle)
pi_pwm_1_1.start(cycle)

while True:
  sleep(1)
  print("duty cycle: ", cycle)
