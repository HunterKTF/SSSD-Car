# Import standard libraries
import RPi.GPIO as GPIO
import pygame
import sys

from pygame.locals import *
from time import sleep

# Import local libraries
from pin_map import *

class Steering:
    def __init__(self):
        # Define mapping values for XBox One Controller Trigger
        self.left_max_t = 1
        self.left_min_t = -1.000030518509476
        self.right_max_t = 100
        self.right_min_t = 0

        # Define mapping values for XBox One Controller Joystick
        self.left_max_j = 1
        self.left_min_j = -1.000030518509476
        self.right_max_j = 90
        self.right_min_j = -90
        
        # Initialize pygame joystick object
        self.joysticks = None
        
        # Initialize movement variables
        self.speed = 0
        self.diff_steering = 0
        
        # Create PWM signals for motors
        self.pi_pwm_rl = None		# PWM signal for Rear Left motor
        self.pi_pwm_rr = None		# PWM signal for Rear Right motor
        self.pi_pwm_fr = None		# PWM signal for Front Right motor
        self.pi_pwm_fl = None		# PWM signal for Front Left motor
        
    def init_sim_params(self):
        # Initialize pyagme
        pygame.init()
        
        # Get joysticks connected to the Raspberry Pi
        self.joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
        
        ## Set GPIO as OUTPUT
        # Set board mode and warnings
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
        GPIO.setup(PWM1_1, GPIO.OUT)	# ENA
        GPIO.setup(GPIO_22, GPIO.OUT)	# IN1
        GPIO.setup(GPIO_23, GPIO.OUT)	# IN2
        
        # Set PWM at 1000 frequency
        self.pi_pwm_rl = GPIO.PWM(PWM0_0, 1000)
        self.pi_pwm_rr = GPIO.PWM(PWM0_1, 1000)
        self.pi_pwm_fr = GPIO.PWM(PWM1_0, 1000)
        self.pi_pwm_fl = GPIO.PWM(PWM1_1, 1000)
        
    @staticmethod
    def mapping(left_max, left_min, right_max, right_min, value):
        left_span = left_max - left_min
        right_span = right_max - right_min
        value_scaled = float(value - left_min) / float(left_span)
        return right_min + value_scaled * right_span
    
    # Function to enable forward movement
    def move_forward(self, value):
        # Speed mapping using the trigger input
        self.speed = int(
            self.mapping(self.left_max_t, self.left_min_t,
                         self.right_max_t, self.right_min_t,
                         value))
        print("Direct at", self.speed)	# Displays PWM speed
        
        # Set config for forward movement [Rear-Left]
        GPIO.output(GPIO_2, GPIO.HIGH)
        GPIO.output(GPIO_3, GPIO.LOW)

        # Set config for forward movement [Rear-Right]
        GPIO.output(GPIO_14, GPIO.HIGH)
        GPIO.output(GPIO_4, GPIO.LOW)

        # Set config for forward movement [Front-Right]
        GPIO.output(GPIO_17, GPIO.LOW)
        GPIO.output(GPIO_27, GPIO.HIGH)

        # Set config for forward movement [Front-Left]
        GPIO.output(GPIO_22, GPIO.LOW)
        GPIO.output(GPIO_23, GPIO.HIGH)
        
    # Function to enable reverse movement
    def move_backward(self, value):
        # Speed mapping using the trigger input
        self.speed = int(
            self.mapping(self.left_max_t, self.left_min_t,
                         self.right_max_t, self.right_min_t,
                         value))
        print("Reverse at", self.speed)
        
        # Set config for reverse movement [Rear-Left]
        GPIO.output(GPIO_2, GPIO.LOW)
        GPIO.output(GPIO_3, GPIO.HIGH)

        # Set config for reverse movement [Rear-Rright]
        GPIO.output(GPIO_14, GPIO.LOW)
        GPIO.output(GPIO_4, GPIO.HIGH)

        # Set config for reverse movement [Front-Right]
        GPIO.output(GPIO_17, GPIO.HIGH)
        GPIO.output(GPIO_27, GPIO.LOW)

        # Set config for reverse movement [Front-Left]
        GPIO.output(GPIO_22, GPIO.HIGH)
        GPIO.output(GPIO_23, GPIO.LOW)
        
    # Function to get steering direction
    def steer_vehicle(self, value):
        # Steering mapping using joystick input
        self.diff_steering = int(
            self.mapping(self.left_max_j, self.left_min_j,
                         self.right_max_j, self.right_min_j,
                         value))
        print("Steering")
        print(self.diff_steering)
        
    # Function to check events in pygame via the controller
    def check_end_event(self):
        for event in pygame.event.get():
			# Event type quit or close tab
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
                
            # Event type key 'ESC'
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    pygame.quit()
                    sys.exit()
                
            # Event type controller button push
            if event.type == JOYBUTTONDOWN:
                if event.button == 0:
                    print("Pressed A button")
            
            # Event type axis motion
            if event.type == JOYAXISMOTION:
				# Left trigger input
                if event.axis == 5:
                    self.move_backward(event.value)
                    
                # Right trigger input
                if event.axis == 4:
                    self.move_forward(event.value)
                    
                # Left joystick horizontal input
                if event.axis == 0:
                    self.steer_vehicle(event.value)
                    
    def vehicle_input(self):
        if self.diff_steering == 0:
            self.pi_pwm_rl.start(self.speed)
            self.pi_pwm_rr.start(self.speed)
            self.pi_pwm_fr.start(self.speed)
            self.pi_pwm_fl.start(self.speed)
        if self.diff_steering > 0:
            diff_steer = self.speed - self.diff_steering
            if diff_steer < 0:
                diff_steer = 0
            if diff_steer > 100:
                diff_steer = 100
            self.pi_pwm_rl.start(self.speed)
            self.pi_pwm_rr.start(diff_steer)
            self.pi_pwm_fr.start(diff_steer)
            self.pi_pwm_fl.start(self.speed)
        if self.diff_steering < 0:
            diff_steer = self.speed + self.diff_steering
            if diff_steer < 0:
                diff_steer = 0
            if diff_steer > 100:
                diff_steer = 100
            self.pi_pwm_rl.start(diff_steer)
            self.pi_pwm_rr.start(self.speed)
            self.pi_pwm_fr.start(self.speed)
            self.pi_pwm_fl.start(diff_steer)
        
#car = Steering()
#car.init_sim_params()

#while True:
 #   car.check_end_event()
  #  car.vehicle_input()
