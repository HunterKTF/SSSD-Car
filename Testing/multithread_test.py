# Import Standard libraries
import threading
from concurrent.futures import ThreadPoolExecutor
import time
import signal
#from adafruit_servokit import ServoKit
#import board
#import busio
import numpy as np
import cv2
#from xbox360controller import Xbox360Controller


#Import user defined libraries
import driving_feature as driving
import camera_test as cameraThread
import gps_test as gpsThread


# creating a lock
#lock = threading.Lock()

"""def on_axis_moved(axis):
    desired_angle = (axis.x+1)/2*180
    desired_angle = servoThread._map(desired_angle,0,180,55,125)
    kit.servo[0].angle = desired_angle
    print(desired_angle)
    print('Axis {0} moved to {1} {2}'.format(axis.name, axis.x, axis.y))
"""
# On the Jetson Nano
# Bus 0 (pins 28,27) is board SCL_1, SDA_1 in the jetson board definition file
# Bus 1 (pins 5, 3) is board SCL, SDA in the jetson definition file
# Default is to Bus 1; We are using Bus 0, so we need to construct the busio first ...
#print("Initializing Servos")
#print("Initializing ServoKit")
#kit = ServoKit(channels=16)
# kit[0] is the bottom servo
# kit[1] is the top servo
#print("Done initializing")
#time.sleep(0.5)

#motores
def sensor1():   
    car = driving.Steering()
    car.init_sim_params()
    while True:
        car.check_end_event()
        car.vehicle_input()

#camara
def sensor2():
    
    cap = cv2.VideoCapture(0) # check this 
    frame_width = int(cap.get(3))
    frame_height = int(cap.get(4))
    print(f"width {frame_width}, height {frame_height}")
    size = (frame_width, frame_height)
    result = cv2.VideoWriter('filename.avi', 
                         cv2.VideoWriter_fourcc(*'MJPG'),
                         30, size)
    while True:     
        cameraThread.video_cap(cap, result)


#gps
def sensor3():

    while True:
        gpsThread.run_gps()

def main():

    # creating threads
    t1 = threading.Thread(target = sensor1)
    t2 = threading.Thread(target=sensor2)
    t3 = threading.Thread(target=sensor3)

    # start threads
    t1.start()
    t2.start()  
    t3.start()

    print("Active Threads: {}".format(threading.active_count()))
    # wait until threads finish their job
    t1.join()
    t2.join()
    t3.join()


if __name__ == "__main__":

    main()
    
    
