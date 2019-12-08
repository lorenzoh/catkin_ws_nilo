#!/usr/bin/env python

import time
import csv
import math
import sys
import numpy as np
import rospy
from autominy_msgs.msg import NormalizedSteeringCommand, SpeedCommand, SteeringFeedback , Tick, SteeringAngle, Speed
#from autominy_msgs.mgs import SteeringAngle
# from nav_msgs.msg import Odometry
# from std_msgs.msg import Float32, String

class Timer:
    def __index__(self):
        self.start_time = time.time()
    def start_timer(self):
        self.start_time = time.time()
    def stop_timer(self):
        return time.time() - self.start_time

x_pos = 0
y_pos = 0
steering = 0
last_steering = 0
timer = Timer()

def callback_speed(data):
    global last_steering, timer
    speed = data.value
    if(last_steering != None):
        delta_time = timer.stop_timer()
        timer.start_timer()
        print(delta_time)
        calc_next_pos(speed,last_steering,delta_time)
    #print('speed: '+ str(speed))

def callback_steering(data):
    global last_steering
    steering = data.value
    last_steering = steering
    #print('steering angle: '+ str(steering))

def callback_gps(self, data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

def calc_next_pos(speed,steering_angle,delta_time):
    global last_steering, x_pos, y_pos, steering
    last_steering = None

    print("x_pos: " + str(x_pos)[:4] + ", y_pos: " + str(y_pos)[:4])

    current_steering = (speed / 0.27) * math.tan(steering_angle)
    current_x = speed * math.cos(current_steering)
    current_y = speed * math.sin(current_steering)

    #calc new pos
    steering = steering + delta_time * current_steering
    x_pos = x_pos + delta_time * current_x
    y_pos = y_pos + delta_time * current_y

def main():
    rospy.init_node("odometry_node")
    timer.start_timer()
    subscriber_steering = rospy.Subscriber(
        "/sensors/steering", SteeringAngle, callback_steering)
    subscriber_speed = rospy.Subscriber(
        "/sensors/speed", Speed, callback_speed)

    # subscriber_gps = rospy.Subscriber(
    #     "/communication/gps/10", Odometry, callback_gps)
    #
    # publisher_speed = rospy.Publisher(
    #     "/actuators/speed", SpeedCommand, queue_size=100)
    #
    # rospy.sleep(1)
    # publisher_speed.publish(
    #     SpeedCommand(value=0.))

    rospy.spin()

if __name__ == "__main__":
    main()