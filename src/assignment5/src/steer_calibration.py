#!/usr/bin/env python

import rospy
from autominy_msgs.msg import SpeedCommand, NormalizedSteeringCommand
from std_msgs.msg import String, Float32
import numpy as np
import math
from nav_msgs.msg import Odometry

x = 0
y = 0

def calc_radius(p1,p2,p3):
    q = (p1[0]**2)/2 + (p1[1]**2)/2 - (p2[0]**2)/2 - (p2[1]**2)/2
    p = (p3[0]**2)/2 + (p3[1]**2)/2 - (p2[0]**2)/2 - (p2[1]**2)/2
    #linear system
    s = np.array([[p1[0]-p2[0],p1[1]-p2[1]], [p3[0]-p2[0],p3[1]-p2[1]]])
    t = np.array([q,p])
    try:
        x = np.linalg.solve(s,t)
        r = math.sqrt((p1[0]-x[0])**2+(p1[1]-x[1])**2)
        print(r)
    except np.linalg.LinAlgError:
        raise np.linalg.LinAlgError
    return r

def calc_steering_angle(r):
    steering_angle = 0 #Todo

    return steering_angle

def callback_gps(data):
    global x
    x = data.pose.pose.position.x
    global y
    y = data.pose.pose.position.y
    print("X: " + str(x)[:3] + ", Y: " + str(y)[:3])

def callback_steering(data):
    print(data) #Todo
    #return data.angle

car_psoitions = []
def get_car_position():
    car_psoitions.append((x,y))
    print(car_psoitions)

def main():
    rospy.init_node("steer_calibration")
    #subscriber for gps and steering
    rospy.Subscriber('/communication/gps/10', Odometry, callback_gps)
    rospy.Subscriber('/sensors/arduino/steering_angle', Odometry, callback_steering)

    #publisher for speed and sterring
    pub_speed = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=100)
    pub_steering = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=100)
    steering_cmd = NormalizedSteeringCommand()
    speed_cmd = SpeedCommand()

    rospy.sleep(2)
    steering_cmd.value = 1
    pub_steering.publish(steering_cmd)

    #drive 3 times and measure positions
    for i in range(3):
        get_car_position()
        speed_cmd.value = 1.4
        pub_speed.publish(speed_cmd)
        rospy.sleep(2)
        speed_cmd.value = 0
        pub_speed.publish(speed_cmd)
        rospy.sleep(2)

    radius = calc_radius(car_psoitions[0],car_psoitions[1],car_psoitions[2])
    calc_steering_angle(radius)

    #reset steering
    steering_cmd.value = 0
    pub_steering.publish(steering_cmd)

    rospy.spin()
    print("End")


if __name__ == "__main__":
    p1 = np.array([1, 1])
    p2 = np.array([2, 4])
    p3 = np.array([5, 3])
    calc_radius(p1,p2,p3)
    main()
