#!/usr/bin/env python

import rospy
from autominy_msgs.msg import SpeedCommand, NormalizedSteeringCommand, SteeringFeedback
from std_msgs.msg import String, Float32
import numpy as np
import math
from nav_msgs.msg import Odometry
import csv


x = 0
y = 0
car_psoitions = []

def callback_gps(data):
    global x
    global y
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    #print("X: " + str(x)[:3] + ", Y: " + str(y)[:3])


def calc_distance(p1, p2):
    if(len(p1) == 2 and len(p2) == 2):
        distance = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p2[0])**2)
        return distance
    else:
        print("Cannot calc distance between p1: " + str(p1) + " and p2: " + str(p2))


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


def callback_ticks(data):
    ticks = data
    print("ticks: " + ticks )

def get_car_position():
    car_psoitions.append((x,y))

def drive():
    # publisher for speed and sterring
    pub_speed = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=100)
    pub_steering = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=100)
    steering_cmd = NormalizedSteeringCommand()
    speed_cmd = SpeedCommand()
    rospy.sleep(1)

    #measure steering angle (1.0, 0.0, -1.0)
    steering_cmd.value = 0
    pub_steering.publish(steering_cmd)
    rospy.sleep(1)

    get_car_position()
    speed_cmd.value = 1.4
    pub_speed.publish(speed_cmd)
    rospy.sleep(2)
    speed_cmd.value = 0
    pub_speed.publish(speed_cmd)
    get_car_position()

    rospy.sleep(2)
    distance = calc_distance(car_psoitions[0],car_psoitions[1])
    print(distance)

    # # drive 3 times and measure positions
    # for i in range(3):
    #     get_car_position()
    #     speed_cmd.value = 1.4
    #     pub_speed.publish(speed_cmd)
    #     rospy.sleep(2)
    #     speed_cmd.value = 0
    #     pub_speed.publish(speed_cmd)
    #     rospy.sleep(2)
    # radius = calc_radius(car_psoitions[0], car_psoitions[1], car_psoitions[2])

    # reset steering
    steering_cmd.value = 0
    pub_steering.publish(steering_cmd)

def main():
    rospy.init_node("tick_calibration")
    rospy.Subscriber('/sensors/arduino/ticks', Odometry, callback_ticks)
    rospy.Subscriber('/communication/gps/10', Odometry, callback_gps)
    drive()
    rospy.spin()
    print("End")


if __name__ == "__main__":
    main()
