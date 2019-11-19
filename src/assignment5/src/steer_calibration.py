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
sensor_steering_angle = 0


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
    steering_angle = np.arctan(0.2/r)
    return steering_angle

def calc_radiant(angle):
    return math.radians(angle)

def callback_gps(data):
    global x
    x = data.pose.pose.position.x
    global y
    y = data.pose.pose.position.y
    #print("X: " + str(x)[:3] + ", Y: " + str(y)[:3])

def callback_steering(data):
    steering_anlge = data.value
    steering_radiant = calc_radiant(steering_anlge)
    global sensor_steering_angle
    sensor_steering_angle = steering_anlge

car_psoitions = []
def get_car_position():
    car_psoitions.append((x,y))

def measure():
    # publisher for speed and sterring
    pub_speed = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=100)
    pub_steering = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=100)
    steering_cmd = NormalizedSteeringCommand()
    speed_cmd = SpeedCommand()
    table = []
    real_angles = []
    calculated_angles = []
    current_commands = []

    rospy.sleep(2)
    #measure steering angle (1.0, 0.0, -1.0)
    current_sterring_cmd = 0
    steering_cmd.value = current_sterring_cmd
    pub_steering.publish(steering_cmd)
    rospy.sleep(1)
    real_angles.append(sensor_steering_angle)

    # drive 3 times and measure positions
    for i in range(3):
        get_car_position()
        speed_cmd.value = 1.4
        pub_speed.publish(speed_cmd)
        rospy.sleep(2)
        speed_cmd.value = 0
        pub_speed.publish(speed_cmd)
        rospy.sleep(2)


    radius = calc_radius(car_psoitions[0], car_psoitions[1], car_psoitions[2])
    calculated_angle = calc_steering_angle(radius)
    current_commands.append(current_sterring_cmd)
    calculated_angles.append(calculated_angle)


    table =  [current_commands, calculated_angles,real_angles]
    print(table)

    # reset steering
    steering_cmd.value = 0
    pub_steering.publish(steering_cmd)

    # table = [[1, 2, 3], [4, 5, 6]]
    # write csv
    with open('measurements.csv', 'w') as csvfile:
        writer = csv.writer(csvfile)
        [writer.writerow(r) for r in table]

def main():
    rospy.init_node("steer_calibration")
    #subscriber for gps and steering
    rospy.Subscriber('/communication/gps/10', Odometry, callback_gps)
    rospy.Subscriber('/sensors/arduino/steering_angle',SteeringFeedback , callback_steering)
    measure()
    rospy.spin()
    print("End")


if __name__ == "__main__":
    p1 = np.array([1, 1])
    p2 = np.array([2, 4])
    p3 = np.array([5, 3])
    calc_radius(p1,p2,p3)
    main()
