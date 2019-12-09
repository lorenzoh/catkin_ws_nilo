#!/usr/bin/env python

import time
import csv
import math
import sys
import numpy as np
import tf

import rospy
from autominy_msgs.msg import NormalizedSteeringCommand, SpeedCommand, SteeringFeedback , Tick, SteeringAngle, Speed
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion, Point, Twist, Vector3

#from autominy_msgs.mgs import SteeringAngle
# from nav_msgs.msg import Odometry
# from std_msgs.msg import Float32, String

class Timer:
    def __index__(self):
        self.start_time = rospy.Time.now()
    def start_timer(self):
        self.start_time = rospy.Time.now()
    def stop_timer(self):
        return (rospy.Time.now() - self.start_time).to_sec()

x_pos = 0.0
y_pos = 0.0
steering = 0.0
last_steering = 0
timer = Timer()

publisher_calc_odom = None
odom_broadcaster = None

def callback_speed(data):
    global last_steering, timer
    speed = data.value
    if(last_steering != None):
        delta_time = timer.stop_timer()
        timer.start_timer()
        #print(delta_time)
        calc_next_pos(speed,last_steering,delta_time)
    # print('speed: '+ str(speed))

def callback_steering(data):
    global last_steering
    steering = data.value
    last_steering = steering
    # print('steering angle: '+ str(steering))

def callback_gps_cam(data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    # print("cam_x: "+ str(x) + ", y_cam: " + str(y))

def calc_next_pos(speed,steering_angle,delta_time):
    global last_steering, x_pos, y_pos, steering
    last_steering = None

    print("x_pos: " + str(x_pos)[:4] + ", y_pos: " + str(y_pos)[:4])

    current_steering = (speed / 0.27) * math.tan(steering_angle)
    current_x = speed * math.cos(current_steering)
    current_y = speed * math.sin(current_steering)

    # calc new pos
    steering = steering + delta_time * current_steering
    x_pos = x_pos + delta_time * current_x
    y_pos = y_pos + delta_time * current_y

    # pub calc odom
    current_time = rospy.Time.now()

    odom = Odometry()
    odom.header.frame_id = "map"
    odom.header.stamp = current_time

    # set the position
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, steering)
    odom.pose.pose = Pose(Point(x_pos, y_pos, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_link"
    #odom.twist.twist = Twist(Vector3(speed, vy, 0), Vector3(0, 0, vth)) TODO
    odom.twist.twist = Twist(Vector3(speed, 0, 0), Vector3(0, 0, current_steering))

    # publish the message
    publisher_calc_odom.publish(odom)


def main():
    rospy.init_node("odometry_node")
    timer.start_timer()
    subscriber_steering = rospy.Subscriber("/sensors/steering", SteeringAngle, callback_steering)
    subscriber_speed = rospy.Subscriber("/sensors/speed", Speed, callback_speed)
    subscriber_gps_cam = rospy.Subscriber("/communication/gps/16", Odometry, callback_gps_cam)
    global publisher_calc_odom
    global odom_broadcaster
    publisher_calc_odom = rospy.Publisher("calc_odom", Odometry, queue_size=100)
    odom_broadcaster = tf.TransformBroadcaster()

    # publisher_speed = rospy.Publisher(
    #     "/actuators/speed", SpeedCommand, queue_size=100)
    #
    # rospy.sleep(1)
    # publisher_speed.publish(
    #     SpeedCommand(value=0.))

    rospy.spin()

if __name__ == "__main__":
    main()