#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
import os
import random
import math
import sys

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped

# from autominy_msgs.msg import NormalizedSteeringCommand, SpeedCommand
# from nav_msgs.msg import Odometry
# from tf.transformations import euler_from_quaternion

def add_diff(lane):
    for column in (1,2):
        diffs = np.array([abs(lane[0, [column]] - lane[-1, [column]])])
        for i in range(1, len(lane)):
            current = lane[i, [column]]
            pref = lane[i - 1, [column]]
            diff = np.array([abs(current - pref)])
            diffs = np.append(diffs, diff, axis=0)
        lane = np.append(lane, diffs, axis=1)
    return lane

def select_points(lane):
    last_point = np.array([lane[-1]])
    lane = lane[0::50]
    lane = np.append(lane, last_point,axis=0)
    #lane = add_diff(lane)

    plt.plot(lane[:, 1], lane[:, 2], 'o')
    plt.ylabel('circle')
    plt.show()

    # plt.plot(lane[:,0],lane[:,1],'o')
    # plt.ylabel('x values')
    # plt.show()
    # plt.plot(lane[:,0],lane[:,2],'o')
    # plt.ylabel('y values')
    # plt.show()
    # plt.plot(lane[:,0],lane[:,3],'o')
    # plt.ylabel('diff_x')
    # plt.show()
    # plt.plot(lane[:,0],lane[:,4],'o')
    # plt.ylabel('diffs_y')
    # plt.show()

    return lane

def interpolate(lane):
    spl_x = CubicSpline(lane[:, 0], lane[:, 1])
    spl_y = CubicSpline(lane[:, 0], lane[:, 2])

    xs = np.linspace(0, 12.76, 1277)
    interpolate_x = spl_x(xs)
    interpolate_y = spl_y(xs)

    publisher_spline = rospy.Publisher("LINE_STRIP", Marker, queue_size=10)

    # rate = rospy.Rate(1)
    # while not rospy.is_shutdown():
    #
    #     msg = Marker(type=Marker.LINE_STRIP, action=Marker.ADD)
    #     msg.header.frame_id = "map"
    #     msg.scale.x = 0.01
    #     msg.scale.y = 0.01
    #     msg.color.r = 1.0
    #     msg.color.a = 1.0
    #
    #     for i in range(len(lane)):
    #         msg.points.append(Point(interpolate_x[i], interpolate_y[i], 0.0))
    #     publisher_spline.publish(msg)
    #
    #     rate.sleep()

    #select random point (instead of click)
    rand_x = random.randint(0, 6)
    rand_y = random.randint(0, 4)
    point = (rand_x, rand_y)

    plt.plot(calc_closest_point(lane1, point)[1],calc_closest_point(lane1, point)[2],'bo')
    plt.plot(point[0],point[1],'r+')
    plt.plot(interpolate_x, interpolate_y)
    plt.show()

def callback_click_point(data):
    print("clicked!")
    print(data.data)

def calc_distance(point1, point2):
    distance_vector = (point1[0]-point2[0],point1[1]-point2[1])
    distance = math.sqrt(distance_vector[0]**2+distance_vector[1]**2)
    return distance

def get_point(lane,i):
    return (lane[i,1],lane[i,2])

def calc_closest_point(lane,point):
    index = np.linspace(0,len(lane),3,endpoint=False,dtype=int)
    smallest_distance = float('inf') #math.inf
    smallest_i = None
    for i in range(3):
        distance = calc_distance(point,get_point(lane,index[i]))
        if(distance < smallest_distance):
            smallest_distance = distance
            smallest_i = i

    return search(index[smallest_i-1],index[(smallest_i+1)%3],lane,point)

def search(down,up,lane,point):
    if(down > up):
        tmp = down
        down = up
        up = tmp

    if(up-down < 2):
        return lane[down]
    new_i = down + (up - down)//2

    print("index " + str(new_i) + ": distance = " +str(calc_distance(get_point(lane, new_i),point)))

    distance_down = calc_distance(get_point(lane, down),point)
    distance_up = calc_distance(get_point(lane, up),point)
    if(distance_down < distance_up):
        return search(down,new_i,lane,point)
    else:
        return search(new_i,up,lane,point)

if __name__ == "__main__":
    print("START")
    rospy.init_node("spline")
    rospy.Subscriber('/clicked_point', PointStamped, callback_click_point,queue_size=1)

    lane1 = np.load("lane1.npy")
    lane2 = np.load("lane2.npy")

    # interpolate circle
    lane1 = select_points(lane1)
    interpolate(lane1)

    # closest point
    rand_x = random.randint(0,6)
    rand_y = random.randint(0,4)
    point = (rand_x,rand_y)
    print("closest point for " + str(point) + "is "+ str(calc_closest_point(lane1,point)))

    rospy.spin()

