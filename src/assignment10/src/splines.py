#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
import random
import math

# import rospy
# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import Point, PointStamped

def select_points(lane):
    last_point = np.array([lane[-1]])
    lane = lane[0::10]
    lane = np.append(lane, last_point,axis=0)

    # plt.plot(lane[:, 1], lane[:, 2], 'o')
    # plt.ylabel('circle')
    # plt.show()

    return lane

def interpolate(lane):
    spl_x = CubicSpline(lane[:, 0], lane[:, 1])
    spl_y = CubicSpline(lane[:, 0], lane[:, 2])

    xs = np.linspace(0, 12.76, 1277)
    interpolate_x = spl_x(xs)
    interpolate_y = spl_y(xs)

    #select random point (instead of click)
    rand_x = random.randint(0, 6)
    rand_y = random.randint(0, 4)
    point = (rand_x, rand_y)
    print(point)

    closest_point = calc_closest_point(lane1, point)
    plt.plot(closest_point[1],closest_point[2],'bo')
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
    smallest_distance = float('inf')
    smallest_i = None
    for i in range(3):
        distance = calc_distance(point,get_point(lane,index[i]))
        if(distance < smallest_distance):
            smallest_distance = distance
            smallest_i = i

    print("smallest: "+str(index[smallest_i])+ " down: " +str(index[smallest_i-1])+ " up: "+str(index[(smallest_i+1)%3]))
    return search(index[smallest_i-1],index[(smallest_i+1)%3],lane,point)

def search(down,up,lane,point):

    #check if finished
    if(down > up):
        if (abs(up + len(lane) - down) < 3):
            return lane[down]
    if(abs(up-down) < 3):
        return lane[down]

    # calc middle point
    if(down > up):
        middle_point = (down+(len(lane)-down+up)//2) % len(lane)
    else:
        middle_point = down + (up - down)//2

    distance_down = calc_distance(get_point(lane, down),point)
    distance_up = calc_distance(get_point(lane, up),point)
    if(distance_down < distance_up):
        return search(down,middle_point,lane,point)
    else:
        return search(middle_point,up,lane,point)

if __name__ == "__main__":
    # rospy.init_node("spline")
    # rospy.Subscriber('/clicked_point', PointStamped, callback_click_point,queue_size=1)

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

    # rospy.spin()

