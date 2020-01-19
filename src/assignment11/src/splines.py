#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
import random
import math

# import rospy
# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import Point, PointStamped

current_spline_x = None
current_spline_y = None

class Point:
    def __init__(self, arc, x, y):
        self.arc = arc
        self.x = x
        self.y = y
    def __str__(self):
        return "("+str(self.x)[:3]+","+str(self.y)[:3]+")"

def select_points(lane):
    last_point = np.array([lane[-1]])
    lane = lane[0::10]
    lane = np.append(lane, last_point,axis=0)
    return lane

def calc_spline_interpolations(lanes):
    splines_x = [CubicSpline(lanes[0][:, 0], lanes[0][:, 1]),CubicSpline(lanes[1][:, 0], lanes[1][:, 1])]
    splines_y = [CubicSpline(lanes[0][:, 0], lanes[0][:, 2]),CubicSpline(lanes[1][:, 0], lanes[1][:, 2])]
    return splines_x,splines_y

def calc_distance(point1, point2):
    distance_vector = (point1.x-point2.x,point1.y-point2.y)
    distance = math.sqrt(distance_vector[0]**2+distance_vector[1]**2)
    return distance

def get_point(lane,i):
    return Point(lane[i,0],lane[i,1],lane[i,2])

def plot_first_points(index):
    plot_point( [Point(None, current_lane[index[0], 1], current_lane[index[0], 2]),
                 Point(None, current_lane[index[1], 1], current_lane[index[1], 2]),
                 Point(None, current_lane[index[2], 1], current_lane[index[2], 2]),
                 Point(None, current_lane[index[3], 1], current_lane[index[3], 2]),
                 Point(None, current_lane[index[4], 1], current_lane[index[4], 2]),
                 Point(None, current_lane[index[5], 1], current_lane[index[5], 2]),
                 ] )

def calc_closest_point(lane,point):
    index = np.linspace(0,len(lane),6,endpoint=False,dtype=int)
    #plot_first_points(index)
    smallest_distance = float('inf')
    smallest_i = None
    for i in range(len(index)):
        distance = calc_distance(point,get_point(lane,index[i]))
        if(distance < smallest_distance):
            smallest_distance = distance
            smallest_i = i
    return search(index[smallest_i-1],index[(smallest_i+1)%3],lane,point)

def search(down,up,lane,point):
    #check if finished
    if(down > up):
        if (abs(up + len(lane) - down) < 2):
            return get_point_from_spline(lane[down,0])
    if(abs(up-down) < 2):
        return get_point_from_spline(lane[down,0])

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

def get_point_from_spline(arc):
    p = Point(arc,current_spline_x(arc).item(),current_spline_y(arc).item())
    return p

def get_car_pos(x=None,y=None):

    #TODO implement
    if(x == None and y == None):
        x = random.randint(0, 6)
        y = random.randint(0, 4)
    p = Point(None,x, y)
    return  p

def plot_point(points_array):

    max1 = lanes[0].max()
    max2 = lanes[1].max()

    xs1 = np.linspace(0, max1, int(max1*100)+1)
    xs2 = np.linspace(0, max2, int(max2*100)+1)
    lane1_x = splines_x[0](xs1)
    lane1_y = splines_y[0](xs1)
    lane2_x = splines_x[1](xs2)
    lane2_y = splines_y[1](xs2)

    for p in points_array:
        plt.plot(p.x,p.y,'bo')
    plt.plot(lane1_x, lane1_y)
    plt.plot(lane2_x, lane2_y)
    plt.plot()

    plt.show()

def add_lookahead(point, meter):
    new_point = get_point_from_spline((point.arc + meter)%current_lane.max())
    return new_point

def drive():
    point_to_drive = get_car_pos(2.4,0.7)
    for i in range(50):
        if(i==20):
            change_line()
        car_pos = get_car_pos(point_to_drive.x,point_to_drive.y)
        point_on_spline = calc_closest_point(current_lane, car_pos)
        plot_point([car_pos])
        point_to_drive = add_lookahead(point_on_spline,0.5)

def change_line():
    global current_lane, current_i, current_spline_x, current_spline_y
    current_i = (current_i + 1) % 2
    current_lane = select_points(lanes[current_i])
    current_spline_x = splines_x[current_i]
    current_spline_y = splines_y[current_i]

if __name__ == "__main__":
    # rospy.init_node("spline")
    current_i = -1
    lanes = [np.load("lane1.npy"), np.load("lane2.npy")]
    splines_x,splines_y = calc_spline_interpolations(lanes)

    change_line()

    #drive()

    car_pos = get_car_pos()
    point_on_spline = calc_closest_point(current_lane, car_pos)
    point_to_drive = add_lookahead(point_on_spline,0.5)

    plot_point([point_on_spline,car_pos,point_to_drive])

    # rospy.spin()

