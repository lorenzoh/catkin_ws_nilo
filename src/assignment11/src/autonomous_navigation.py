#!/usr/bin/env python

from .steering_pid import SteeringPID

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
import random
import math

import rospy
#from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker
from autominy_msgs.msg import NormalizedSteeringCommand, NormalizedSpeedCommand, SteeringCommand
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

current_spline_x = None
current_spline_y = None

car_pos = None

CAR_ID = "15"

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

def calc_closest_point(lane,point):
    index = np.linspace(0,len(lane),6,endpoint=False,dtype=int)
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
    if(x == None and y == None):
        x = random.randint(0, 6)
        y = random.randint(0, 4)
        p = Point(None,x, y)
    else:
        p = car_pos
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

# def drive():
#     point_to_drive = get_car_pos(2.4,0.7)
#     for i in range(50):
#         if(i==20):
#             change_line()
#         car_pos = get_car_pos(point_to_drive.x,point_to_drive.y)
#         point_on_spline = calc_closest_point(current_lane, car_pos)
#         plot_point([car_pos])
#         point_to_drive = add_lookahead(point_on_spline,0.5)

def change_line():
    global current_lane, index_spline, current_spline_x, current_spline_y
    index_spline = (index_spline + 1) % 2
    current_lane = select_points(lanes[index_spline])
    current_spline_x = splines_x[index_spline]
    current_spline_y = splines_y[index_spline]

# def drive_to_next_point():
#     global last_error
#     diff = target_radiant - car_radiant
#     # normalize steering angles (keep between -pi and pi)
#     error = math.atan2(math.sin(diff), math.cos(diff))
#     derivative_error = (error - last_error) / (0.1)
#
#     pid_output = Kp * error + Kd * derivative_error
#     publisher_steering.publish(NormalizedSteeringCommand(value=pid_output))
#
#     # for next step's derivative
#     last_error = e

def callback_gps(data):

    #get target from car pos
    p = data.pose.pose.position
    car_pos_tmp = Point(None, p.x, p.y)
    point_on_spline = calc_closest_point(current_lane, car_pos_tmp)
    point_to_drive = add_lookahead(point_on_spline, 1)
    target_vector = (point_to_drive.x - car_pos_tmp.x, point_to_drive.y - car_pos_tmp.y)
    target_radiant = math.atan2(target_vector[1], target_vector[0])

    #pub target and car pos
    target_msg = SteeringCommand(value=target_radiant)
    pub_target.pub(target_msg)
    pub_car_pos.pub(data)

    # o = data.pose.pose.orientation
    # orientation = [o.x, o.y, o.z, o.w]
    # _, _, car_radiant = euler_from_quaternion(orientation)

# def main():
#     rospy.sleep(1)
#     rate = rospy.Rate(hz)
#     pub_speed.publish(NormalizedSpeedCommand(value=0.12))
#     change_lane_counter = 0
#     while not rospy.is_shutdown():
#         # if(change_lane_counter > 100):
#         #     change_lane_counter = 0
#         #     print("change lane")
#         #     change_line()
#         drive_to_next_point()
#         change_lane_counter = change_lane_counter + 1
#         rate.sleep()

if __name__ == "__main__":
    rospy.init_node("spline")

    steering_pid = SteeringPID()

    #init
    index_spline = -1
    lanes = [np.load("lane1.npy"), np.load("lane2.npy")]
    splines_x,splines_y = calc_spline_interpolations(lanes)
    change_line()
    change_line()

    pub_car_pos = rospy.Publisher('/assignment11/car_pos', Odometry, queue_size=1)
    pub_target = rospy.Publisher('/assignment11/target', SteeringCommand, queue_size=1)

    pub_speed = rospy.Publisher('/actuators/speed_normalized', NormalizedSpeedCommand, queue_size=10)

    subscriber_odometry = rospy.Subscriber("/communication/gps/" + CAR_ID, Odometry, callback_gps, queue_size=10)
    rospy.sleep(2)

    #main()
    rospy.spin()


def marker():
    pub_marker = rospy.Publisher("/spline/close", Marker, queue_size=10)

    marker = Marker()
    marker.header.frame_id = "map"

    marker.ns = "road"
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD

    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.5
    marker.pose.position.x = 1
    marker.pose.position.y = 1
    marker.pose.position.z = 0
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0
    marker.color.b = 1
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.color.r = 1.0
    marker.color.g = 0
    marker.color.b = 1
    pub_marker.publish(marker)