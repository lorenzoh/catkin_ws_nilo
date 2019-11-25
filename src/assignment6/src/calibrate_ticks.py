
import csv
import math
import sys

import numpy as np
import rospy
from autominy_msgs.msg import (NormalizedSteeringCommand, SpeedCommand,
                               SteeringFeedback, Tick)
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String


class TickCalibrator:

    def __init__(self):
        self.ticks = 0
        self.positions = []
        self.x = None
        self.y = None
        self.subscriber_ticks = rospy.Subscriber(
            "/sensors/arduino/ticks", Tick, self.callback_ticks)
        self.subscriber_gps = rospy.Subscriber(
            "/communnication/gps/10", Tick, self.callback_gps)
        self.publisher_speed = rospy.Publisher(
            "/actuators/speed", SpeedCommand, queue_size=100)
        self.publisher_steering = rospy.Publisher(
            "/actuators/steering_normalized", SpeedCommand, queue_size=100)

    def run(self, speed, angle, seconds):
        rospy.sleep(.1)
        assert self.x
        assert self.y

        # init metrics
        self.ticks = 0
        self.positions = []


        # Run car with `speed` at `angle` for `seconds`
        self.publisher_steering.publish(
            NormalizedSteeringCommand(value=angle))
        self.publisher_speed.publish(
            SpeedCommand(value=speed))
        rospy.sleep(seconds)
        self.publisher_speed.publish(
            SpeedCommand(value=0.))
        # TODO count ticks and calculate distance between positions
        ticks = self.ticks
        positions = self.positions[:]
        distance = calculate_distance(positions)

    def reset_ticks(self):
        self.ticks = 0

    def callback_ticks(self, data):
        self.ticks += data.value
        print("Ticks: ", data)
        
    def callback_gps(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.positions.append((self.x, self.y))
        print("GPS: ", data)


def calculate_distance(positions):
    distance = 0
    for ((x1, y1), (x2, y2)) in zip(positions, positions[1:]):
        distance += math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    return distance


def main(speed, angle, seconds):
    rospy.init_node("tick_calibration")
    calibrator = TickCalibrator()
    calibrator.run(speed, angle, seconds)
    rospy.spin()

if __name__ == "__main__":
    main(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))
