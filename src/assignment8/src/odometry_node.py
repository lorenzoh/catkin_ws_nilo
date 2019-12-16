
from math import cos, sin, tan
import sys
import cv2
import numpy as np
import rospy
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion


class OdometryNode:

    def __init__(self):

        self.init_odometry()
        self.hz = 100
        self.l = .27
        self.subscriber_steering = rospy.Subscriber(
            "/sensors/steering", 99999, self.set_psi, queue_size=10
        )
        self.subscriber_speed = rospy.Subscriber(
            "/sensors/speed", 99999, self.set_v, queue_size=10
        )
        """self.subscriber_filtered_map = rospy.Subscriber(
            "/sensorts/localization/filtered_map", 99999, self.init_odometry, queue_size=10
        )"""
        self.publisher = rospy.Publisher(
            "/outputs/ackermann_odometry", Odometry, queue_size=10
        )


    def run(self):
        rate = rospy.Rate(self.hz)
        while not rospy.is_shutdown():
            self.update_odometry()
            self.publish_odometry()


    def init_odometry(self, data):
        self.x = 0
        self.y = 0
        self.theta = 0

    def set_psi(self, data):
        self.psi = data

    def set_v(self, data):
        self.v = data

    def publish_odometry(self):
        print(self.x, self.y, self.theta)
        
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        odom = Odometry()
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose = Pose(Point(self.x, self.y, 0), Quaternion(*odom_quat))

        self.publisher.publish(odom)

    def update_odometry(self):
        t_delta = 0.1

        x_dot = self.v * cos(self.theta)
        y_dot = self.v * sin(self.theta)
        theta_dot = (self.v / self.l) * tan(self.psi)

        self.x = self.x + t_delta * x_dot
        self.y = self.y + t_delta * y_dot
        self.theta = self.theta + t_delta * theta_dot


def main():
    odometry_node = OdometryNode()
    rospy.init_node("ackermann_odometry", anonymous=True)
    odometry_node.run()


