#!/usr/bin/env python

import rospy
import sys
import math

from autominy_msgs.msg import NormalizedSteeringCommand, SpeedCommand
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

CAR_ID = "11"

class PDController:
    # """
    # PD-controller that tries to achieve constant yaw `r`.
    # Hence r(t) = r is constant for all t
    # y(t) is the actual yaw obtained from the odometry information
    # e(t) = r(t) - y(t)
    #
    # Resulting in:
    # u(t) = Kp * e(t)  +  Kd * de(t)  where
    #     de(t) = e(t-1) - e(t) / (1/frequency)
    # """

    def __init__(self, r, Kp, Kd):
        self.hz = 100
        self.Kp = Kp
        self.Kd = Kd

        # r(t) is constant in this case as the desired yaw doesn't change
        self.r = r

        # initialize running values
        self.yt = 0
        self.e = 0
        self.eprev = 0

        # repeatedly update `self.yt` (y(t)) from odometry information
        self.subscriber_odometry = rospy.Subscriber(
            "/communication/gps/" + CAR_ID, Odometry, self.set_yt, queue_size=10
        )

        # publisher for steering output
        self.publisher_steering = rospy.Publisher(
            "/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=10
        )


    def run(self):
        # Run PD-controller at constant frequency to make calculating
        # derivative easier
        rate = rospy.Rate(self.hz)
        while not rospy.is_shutdown():
            self.pd_step()
            rate.sleep()

    def pd_step(self):
        
        # calculate e(t) = r(t) - y(t)
        self.e = self.r - self.yt

        print("Error: " +str(self.e))

        # approximate derivative with $(e(t) - e(t-1)) / d$  where
        # d is time passed, i.e. `1/self.hz`
        self.de = (self.e - self.eprev) / (1. / self.hz)

        # u(t) is the steering angle output
        u = (self.Kp * self.e + self.Kd * self.de) / math.pi

        print(u)

        # publish steering command
        self.steer(u)

        # for next step's derivative
        self.eprev = self.e

    def steer(self, u):
        self.publisher_steering.publish(
            NormalizedSteeringCommand(value=u)
        )
    
    def set_yt(self, data):
        """Continually updates y(t) with odometry data"""
        o = data.pose.pose.orientation
        orientation = [o.x, o.y, o.z, o.w]
        _, _, yaw = euler_from_quaternion(orientation)
        self.yt = yaw


if __name__ == "__main__":

    print("START")

    rospy.init_node("pd_controller")

    #drive
    pub_speed = rospy.Publisher('/actuators/speed', SpeedCommand, queue_size=1, tcp_nodelay=True)
    rospy.sleep(2)
    pub_speed.publish(SpeedCommand(value=0.15))#.15))


    r = float(sys.argv[1])
    Kp = float(sys.argv[2])
    Kd = float(sys.argv[3])

    pdcontroller = PDController(r, Kp, Kd)
    pdcontroller.run()


    rospy.spin()