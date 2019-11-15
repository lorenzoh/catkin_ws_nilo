#!/usr/bin/env python

import rospy
from autominy_msgs.msg import SpeedCommand, NormalizedSteeringCommand
from std_msgs.msg import String, Float32



def main():
    rospy.init_node("steer_calibration")
    pub_speed = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=100)
    pub_steering = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=100)
    steering_cmd = NormalizedSteeringCommand()
    speed_cmd = SpeedCommand()

    rospy.sleep(2)
    steering_cmd.value = 1
    pub_steering.publish(steering_cmd)

    for i in range(3):
        speed_cmd.value = 1.4
        pub_speed.publish(speed_cmd)
        rospy.sleep(2)
        speed_cmd.value = 0
        pub_speed.publish(speed_cmd)
        rospy.sleep(2)


    #rospy.spin()
    print("End")


if __name__ == "__main__":
    main()
