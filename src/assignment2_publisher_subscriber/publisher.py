#!/usr/bin/env python

import rospy
from autominy_msgs.msg import NormalizedSteeringCommand, SpeedCommand
#from std_msgs.msg import String

def main():
    rospy.init_node('Speed_Steering_A2', anonymous=True)
    pub_steering = rospy.Publisher('/actuators/steering normalized', NormalizedSteeringCommand, queue_size=1, tcp_nodelay=True)
    pub_speed = rospy.Publisher('/actuators/speed', SpeedCommand, queue_size=1, tcp_nodelay=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        steering_msg = NormalizedSteeringCommand()
        steering_msg.value = 1
        steering_msg.header.frame_id = "base_link"
        steering_msg.header.stamp = rospy.Time.now()
        pub_steering.publish(steering_msg)

        speed_msg = SpeedCommand()
        speed_msg.value = 0.3
        speed_msg.header.frame_id = "base_link"
        speed_msg.header.stamp = rospy.Time.now()
        pub_speed.pub_speed.publish(speed_msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

