import sys
import rospy
from autominy_msgs.msg import NormalizedSteeringCommand, SpeedCommand


def publisher():
    pub1 = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=10)
    pub2 = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=10)
    rospy.init_node("assignment2publisher", anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        steering_msg = NormalizedSteeringCommand(value=1.)
        speed_msg = SpeedCommand(value=float(sys.argv[1]))
       
        pub1.publish(steering_msg)
        pub2.publish(speed_msg)
        print("published")
        rate.sleep()


if __name__ == "__main__":
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
