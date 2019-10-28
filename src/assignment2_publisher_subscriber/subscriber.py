#!/usr/bin/env python

import rospy
#from std_msgs.msg import String
from autominy_msgs import Speed

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'Speed: %s', data.data)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'speed_node' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('speed_node', anonymous=True)

    rospy.Subscriber('/sensors/speed', Speed, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
