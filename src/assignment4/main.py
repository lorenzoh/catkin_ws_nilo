#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CameraInfo

fx = 383,7944641113281
fy = 383,7944641113281
cx = 322,3056945800781
cy = 241,67051696777344
k1 = 0,0
k2 = 0,0
t1 = 0,0
t2 = 0,0
k3 = 0,0


def callback(data):
    print(data.D)
    print(data.K)
    D = data.D
    #print(str(D[1]) + ":"+str(D[2]))
    K = data.K
    rospy.signal_shutdown("get data")
    #rospy.loginfo("data: " + str(data))

def listener():
    first = True
    rospy.init_node("listener", anonymous=True)
    rospy.Subscriber("/sensors/camera/infra1/camera_info", CameraInfo, callback)
    rospy.spin()
if __name__ == "__main__":
    listener()
