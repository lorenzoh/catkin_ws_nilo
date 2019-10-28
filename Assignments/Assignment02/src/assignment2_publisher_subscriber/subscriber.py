import rospy
import autominy_msgs
from autominy_msgs.msg import Speed

def callback(data):
    rospy.loginfo("Speed: " + str(data.value))

def listener():
    rospy.init_node("listener", anonymous=True)
    rospy.Subscriber("/sensors/speed", Speed, callback)
    rospy.spin()
if __name__ == "__main__":
    listener()
