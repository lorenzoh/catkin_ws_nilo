import json
import rospy
from sensor_msgs.msg import CameraInfo


def callback(data):

    params = {
        "fx": data.K[0],
        "fy": data.K[4],
        "cx": data.K[2],
        "cy": data.K[5],
        "k1": data.D[0],
        "k2": data.D[1],
        "t1": data.D[2],
        "t2": data.D[3],
        "k3": data.D[4],
    }

    json.dump(params, open("cameraparams.json", "w"))


def main():
    rospy.init_node("export_camera", anonymous=True)
    rospy.Subscriber(
        "/sensors/camera/infra1/camera_info", CameraInfo, callback)
    rospy.spin()



if __name__ == "__main__":
    main()