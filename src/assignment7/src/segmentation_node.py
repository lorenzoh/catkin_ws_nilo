import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image

MASK_REGIONS = [
    [(0, 150), (0, 800)],
    [(300, 800), (250, 550)]
]


class LaneSegmentationNode:
    def __init__(self):
        self.subscriber_infrared = rospy.Subscriber(
            "/sensors/camera/infra1/image_rect_raw", Image, self.callback, queue_size=10
        )
        self.publisher_mask = rospy.Publisher(
            "/outputs/infra_mask", Image, queue_size=10
        )
        self.bridge = CvBridge()

    def callback(self, data):
        image = self.bridge.imgmsg_to_cv2(data, "8UC1")
        mask = process_image(image)
        self.publisher_mask.publish(self.bridge.cv2_to_imgmsg(mask, "8UC1"))


def process_image(image):
    THRESHOLD_VALUE = 200  # TODO: tweak value

    _, mask = cv2.threshold(image, THRESHOLD_VALUE, 255, cv2.THRESH_BINARY)

    for (y1, y2), (x1, x2) in MASK_REGIONS:
        mask[y1:y2, x1:x2] = 0

    return mask


def main():
    rospy.init_node("lane_segmentation", anonymous=True)
    _ = LaneSegmentationNode()
    rospy.spin()


if __name__ == "__main__":
    main()
