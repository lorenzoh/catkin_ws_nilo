import sys
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image

from ransac import ransac, LineModel

ROIS = [
    [(10, 500), (10, 300)],
    [(10, 500), (300, 380)],
    [(10, 500), (380, 640)],
]


class RANSACNode:
    def __init__(self, n_iterations, threshold, percentage):
        self.n_iterations = n_iterations
        self.threshold = threshold
        self.percentage = percentage
        self.subscriber_mask = rospy.Subscriber(
            "/outputs/infra_mask", Image, self.callback, queue_size=10
        )
        self.publisher_image = rospy.Publisher(
            "/outputs/line_image", Image, queue_size=10
        )
        self.bridge = CvBridge()

    def callback(self, data):
        mask = self.bridge.imgmsg_to_cv2(data, "8UC1")
        mask = process_image(mask, self.n_iterations, self.threshold, self.percentage)
        self.publisher_image.publish(self.bridge.cv2_to_imgmsg(mask, "8UC1"))


def process_image(mask, n_iterations, threshold, percentage):
    outmask = np.zeros_like(mask)

    candidates = np.argwhere(mask)
    for (y1, y2), (x1, x2) in ROIS:
        # find line parameters with RANSAC
        candidates_roi = np.array([(y, x) for (y, x) in candidates if x1 <= x <= x2 and y1 <= y <= y2])
        m, b = ransac(candidates_roi, LineModel, n_iterations, threshold, percentage)

        # draw line
        x1, x2 = np.min(candidates_roi[:, 1]), np.max(candidates_roi[:, 1])
        y1, y2 = m * x1 + b, m * x2 + b
        cv2.line(outmask, (int(x1), int(y1)), (int(x2), int(y2)), 255, thickness=2)

    return outmask


def main():
    _ = RANSACNode(int(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))
    rospy.init_node("ransac", anonymous=True)
    rospy.spin()


if __name__ == "__main__":
    main()
