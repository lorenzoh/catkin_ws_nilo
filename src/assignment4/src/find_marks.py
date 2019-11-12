
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image


RECT = [110, 245, 190, 520]  # y1, y2, x1, x2

THRESHOLD = 230

MARKER_RECTS = [
    [0, 30, 0, 150],  # upper left
    [0, 20, 180, 260],  # upper right
    [20, 70, 0, 150],  # middle left
    [20, 70, 180, 260],  # middle right
    [70, 134, 0, 150],  # bottom left
    [70, 134, 180, 330],  # bottom right
]

# intrinsic camera coefficients obtained through `export_camera.py`
CPARAMS = {
    "fx": 383.7944641113281, "fy": 383.7944641113281,
    "cx": 322.3056945800781, "cy": 241.67051696777344,
    "k1": 0.0, "k2": 0.0, "t1": 0.0, "t2": 0.0, "k3": 0.0}

A = np.array([
    [CPARAMS["fx"], 0, CPARAMS["cx"]],
    [0, CPARAMS["fy"], CPARAMS["cy"]],
    [0, 0, 1],
])

DIST_COEFFS = np.array([CPARAMS[key] for key in ["k1", "k2", "t1", "t2", "k3"]])

OBJECT_POINTS = np.array([
    [1.1, 0.2, 1.],  # upper left
    [1.1, -.2, 1.],  # upper right
    [0.8, 0.2, 1.],  # middle left
    [0.8, -.2, 1.],  # middle right
    [0.5, 0.2, 1.],  # bottom left
    [0.5, -.2, 1.],  # bottom right

])


class MarkerFinder:

    def __init__(self):
        self.publisher_thresh = rospy.Publisher(
            "/outputs/image_thresh", Image, queue_size=10
        )
        self.publisher_image = rospy.Publisher(
            "/outputs/image_marked", Image, queue_size=10
        )
        self.subsriber = rospy.Subscriber(
            "/sensors/camera/infra1/image_rect_raw", Image, self.callback
        )
        self.bridge = CvBridge()

    def callback(self, data):
        # load and crop image
        image = self.parse_image(data)
        imagecropped = crop(image, RECT)

        # convert to grayscale and threshold
        mask = cv2.cvtColor(imagecropped, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(mask, THRESHOLD, 255, cv2.THRESH_BINARY)

        self.publish_thresh(mask)

        # find all 6 markers in every region
        marker_positions_cropped = [locate_marker(mask, marker_rect) for marker_rect in MARKER_RECTS]
        marker_positions = [translateback(p, RECT) for p in marker_positions_cropped]
        rospy.loginfo("Marker positions: " + str(marker_positions))

        # solvepnp
        rvec, tvec = solve_rvec_tvec(np.array(marker_positions))
        rospy.loginfo("rvec: " + str(rvec.ravel()) + ", tvec: " + str(tvec.ravel()))

        # get rotation matrix
        rmat, _ = cv2.Rodrigues(rvec)
        rospy.loginfo("Rotation matrix: " + str(rmat))


        # draw markers on image to visualize
        for (y, x) in marker_positions:
            cv2.circle(image, (x, y), 4, [0, 0, 255], thickness=-1)


        self.publish_image(image)

    def parse_image(self, data):
        return self.bridge.imgmsg_to_cv2(data, "bgr8")

    def publish_thresh(self, mask):
        self.publisher_thresh.publish(self.bridge.cv2_to_imgmsg(mask, "8UC1"))

    def publish_image(self, image):
        self.publisher_image.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))




def locate_marker(image, rect):
    """
    Calculate the mean value of all coordinates in image[rect] that are non-zero

    Returns a coordinate in `image`s coordinate system
    """
    region = crop(image, rect)
    idxsy, idxsx = np.nonzero(region)

    p = [int(np.mean(idxsy)), int(np.mean(idxsx))]
    return translateback(p, rect)


def solve_rvec_tvec(image_points):
    ret, rvec, tvec = cv2.solvePnP(OBJECT_POINTS, image_points.astype(np.float32), A, DIST_COEFFS)
    return rvec, tvec


# Utilities

def crop(image, rect):
    return image[rect[0]:rect[1], rect[2]:rect[3]]

def translateback(p, rect):
    return [p[0] + rect[0], p[1] + rect[2]]

def main():
    _ = MarkerFinder()
    rospy.init_node("marks", anonymous=True)
    rospy.spin()


if __name__ == "__main__":
    main()
