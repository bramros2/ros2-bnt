import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import argparse
from operator import xor


class RangeDetectorNode(Node):
    def __init__(self):
        super().__init__('range_detector')

        self.cv_bridge = CvBridge()

        # Set up the arguments
        ap = argparse.ArgumentParser()
        ap.add_argument('-f', '--filter', required=True,
                        help='Range filter. RGB or HSV')
        ap.add_argument('-i', '--image', required=False,
                        help='Path to the image')
        ap.add_argument('-w', '--webcam', required=False,
                        help='Use webcam', action='store_true')
        ap.add_argument('-p', '--preview', required=False,
                        help='Show a preview of the image after applying the mask',
                        action='store_true')
        args = vars(ap.parse_args())

        if not xor(bool(args['image']), bool(args['webcam'])):
            ap.error("Please specify only one image source")

        if not args['filter'].upper() in ['RGB', 'HSV']:
            ap.error("Please specify a correct filter.")

        self.range_filter = args['filter'].upper()

        if args['image']:
            image = cv2.imread(args['image'])

            if self.range_filter == 'RGB':
                self.frame_to_thresh = image.copy()
            else:
                self.frame_to_thresh = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        else:
            self.camera = cv2.VideoCapture(0)

        self.setup_trackbars()

        self.subscription = self.create_subscription(
            Image,
            'image',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.range_filter == 'RGB':
            self.frame_to_thresh = image.copy()
        else:
            self.frame_to_thresh = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = self.get_trackbar_values()

        thresh = cv2.inRange(self.frame_to_thresh, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))

        if args['preview']:
            preview = cv2.bitwise_and(image, image, mask=thresh)
            cv2.imshow("Preview", preview)
        else:
            cv2.imshow("Original", image)
            cv2.imshow("Thresh", thresh)

        if cv2.waitKey(1) & 0xFF is ord('q'):
            cv2.destroyAllWindows()
            self.destroy_node()
            rclpy.shutdown()

    def setup_trackbars(self):
        cv2.namedWindow("Trackbars", 0)

        for i in ["MIN", "MAX"]:
            v = 0 if i == "MIN" else 255

            for j in self.range_filter:
                cv2.createTrackbar("%s_%s" % (j, i), "Trackbars", v, 255, self.callback)

    def callback(self, value):
        pass

    def get_trackbar_values(self):
        values = []

        for i in ["MIN", "MAX"]:
            for j in self.range_filter:
                v = cv2.getTrackbarPos("%s_%s" % (j, i), "