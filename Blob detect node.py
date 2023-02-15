import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class BlobDetectorNode(rclpy.Node):
    def __init__(self):
        super().__init__('blob_detector_node')
        self._bridge = CvBridge()
        self._sub = self.create_subscription(Image, 'image_raw', self.callback, 10)
        self._pub = self.create_publisher(Image, 'image_with_blobs', 10)

    def callback(self, msg):
        try:
            image = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        result = self.detect_blobs(image)

        try:
            result_msg = self._bridge.cv2_to_imgmsg(result, "bgr8")
            result_msg.header = msg.header
            self._pub.publish(result_msg)
        except CvBridgeError as e:
            print(e)

    def detect_blobs(self, image):
        # Convert image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Detect blobs in the image
        params = cv2.SimpleBlobDetector_Params()
        detector = cv2.SimpleBlobDetector_create(params)
        keypoints = detector.detect(gray)

        # Print the width of each blob in pixels            #should be in detect_blobs
        for keypoint in keypoints:
            x, y, w, h = cv2.boundingRect(keypoint.pt)
            print('Blob width:', w)

        # Draw blobs on the image
        result = cv2.drawKeypoints(image, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        return result



def main(args=None):
    rclpy.init(args=args)

    node = BlobDetectorNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()