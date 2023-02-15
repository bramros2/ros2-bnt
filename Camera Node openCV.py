import rclpy
import v4l2
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraNode:
    def __init__(self):
        self.node = rclpy.create_node('camera_node')
        self.pub = self.node.create_publisher(Image, 'image_raw')
        self.bridge = CvBridge()

        # Open the video capture
        self.cap = cv2.VideoCapture(0)
        # Configure the camera
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

    def run(self):
        # Read image data from camera
        ret, frame = self.cap.read()

        # Convert to ROS2 message
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.node.get_clock().now().to_msg()

        # Publish the message
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    timer_period = 0.033  # in seconds
    timer = camera_node.node.create_timer(timer_period, camera_node.run)
    rclpy.spin(camera_node.node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()