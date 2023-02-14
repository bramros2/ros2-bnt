import rclpy
from rclpy.node import Node
import cv2

from sensor_msgs.msg      import Image
from cv_bridge            import CvBridge, CvBridgeError


class MaskSubscriber(Node):

    def __init__(self):
        super().__init__('mask_subscriber')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/image/flow',
            self.listener_callback,
            1)
        self.get_logger().info('Node initialised')
        self.subscription  # prevent unused variable warning
        

    def listener_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
        cv2.imshow("Mask", cv_image)
        self.get_logger().info("I heard, Image size: '%s'" % data.width)



def main(args=None):
    rclpy.init(args=args)

    mask_subscriber = MaskSubscriber()

    rclpy.spin(mask_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mask_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()