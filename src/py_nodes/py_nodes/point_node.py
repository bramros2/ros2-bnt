import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg      import Point


class PointSubscriber(Node):

    def __init__(self):
        super().__init__('point_subscriber')
        self.subscription = self.create_subscription(
            Point,
            '/image/point',
            self.listener_callback,
            1)
        self.get_logger().info('Node initialised')
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info("Recieved point data")


def main(args=None):
    rclpy.init(args=args)

    point_subscriber = PointSubscriber()

    rclpy.spin(point_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    point_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()