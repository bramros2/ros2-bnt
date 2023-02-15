import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller')

        # Initialize PID controller with some parameters
        self.Kp = 0.5
        self.Ki = 0.1
        self.Kd = 0.2
        self.min_ref = -10
        self.max_ref = 10
        self.min_output = -1
        self.max_output = 1
        self.integral = 0
        self.last_error = 0
        self.last_time = self.get_clock().now()

        # Create a subscription to the point topic
        self.subscription = self.create_subscription(
            Point,
            'point_topic',
            self.callback,
            10)

        # Create a publisher for the control signal
        self.publisher = self.create_publisher(
            Float64,
            'control_signal_topic',
            10)

    def callback(self, msg):
        # Get the new reference value from the received point message
        new_ref = msg.x

        # Calculate the elapsed time since the last update
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9

        # Calculate the error between the current position and the new reference
        error = new_ref - current_position

        # Calculate the proportional term
        p = self.Kp * error

        # Calculate the integral term
        self.integral += error * dt
        i = self.Ki * self.integral

        # Calculate the derivative term
        derivative = (error - self.last_error) / dt
        d = self.Kd * derivative

        # Calculate the control signal as the sum of the three terms
        control_signal = p + i + d

        # Limit the control signal to the specified range
        control_signal = max(control_signal, self.min_output)
        control_signal = min(control_signal, self.max_output)

        # Publish the control signal as a Float64 message
        control_signal_msg = Float64()
        control_signal_msg.data = control_signal
        self.publisher.publish(control_signal_msg)

        # Update the last error and time for the next iteration
        self.last_error = error
        self.last_time = now


def main(args=None):
    rclpy.init(args=args)

    pid_controller = PIDControllerNode()

    rclpy.spin(pid_controller)

    pid_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()