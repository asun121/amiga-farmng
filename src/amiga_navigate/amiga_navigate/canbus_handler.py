import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
import serial


class CANBusHandler(Node):

    def __init__(self):
        super().__init__('canbus_handler')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.ser = serial.Serial('/dev/ttyACM0', 115200)

    def listener_callback(self, msg):
        self.get_logger().info('Twist: "%s"' % msg)
        twist_str = str(msg.linear.x) + " " + str(msg.angular.z)
        self.ser.write(twist_str.encode('utf-8'))


def main(args=None):
    rclpy.init(args=args)

    handler = CANBusHandler()

    rclpy.spin(handler)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    handler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()