import serial
import pynmea2

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D

PORT = '/dev/ttyUSB0'

def read_gps_data(serial_port):
    with serial.Serial(serial_port, 9600, timeout=1) as ser:
        line = ser.readline().decode('utf-8').strip()
        if line.startswith('$GP') or line.startswith('$GN'):
            try:
                msg = pynmea2.parse(line)
                if isinstance(msg, pynmea2.types.talker.GGA):
                    latitude = msg.latitude
                    longitude = msg.longitude
                elif isinstance(msg, pynmea2.types.talker.VTG):
                    heading = msg.true_track
                elif isinstance(msg, pynmea2.types.talker.GLL):
                    latitude = msg.latitude
                    longitude = msg.longitude
                elif isinstance(msg, pynmea2.types.talker.RMC):
                    latitude = msg.latitude
                    longitude = msg.longitude
                    heading = msg.true_course
                elif isinstance(msg, pynmea2.types.talker.GNS):
                    latitude = msg.latitude
                    longitude = msg.longitude
                    heading = msg.true_track
                return latitude, longitude, heading
            except pynmea2.ParseError as e:
                print(f"Parse error: {e}")

class GPS(Node):

    def __init__(self):
        super().__init__('gps')
        self.get_logger().info('Launching GPS')

        self.publisher_ = self.create_publisher(Pose2D, 'position', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Pose2D()
        y,x,theta = read_gps_data(PORT)
        msg.x = x
        msg.y = y
        msg.theta = theta
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.x + ',' + msg.y + ',' + msg.theta)
        self.i += 1
        
def main(args=None):
    rclpy.init(args=args)
    gps = GPS()
    rclpy.spin(gps)
    gps.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
