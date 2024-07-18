import serial
import pynmea2

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np

GPS_PORT = '/dev/ttyACM0'



def read_gps_data(serial_port):
    with serial.Serial(serial_port, 38400, timeout=1) as ser:
        line = ser.readline().decode('utf-8').strip()
        if line.startswith('$GP') or line.startswith('$GN'):
            try:
                msg = pynmea2.parse(line)
                if isinstance(msg, pynmea2.types.talker.GGA):
                    latitude = msg.latitude
                    longitude = msg.longitude
                    heading = 0.0
                elif isinstance(msg, pynmea2.types.talker.VTG):
                    heading = msg.true_track
                elif isinstance(msg, pynmea2.types.talker.GLL):
                    latitude = msg.latitude
                    longitude = msg.longitude
                    heading = 0.0
                elif isinstance(msg, pynmea2.types.talker.RMC):
                    latitude = msg.latitude
                    longitude = msg.longitude
                    heading = msg.true_course
                elif isinstance(msg, pynmea2.types.talker.GNS):
                    latitude = msg.latitude
                    longitude = msg.longitude
                    heading = msg.true_track
                return [latitude, longitude, heading]
            except pynmea2.ParseError as e:
                print(f"Parse error: {e}")

def euler_to_quaternion(yaw, pitch, roll):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        quat = Quaternion()
        quat.x = qx
        quat.y = qy
        quat.z = qz
        quat.w = qw
        return quat

class GPS(Node):

    def __init__(self):
        super().__init__('gps')
        self.get_logger().info('Launching GPS Module')

        self.publisher_ = self.create_publisher(Odometry, '/odom', 10)
        try:
            self.gps_port = serial.Serial('/dev/ttyACM0', 38400, timeout=1)
        except:
            self.gps_port = serial.Serial('/dev/ttyACM1', 38400, timeout=1)

        self.latitude = None
        self.longitude = None
        self.heading = 0.0

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def get_gps(self):
        line = self.gps_port.readline().decode('utf-8').strip()
        if line.startswith('$GPGGA'):
            try:
                msg = pynmea2.parse(line)
                if isinstance(msg, pynmea2.types.talker.GGA):
                    self.latitude = msg.latitude
                    self.longitude = msg.longitude
            except pynmea2.ParseError as e:
                print(f"Parse error: {e}")
        elif line.startswith('$GPETC'):
            try:
                msg = line.split(',')
                self.heading = float(msg[4])
            except pynmea2.ParseError as e:
                print(f"Parse error: {e}")

    def timer_callback(self):
        msg = Odometry()
        self.get_gps()
        gp = [self.latitude, self.longitude, self.heading]
       # latitude,longitude,heading = gp[0], gp[1], gp[2]
        msg.pose.pose.position.x = gp[0]
        msg.pose.pose.position.y = gp[1]
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = euler_to_quaternion(gp[2], 0 ,0)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % str(gp[0]) + ',' + str(gp[1]) + ',' + str(gp[2]))

    # #Convert current gps data to meters relative to the origin
    # def convert_to_meters(self,gps):
    #     # Define the projection (WGS84 to UTM)
    #     wgs84 = pyproj.Proj(proj='latlong', datum='WGS84')
    #     utm = pyproj.Proj(proj='utm', zone=18, datum='WGS84')  # Change zone according to your location
        
    #     # Convert the first waypoint to UTM and set as origin
    #     origin_x, origin_y = self.origin.pose.position.x, self.origin.pose.position.y
        
    #     x,y = gps.pose.position.x, gps.pose.position.y
    #     x_m, y_m = pyproj.transform(wgs84, utm, x, y)
    #     relative_x = x_m - origin_x
    #     relative_y = y_m - origin_y
    #     relative_pos = (relative_x, relative_y)        
    #     return relative_pos


def main(args=None):
    rclpy.init(args=args)
    gps = GPS()
    rclpy.spin(gps)
    gps.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
