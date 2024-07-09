import serial
import pynmea2

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
from geopy.distance import geodesic
import math

GPS_PORT = '/dev/ttyACM0'

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
        self.subscription = self.create_subscription(PoseStamped, '/gps_origin', self.origin_callback, 10)
        default_orig = PoseStamped()
        default_orig.pose.position.x = 42.875809
        default_orig.pose.position.y = -77.007174
        self.origin = default_orig
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def origin_callback(self, msg):
        self.origin = msg
        self.get_logger().info('Origin set to: "%s"' % msg.pose.position.x + ',' + msg.pose.position.y)

    def timer_callback(self):
        msg = Odometry()
        latitude,longitude,heading = read_gps_data('/dev/ttyACM0')
        curr_pose = self.convert_to_meters
        msg.pose.pose.position.x = longitude
        msg.pose.pose.position.y = latitude
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = euler_to_quaternion(heading, 0 ,0)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % latitude + ',' + longitude + ',' + heading)

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
    def convert_to_meters(self,gps):
        orig = [self.origin.pose.position.x,self.origin.pose.position.y]
        pos = [gps.pose.position.x, gps.pose.position.y]
        distance = geodesic(orig, pos).meters
        bearing = self.calculate_initial_compass_bearing(orig, pos)
        
        relative_x = distance * math.cos(math.radians(bearing))
        relative_y = distance * math.sin(math.radians(bearing))
        
        return [relative_x, relative_y]

    def calculate_initial_compass_bearing(self, pointA, pointB):
        lat1 = math.radians(pointA[0])
        lat2 = math.radians(pointB[0])
        diffLong = math.radians(pointB[1] - pointA[1])

        x = math.sin(diffLong) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(diffLong))

        initial_bearing = math.atan2(x, y)
        initial_bearing = math.degrees(initial_bearing)
        compass_bearing = (initial_bearing + 360) % 360

        return compass_bearing

def main(args=None):
    rclpy.init(args=args)
    gps = GPS()
    rclpy.spin(gps)
    gps.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
