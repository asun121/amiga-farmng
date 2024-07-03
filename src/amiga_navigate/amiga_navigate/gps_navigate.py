import rclpy
from rclpy.node import Node
from rclpy.service import Service
from geometry_msgs.msg import PoseStamped
import pyproj
from amiga_interfaces.srv import Waypoint

class GPSNavigate(Node):
    def __init__(self):
        super().__init__('gps_navigate')
        self.service = self.create_service(Waypoint, 'navigate', self.provide_waypoint_callback)
        # self.waypoints = [ #TODO: Change from list to a queue os you don't have to track index
        #     [1.0, 1.0],
        #     [2.0, 0.0],
        #     [3.0, 1.0],
        #     [4.0, 0.0],
        #     [5.0, 1.0],
        # ]

        self.declare_parameter('waypoints', [])
        waypoints_param = self.get_parameter('waypoints').get_parameter_value().string_array_value
        self.publisher_ = self.create_publisher(PoseStamped, 'gps_origin', 10)


        self.waypoints = self.parse_waypoints(waypoints_param)
        self.current_index = 0

        self.get_logger().info('Navigate service started')

    def parse_waypoints(self, waypoints_param):
        waypoints = []
        for s in waypoints_param:
            x,y = map(float, s.split(','))
            waypoint = PoseStamped()
            waypoint.pose.position.x = x
            waypoint.pose.position.y = y
            waypoint.pose.position.z = 0.0
            waypoints.append(waypoint)
        # Define the projection (WGS84 to UTM)
        wgs84 = pyproj.Proj(proj='latlong', datum='NAD83')
        utm = pyproj.Proj(proj='utm', zone=18, datum='NAD83')  # Change zone according to your location
        
        # Convert the first waypoint to UTM and set as origin
        origin_x, origin_y = pyproj.transform(wgs84, utm, waypoints[0].pose.position.x, waypoints[0].pose.position.y)
        
        relative_waypoints = []
        for way in waypoints:
            x,y = way.pose.position.x, way.pose.position.y
            x_m, y_m = pyproj.transform(wgs84, utm, x, y)
            relative_x = x_m - origin_x
            relative_y = y_m - origin_y
            relative_waypoints.append((relative_x, relative_y))
        orig = PoseStamped()
        orig.pose.position.x = relative_waypoints[0][0]
        orig.pose.position.y = relative_waypoints[0][1]
        self.publisher_.publish(orig)
        return relative_waypoints

    def provide_waypoint_callback(self, request, response):
        self.get_logger().info('Incoming Request')
        if self.current_index < len(self.waypoints):
            waypoint = self.waypoints[self.current_index]
            response.waypoint = PoseStamped()
            response.waypoint.header.frame_id = 'map'
            response.waypoint.header.stamp = self.get_clock().now().to_msg()
            response.waypoint.pose.position.x = float(waypoint[0])
            response.waypoint.pose.position.y = float(waypoint[1])
            response.waypoint.pose.position.z = 0.0
            response.success = True
            self.current_index += 1
        else:
            self.get_logger().info('No waypoints in list')

            response.success = False
        return response
    def convert_to_meters(waypoints):
        # Define the projection (WGS84 to UTM)
        wgs84 = pyproj.Proj(proj='latlong', datum='WGS84')
        utm = pyproj.Proj(proj='utm', zone=18, datum='WGS84')  # Change zone according to your location
        
        # Convert the first waypoint to UTM and set as origin
        origin_x, origin_y = pyproj.transform(wgs84, utm, waypoints[0][0], waypoints[0][1])
        
        relative_waypoints = []
        for lat, lon in waypoints:
            x, y = pyproj.transform(wgs84, utm, lat, lon)
            relative_x = x - origin_x
            relative_y = y - origin_y
            relative_waypoints.append((relative_x, relative_y))
        
        return relative_waypoints

def main(args=None):
    rclpy.init(args=args)
    node = GPSNavigate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
