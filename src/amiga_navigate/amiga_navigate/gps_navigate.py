import rclpy
from rclpy.node import Node
from rclpy.service import Service
from geometry_msgs.msg import PoseStamped
# import pyproj
from amiga_interfaces.srv import Waypoint

class GPSNavigate(Node):
    def __init__(self):
        super().__init__('gps_navigate')
        self.service = self.create_service(Waypoint, 'navigate', self.provide_waypoint_callback)
        self.declare_parameter('waypoints')
        waypoints_param = self.get_parameter('waypoints').get_parameter_value().string_array_value

        self.waypoints = self.parse_waypoints(waypoints_param)
        self.current_index = 0

        self.get_logger().info('Navigate service started')

    def parse_waypoints(self, waypoints_param):
        waypoints = []
        self.get_logger().info(f'Waypoints: {waypoints_param}')
        for s in waypoints_param:
            x,y = map(float, s.split(','))
            waypoints.append([x,y])
        return waypoints


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


def main(args=None):
    rclpy.init(args=args)
    node = GPSNavigate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
