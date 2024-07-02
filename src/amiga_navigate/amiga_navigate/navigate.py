import rclpy
from rclpy.node import Node
from rclpy.service import Service
from geometry_msgs.msg import PoseStamped
from amiga_interfaces.srv import Waypoint

class Navigate(Node):
    def __init__(self):
        super().__init__('navigate')
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
        self.waypoints = self.parse_waypoints(waypoints_param)
        self.current_index = 0
        self.get_logger().info('Navigate service started')

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
    node = Navigate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




############################################# OLD NAV ################################################



# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray

# class Navigate(Node):
#     def __init__(self):
#         super().__init__('navigate')
#         self.get_logger().info('Beginning Navigation')

#         self.publisher_ = self.create_publisher(Float32MultiArray, 'waypoints', 10)
#         timer_period = 10.0  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)
#         self.waypoints = [(1.0, 1.0), (2.0, 2.0), (3.0, 3.0)]
#         self.index = 0

#     def timer_callback(self):
#         if self.index < len(self.waypoints):
#             waypoint = self.waypoints[self.index]
#             msg = Float32MultiArray(data=list(waypoint))
#             self.publisher_.publish(msg)
#             self.get_logger().info('Publishing Point: ' + str(waypoint))

#             self.index += 1

# def main(args=None):
#     rclpy.init(args=args)
#     navigate = Navigate()
#     rclpy.spin(navigate)
#     navigate.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
