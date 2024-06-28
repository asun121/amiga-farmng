
import rclpy
from rclpy.node import Node
from simple_pid import PID
from rclpy.task import Future
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
from time import sleep
from amiga_interfaces.srv import Waypoint

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        self.declare_parameter('kp_linear', 0.3)
        self.declare_parameter('ki_linear', 0.0)
        self.declare_parameter('kd_linear', 0.0)
        self.declare_parameter('kp_angular', 0.65)
        self.declare_parameter('ki_angular', 0.0)
        self.declare_parameter('kd_angular', 0.1)

        kp_linear = self.get_parameter('kp_linear').get_parameter_value().double_value
        ki_linear = self.get_parameter('ki_linear').get_parameter_value().double_value
        kd_linear = self.get_parameter('kd_linear').get_parameter_value().double_value

        kp_angular = self.get_parameter('kp_angular').get_parameter_value().double_value
        ki_angular = self.get_parameter('ki_angular').get_parameter_value().double_value
        kd_angular = self.get_parameter('kd_angular').get_parameter_value().double_value

        self.linear_pid = PID(kp_linear, ki_linear, kd_linear, setpoint=0)
        self.angular_pid = PID(kp_angular, ki_angular, kd_angular, setpoint=0)

        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.current_pose = None
        self.stop = Twist()
        self.stop.linear.x = 0.0
        self.stop.angular.z = 0.0
        self.current_waypoint = None
        self.request_new_waypoint()

    def request_new_waypoint(self):
        self.cli = self.create_client(Waypoint, 'navigate')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waypoint service not available, waiting...')
            self.cmd_vel_publisher.publish(self.stop)



        self.req = Waypoint.Request()
        self.future = self.cli.call_async(self.req)
        #rclpy.spin_until_future_complete(self,self.future)
        self.future.add_done_callback(self.waypoint_response_callback)

    def waypoint_response_callback(self, future: Future):
        try:
            response = future.result()
            if response.success:
                self.current_waypoint = response.waypoint
                self.get_logger().info(f'New waypoint received: {self.current_waypoint}')
            else:
                self.get_logger().info('No more waypoints available')
                self.cmd_vel_publisher.publish(self.stop)
                sleep(1)


        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        if self.current_waypoint:
            self.pid_control()

    def pid_control(self):
        if not self.current_waypoint:
            return

        distance = self.get_distance(self.current_pose.position, self.current_waypoint.pose.position)
        if distance < 0.1:
            self.request_new_waypoint()
            return

        angle_to_goal = math.atan2(
            self.current_waypoint.pose.position.y - self.current_pose.position.y,
            self.current_waypoint.pose.position.x - self.current_pose.position.x
        )

        orientation = self.current_pose.orientation

        yaw = self.quaternion_to_euler(orientation)
        angle_diff = self.angle_wrap(angle_to_goal - yaw)

        linear_speed = self.linear_pid(distance)
        angular_speed = self.angular_pid(angle_diff)

        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self.cmd_vel_publisher.publish(twist)
#     def control_loop(self):
#         if self.current_pose and self.current_waypoint:
#             orientation = self.current_pose.orientation
#             yaw = self.quaternion_to_euler(orientation)

#             dx = self.current_waypoint[0] - position.x
#             dy = self.current_waypoint[1] - position.y

#             distance = math.sqrt(dx**2 + dy**2)
#             angle_to_goal = math.atan2(dy, dx)
#             angle_error = self.normalize_angle(angle_to_goal - yaw)

#             control_signal = self.pid_angular(angle_error)
#             linear_signal = self.pid_linear(distance)

#             twist = Twist()
#             twist.linear.x = linear_signal  # Adjust this constant based on your robot's capabilities
#             twist.angular.z = control_signal

#             self.velocity_publisher.publish(twist)
#             self.get_logger().info('Current Twist: "%s"' % twist)

#             if distance < 0.15:
#                 self.current_waypoint = None
#         else:
#             zero = Twist()
#             zero.angular.z = 0
#             zero.linear.x = 0
#             self.velocity_publisher.publish(zero)
    def quaternion_to_euler(self, orientation):
        q = orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def get_distance(position1, position2):
        return math.sqrt((position1.x - position2.x) ** 2 + (position1.y - position2.y) ** 2)

    @staticmethod
    def angle_wrap(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



################### OLD PID WITHOUT ACTION #######################################
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from std_msgs.msg import Float32MultiArray
# from simple_pid import PID
# import math

# class PIDController(Node):
#     def __init__(self):
#         super().__init__('pid_controller')
#         self.declare_parameter('kp', 0.3)
#         self.declare_parameter('ki', 0.0)
#         self.declare_parameter('kd', 0.0)
        
#         kp = self.get_parameter('kp').get_parameter_value().double_value
#         ki = self.get_parameter('ki').get_parameter_value().double_value
#         kd = self.get_parameter('kd').get_parameter_value().double_value

#         self.pid_linear = PID(kp, ki, kd, setpoint=0)
#         self.pid_linear.sample_time = 0.1  # Update at 10 Hz

#         self.pid_angular = PID(kp, ki, kd, setpoint=0)
#         self.pid_angular.sample_time = 0.1  # Update at 10 Hz
#         self.get_logger().info('Launching PID Controller')

#         self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
#         self.odom_subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
#         #self.waypoint_subscription = self.create_subscription(Float32MultiArray, 'waypoints', self.waypoint_callback, 10)

#         self.current_waypoint = [-1,1]
#         self.current_pose = None

#     def odom_callback(self, msg):
#         self.current_pose = msg.pose.pose
#         self.get_logger().info('Current pose: "%s"' % self.current_pose)
#         if self.current_waypoint:
#             self.control_loop()

#     def waypoint_callback(self, msg):
#         if msg.data:
#             self.get_logger().info('Current target: "%s"' % msg.data)
#             self.current_waypoint = (msg.data[0], msg.data[1])

#     def control_loop(self):
#         if self.current_pose and self.current_waypoint:
#             position = self.current_pose.position
#             orientation = self.current_pose.orientation
#             yaw = self.quaternion_to_euler(orientation)

#             dx = self.current_waypoint[0] - position.x
#             dy = self.current_waypoint[1] - position.y

#             distance = math.sqrt(dx**2 + dy**2)
#             angle_to_goal = math.atan2(dy, dx)
#             angle_error = self.normalize_angle(angle_to_goal - yaw)

#             control_signal = self.pid_angular(angle_error)
#             linear_signal = self.pid_linear(distance)

#             twist = Twist()
#             twist.linear.x = linear_signal  # Adjust this constant based on your robot's capabilities
#             twist.angular.z = control_signal

#             self.velocity_publisher.publish(twist)
#             self.get_logger().info('Current Twist: "%s"' % twist)

#             if distance < 0.15:
#                 self.current_waypoint = None
#         else:
#             zero = Twist()
#             zero.angular.z = 0
#             zero.linear.x = 0
#             self.velocity_publisher.publish(zero)


#     def quaternion_to_euler(self, orientation):
#         q = orientation
#         siny_cosp = 2 * (q.w * q.z + q.x * q.y)
#         cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
#         return math.atan2(siny_cosp, cosy_cosp)

#     def normalize_angle(self, angle):
#         return math.atan2(math.sin(angle), math.cos(angle))

# def main(args=None):
#     rclpy.init(args=args)
#     pid_controller = PIDController()
#     rclpy.spin(pid_controller)
#     pid_controller.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

