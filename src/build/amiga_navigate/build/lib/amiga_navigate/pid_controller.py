import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray
from simple_pid import PID
import math

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        
        kp = self.get_parameter('kp').get_parameter_value().double_value
        ki = self.get_parameter('ki').get_parameter_value().double_value
        kd = self.get_parameter('kd').get_parameter_value().double_value

        self.pid_linear = PID(kp, ki, kd, setpoint=0)
        self.pid_linear.sample_time = 0.1  # Update at 10 Hz

        self.pid_angular = PID(kp, ki, kd, setpoint=0)
        self.pid_angular.sample_time = 0.1  # Update at 10 Hz
        self.get_logger().info('Launching PID Controller')

        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pose_subscription = self.create_subscription(Pose2D, 'position', self.pose_callback, 10)
        self.waypoint_subscription = self.create_subscription(Float32MultiArray, 'waypoints', self.waypoint_callback, 10)

        self.current_waypoint = None
        self.pose = None

    def pose_callback(self, msg):
        self.pose = msg
        self.get_logger().info('Current pose: "%s"' % msg)
        if self.current_waypoint:
            self.control_loop()

    def waypoint_callback(self, msg):
        if msg.data:
            self.get_logger().info('Current target: "%s"' % msg.data)
            self.current_waypoint = (msg.data[0], msg.data[1])

    def control_loop(self):
        if self.pose and self.current_waypoint:
            position = self.pose
            yaw = self.pose.theta

            dx = self.current_waypoint[0] - position.x
            dy = self.current_waypoint[1] - position.y

            distance = math.sqrt(dx**2 + dy**2)
            angle_to_goal = math.atan2(dy, dx)
            angle_error = self.normalize_angle(angle_to_goal - yaw)

            control_signal = self.pid_angular(angle_error)
            linear_signal = self.pid_linear(distance)

            twist = Twist()
            twist.linear.x = linear_signal  # Adjust this constant based on your robot's capabilities
            twist.angular.z = control_signal

            self.velocity_publisher.publish(twist)

            if distance < 0.1:
                self.current_waypoint = None

    # def quaternion_to_euler(self, orientation):
    #     q = orientation
    #     siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    #     cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    #     return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

def main(args=None):
    rclpy.init(args=args)
    pid_controller = PIDController()
    rclpy.spin(pid_controller)
    pid_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

