import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from tf_transformations import euler_from_quaternion
import math
from std_msgs.msg import Float32MultiArray


class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')

        self.declare_parameter('lookahead_distance', 0.5)
        self.declare_parameter('linear_velocity', 0.2)
        self.declare_parameter('max_angular_velocity', 1.0)

        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.linear_velocity = self.get_parameter('linear_velocity').get_parameter_value().double_value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').get_parameter_value().double_value

        self.waypoint_subscription = self.create_subscription(
            Float32MultiArray,
            'waypoints',
            self.waypoint_callback,
            10
        )
        self.pose_subscription = self.create_subscription(Pose2D, 'position', self.pose_callback, 10)

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.current_pose = None
        self.waypoints = []

    def waypoint_callback(self, path):
        self.waypoints = path
        self.get_logger().info('Received waypoints')

    def pose_callback(self, msg):
        self.current_pose = msg
        if self.waypoints:
            self.pure_pursuit_control()

    def pure_pursuit_control(self):
        lookahead_point = self.get_lookahead_point()

        if lookahead_point is None:
            self.get_logger().info('Reached final waypoint')
            return

        # Calculate the control commands
        angle_to_goal = math.atan2(
            lookahead_point[1] - self.current_pose.y,
            lookahead_point[0] - self.current_pose.x
        )

        yaw = self.current_pose.theta
        angle_diff = self.angle_wrap(angle_to_goal - yaw)

        twist = Twist()
        twist.linear.x = self.linear_velocity
        twist.angular.z = max(min(2 * angle_diff, self.max_angular_velocity), -self.max_angular_velocity)
        self.cmd_vel_publisher.publish(twist)

    def get_lookahead_point(self):
        for waypoint in self.waypoints:
            distance = self.get_distance(self.current_pose.position, waypoint.pose.position)
            if distance >= self.lookahead_distance:
                return (waypoint.pose.position.x, waypoint.pose.position.y)
        return None

    @staticmethod
    def get_distance(position1, position2):
        return math.sqrt((position1.x - position2.x) ** 2 + (position1.y - position2.y) ** 2)

    @staticmethod
    def angle_wrap(angle):
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()