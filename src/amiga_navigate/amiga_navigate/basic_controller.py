import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry

from geometry_msgs.msg import Twist
from time import sleep
from amiga_interfaces.srv import Waypoint
from rclpy.task import Future
import math

class BasicController(Node):

    def __init__(self):
        super().__init__('basic_controller')
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.position_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.vel_command = self.create_publisher(Twist, 'cmd_vel',10)
        self.current_waypoint = None

        self.linear_p = 0.5

        self.pos = None
        self.heading = None


    def position_callback(self, msg):
        self.get_logger().info('Curr Position: "%s"' % msg)
        self.pos = msg.pose.pose.position
        self.heading = self.quaternion_to_euler(msg.pose.pose.orientation)
        try:
            self.controller()
        except:
            self.get_logger().warning('Failed Control')
  
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

    def controller(self):
        if(self.current_waypoint):
            err_x = self.current_waypoint.x - self.pos.x
            err_y = self.current_waypoint.y - self.pos.y

            err_dist = float((err_x**2.0+err_y**2.0)**0.5)

            err_theta = math.atan2(err_y, err_x) - (self.heading * math.pi / 180.0)

            while(err_theta > math.pi):
                err_theta -= 2.0 * math.pi
            while(err_theta < -math.pi):
                err_theta += 2.0 * math.pi
            self.get_logger().info(f"Theta Error: {err_theta}, Distance Error: {err_dist}")

            l_v = math.cos(err_theta) * self.linear_p
            a_v = err_theta

            cmd = Twist()
            cmd.linear.x = l_v
            cmd.angular.z = a_v

            self.vel_command.publish(cmd)
        else:
            self.get_logger().warn(f'Controller Passed')

    

    def quaternion_to_euler(self, orientation):
        q = orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
def main(args=None):
    rclpy.init(args=args)

    controller = BasicController()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()