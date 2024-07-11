import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from amiga_interfaces.srv import Waypoint
import do_mpc
import numpy as np
from casadi import *


class MPCController(Node):
    def __init__(self):
        super().__init__('mpc_controller')

        self.declare_parameter('mpc_horizon', 10)
        self.declare_parameter('max_vel_x', 0.5)
        self.declare_parameter('max_vel_theta', 1.0)
        self.declare_parameter('lookahead_distance', 1.0)

        self.mpc_horizon = self.get_parameter('mpc_horizon').value
        self.max_vel_x = self.get_parameter('max_vel_x').value
        self.max_vel_theta = self.get_parameter('max_vel_theta').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value

        self.current_pose = None
        self.current_velocity = None
        self.current_waypoint = None

        self.odom_subscriber = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.waypoint_client = self.create_client(Waypoint, 'navigate')
        while not self.waypoint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waypoint service not available, waiting...')

        self.timer = self.create_timer(0.1, self.control_loop)

        self.model = model()
        self.mpc = self.setup_mpc(self.model)

    def setup_mpc(self, model):
        mpc = do_mpc.controller.MPC(model)

        setup_mpc = {
            'n_horizon': self.mpc_horizon,
            't_step': 0.1,
            'n_robust': 0,
            'store_full_solution': True,
            'open_loop': 0,
        }

        mpc.set_param(**setup_mpc)

        mterm = (model.x['x'] - self.current_waypoint.pose.position.x) ** 2 + \
                (model.x['y'] - self.current_waypoint.pose.position.y) ** 2
        lterm = mterm

        mpc.set_objective(mterm=mterm, lterm=lterm)
        mpc.set_rterm(v=0.1, omega=0.1)

        mpc.bounds['lower', '_u', 'v'] = -self.max_vel_x
        mpc.bounds['upper', '_u', 'v'] = self.max_vel_x

        mpc.bounds['lower', '_u', 'omega'] = -self.max_vel_theta
        mpc.bounds['upper', '_u', 'omega'] = self.max_vel_theta

        mpc.setup()

        return mpc

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.current_velocity = msg.twist.twist

    def control_loop(self):
        if self.current_pose is None or self.current_waypoint is None:
            return

        if self.reached_waypoint():
            self.get_next_waypoint()

        if self.current_waypoint is None:
            return

        control_command = self.compute_control_command()
        self.cmd_vel_publisher.publish(control_command)

    def reached_waypoint(self):
        dx = self.current_waypoint.pose.position.x - self.current_pose.position.x
        dy = self.current_waypoint.pose.position.y - self.current_pose.position.y
        distance = np.sqrt(dx ** 2 + dy ** 2)
        return distance < self.lookahead_distance

    def get_next_waypoint(self):
        request = Waypoint.Request()
        future = self.waypoint_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response.success:
            self.current_waypoint = response.waypoint
        else:
            self.current_waypoint = None

    def compute_control_command(self):
        state = np.array([
            [self.current_pose.position.x],
            [self.current_pose.position.y],
            [self.get_yaw_from_quaternion(self.current_pose.orientation)]
        ])

        u0 = self.mpc.make_step(state)

        twist = Twist()
        twist.linear.x = float(u0[0])
        twist.angular.z = float(u0[1])

        return twist

    def get_yaw_from_quaternion(self, q):
        # Conversion from quaternion to yaw (rotation around z axis)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    node = MPCController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


import do_mpc
from casadi import *

def model():
    model_type = 'continuous'
    model = do_mpc.model.Model(model_type)

    # States
    x = model.set_variable(var_type='_x', var_name='x', shape=(1, 1))
    y = model.set_variable(var_type='_x', var_name='y', shape=(1, 1))
    theta = model.set_variable(var_type='_x', var_name='theta', shape=(1, 1))

    # Controls
    v = model.set_variable(var_type='_u', var_name='v', shape=(1, 1))
    omega = model.set_variable(var_type='_u', var_name='omega', shape=(1, 1))

    # Model equations
    model.set_rhs('x', v * cos(theta))
    model.set_rhs('y', v * sin(theta))
    model.set_rhs('theta', omega)

    model.setup()

    return model
 