import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class Waypoints(Node):

    def __init__(self):
        super().__init__('waypoints')
        self.publisher_ = self.create_publisher(String, 'target', 10)
        timer_period = 0.5 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Pose2D()
        msg.x = 0
        msg.y = 0
        msg.theta = 0
        self.publisher_.publish(msg)