import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class Navigate(Node):
    def __init__(self):
        super().__init__('navigate')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'waypoints', 10)
        timer_period = 10.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.waypoints = [(1.0, 1.0), (2.0, 2.0), (3.0, 3.0)]
        self.index = 0

    def timer_callback(self):
        if self.index < len(self.waypoints):
            waypoint = self.waypoints[self.index]
            msg = Float32MultiArray(data=list(waypoint))
            self.publisher_.publish(msg)
            self.index += 1

def main(args=None):
    rclpy.init(args=args)
    navigate = Navigate()
    rclpy.spin(navigate)
    navigate.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
