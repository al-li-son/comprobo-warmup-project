"""Drive Neato in a square"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist # To control Neato motors

class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower_node')

        self.wall_error = 0
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.drive_msg)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(LaserScan, 'scan', self.get_wall_error, 10)
        self.k = .5
    
    def drive_msg(self):
        msg = Twist()
        msg.linear.x = .2
        msg.angular.z = - self.k * self.wall_error
        self.publisher.publish(msg)

    def get_wall_error(self, msg):
        self.wall_error = msg.ranges[315] - msg.ranges[225] if msg.ranges[270] < msg.ranges[90] else msg.ranges[135] - msg.ranges[45]


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()