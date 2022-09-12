"""Drive Neato in a square"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist # To control Neato motors

class DriveSquareNode(Node):
    def __init__(self):
        super().__init__('drive_square_node')

        timer_period = 0.1
        self.start_timestamp = self.get_clock().now().nanoseconds
        self.timer = self.create_timer(timer_period, self.drive_msg)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
    
    def drive_msg(self):
        msg = Twist()
        current_time = self.get_clock().now().nanoseconds
        if current_time - self.start_timestamp < 3.3e9:
            msg.linear.x = 0.3
            msg.angular.z= 0.0
        elif current_time - self.start_timestamp < 6.6e9:
            msg.linear.x = 0.0
            msg.angular.z= 0.5
        elif current_time - self.start_timestamp < 9.9e9:
            msg.linear.x = 0.3
            msg.angular.z= 0.0
        elif current_time - self.start_timestamp < 13.2e9:
            msg.linear.x = 0.0
            msg.angular.z= 0.5
        elif current_time - self.start_timestamp < 16.5e9:
            msg.linear.x = 0.3
            msg.angular.z= 0.0
        elif current_time - self.start_timestamp < 19.8e9:
            msg.linear.x = 0.0
            msg.angular.z= 0.5
        elif current_time - self.start_timestamp < 23.1e9:
            msg.linear.x = 0.3
            msg.angular.z= 0.0
        elif current_time - self.start_timestamp < 26.4e9:
            msg.linear.x = 0.0
            msg.angular.z= 0.0

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DriveSquareNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()