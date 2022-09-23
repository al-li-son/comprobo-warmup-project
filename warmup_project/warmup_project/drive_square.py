"""Drive Neato in a square"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist # To control Neato motors

class DriveSquareNode(Node):
    def __init__(self):
        super().__init__('drive_square_node')

        # Set class attributes
        self.last_timestamp = self.get_clock().now().nanoseconds
        self.is_turning = False
        self.counter = 0

        # Create timer
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.drive_msg)

        # Create publishers/subscribers
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
    
    def drive_msg(self):
        """
        Publish linear and angular velocities to robot
        """
        msg = Twist()
        current_time = self.get_clock().now().nanoseconds
        # Check if all 4 sides of square have been run
        if self.counter >= 4:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        # Turn robot
        elif self.is_turning:
            if current_time - self.last_timestamp < 3.0e9:
                msg.linear.x = 0.0
                msg.angular.z = 0.6
            else:
                self.is_turning = False
                self.last_timestamp = current_time
        # Drive robot
        else:
            if current_time - self.last_timestamp < 3.33e9:
                msg.linear.x = 0.3
                msg.angular.z = 0.0
            else:
                self.is_turning = True
                self.last_timestamp = current_time
                self.counter += 1

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DriveSquareNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()