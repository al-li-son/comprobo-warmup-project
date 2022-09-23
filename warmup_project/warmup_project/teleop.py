"""Teleop node to drive a Neato with a keyboard"""

import rclpy
import tty
import select
import sys
import termios
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')

        # Create timer to set velocity based on key
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.set_vel)

        # Create publishers/subscribers
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
    
    def set_vel(self):
        """
        Publish linear and angular velocity to robot
        """
        # Get current key
        key = self.getKey()  
        msg = Twist()
        
        # Control based on WASD + spacebar
        if key == 'w':
            msg.linear.x = 0.3
            msg.angular.z = 0.0
        elif key == 's':
            msg.linear.x = -0.3
            msg.angular.z = 0.0
        elif key == 'a':
            msg.linear.x = 0.0
            msg.angular.z = 0.8
        elif key == 'd':
            msg.linear.x = 0.0
            msg.angular.z = -0.8
        elif key == ' ':
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        # Exit if ctrl+C pressed
        elif key == '\x03':
            self.destroy_node()
        
        self.publisher.publish(msg)
    
    def getKey(self):
        """
        Get most recent key press
        """
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
