"""Example Publisher Node"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3 # To control Neato motors

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.send_msg)
        self.publisher = self.create_publisher(String, 'my_topic', 10)
    
    def send_msg(self):
        # msg = String(data = 'hello')
        msg = String()
        msg.data = "hello"
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()