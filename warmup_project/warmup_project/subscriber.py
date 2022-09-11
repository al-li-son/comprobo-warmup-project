"""Example Subscriber Node"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscrberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscriber = self.create_subscription(String, 'my_topic', self.receive_msg, 10)
    
    def receive_msg(self, msg):
        print(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = SubscrberNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()