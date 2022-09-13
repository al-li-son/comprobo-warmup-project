"""Drive Neato in a square"""

import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from neato2_interfaces.msg import Bump
from geometry_msgs.msg import Twist # To control Neato motors
from statistics import mean
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower_node')

        self.wall_error = 0
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.drive_msg)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(LaserScan, 'scan', self.get_wall_error, 10)
        self.declare_parameters(namespace='',
        parameters=[('Kp', 0.5)])
        self.Kp = self.get_parameter('Kp').value
        # the following line is only need to support dynamic_reconfigure
        self.add_on_set_parameters_callback(self.parameter_callback)
    
    def parameter_callback(self, params):
        """ This callback allows the parameters of the node to be adjusted
            dynamically by other ROS nodes (e.g., dynamic_reconfigure).
            This function is only needed to support dynamic_reconfigure. """
        for param in params:
            if param.name == 'Kp' and param.type_ == Parameter.Type.DOUBLE:
                self.Kp = param.value
        print(self.Kp)
        return SetParametersResult(successful=True)
    
    def drive_msg(self):
        msg = Twist()
        msg.linear.x = .2
        msg.angular.z = - self.Kp * self.wall_error
        self.publisher.publish(msg)

    def get_wall_error(self, msg):
        scans = msg.ranges
        right_front = mean([val for val in scans[310:320] if val != 0])
        right_back = mean([val for val in scans[220:230] if val != 0])
        left_front = mean([val for val in scans[40:50] if val != 0])
        left_back = mean([val for val in scans[130:140] if val != 0])
        right = mean([val for val in scans[265:275] if val != 0])
        left = mean([val for val in scans[85:95] if val != 0])
        self.wall_error = right_front - right_back if right < left else left_back - left_front
        print(f"{right_front=}\n{right_back=}\n{left_front=}\n{left_back=}")


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()