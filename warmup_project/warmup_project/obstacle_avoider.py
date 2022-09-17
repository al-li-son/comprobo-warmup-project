"""Avoid obstacles with the Neato"""

from cmath import nan
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from neato2_interfaces.msg import Bump
from geometry_msgs.msg import Twist # To control Neato motors
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from visualization_msgs.msg import Marker
from scipy.stats import norm
import matplotlib.pyplot as plt

class ObstacleAvoiderNode(Node):
    def __init__(self):
        super().__init__('person_follower_node')

        self.error = 0.0
        self.x_avg = 0.0
        self.y_avg = 0.0
        self.collision = False
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.drive_msg)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(LaserScan, 'scan', self.get_turn_direction, 10)
        self.bump_subscriber = self.create_subscription(Bump, 'bump', self.hit_obstacle, 10)
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
    
    def hit_obstacle(self, msg):
        #if not self.collision:
            self.collision = msg.left_front or msg.left_side or msg.right_front or msg.right_side
    
    def drive_msg(self):
        move_msg = Twist()
        if self.collision:
            move_msg.linear.x = 0.0
            move_msg.angular.z = 0.0
        else:
            move_msg.linear.x = 0.2
            move_msg.angular.z = self.Kp * self.error
        
        self.publisher.publish(move_msg)

    def get_turn_direction(self, msg):
        scans = np.array([val if not np.isinf(val) else nan for val in msg.ranges])
        angles = np.linspace(-3,3,181)
        weights = norm.pdf(angles, loc=0, scale=1)
        scans_cropped = np.concatenate((np.flip(scans[0:91]), np.flip(scans[270:360])))
        scans_max = max(scans_cropped)
        scans_normalized= [val/scans_max if not np.isnan(val) else 1 for val in scans_cropped]
        scans_inverted = [val - 1 for val in scans_normalized]
        scans_scaled = weights + np.array(scans_inverted)
        turn_angle = np.argmax(scans_scaled)
        self.error = 90 - turn_angle


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoiderNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()